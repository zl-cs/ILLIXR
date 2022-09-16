#include <iostream>
#include <opencv2/core/mat.hpp>
#include <opencv2/opencv.hpp>

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libswscale/swscale.h>
#include "libavutil/opt.h"
#include "libavutil/imgutils.h"
#include <inttypes.h>
}

#include "phonebook.hpp"
#include "error_util.hpp"

#include <filesystem>
#include <fstream>

namespace ILLIXR {

class encoder : public phonebook::service {

public:
    encoder() {
        // Find and create encoder
        av_codec = avcodec_find_encoder_by_name("h264_nvenc");
        if (!av_codec) {
            ILLIXR::abort("Could not find encoder");
        } else {
            std::cout << "Found encoder " << av_codec->name << "\n";
        }

        // Create context
        av_codec_ctx = avcodec_alloc_context3(av_codec);
        if (!av_codec_ctx) {
            ILLIXR::abort("Failed to create context");
        } else {
            std::cout << "Created " << avcodec_get_name(av_codec_ctx->codec_id) << "\n";
        }

        // set encoder parameters
        av_codec_ctx->codec_type = AVMEDIA_TYPE_VIDEO;
        av_codec_ctx->width = 1504;
        av_codec_ctx->height = 480;
        av_codec_ctx->framerate = (AVRational) {20, 1};
        av_codec_ctx->time_base = (AVRational) {1, 20};

        // av_codec_ctx->compression_level = 0;

        // Let's aim for 1.1 Mbps with no B-Frames
        // av_codec_ctx->bit_rate = 0;
        
        // 0 indicates intra-only coding(iAVC)
        av_codec_ctx->gop_size = 0;
        // av_codec_ctx->max_b_frames = -1;

        // CUDA acceleration with yuv420 fallback
        av_codec_ctx->pix_fmt = AV_PIX_FMT_CUDA;
        av_codec_ctx->sw_pix_fmt = AV_PIX_FMT_YUV420P;

        // Encoder settings. Refer to https://www.ffmpeg.org/doxygen/3.2/nvenc__h264_8c_source.html for details
        int ret;
        // ret = av_opt_set(av_codec_ctx->priv_data, "preset", "p7", 0);
        ret = av_opt_set(av_codec_ctx->priv_data, "tune", "lossless", 0);
        // ret |= av_opt_set(av_codec_ctx->priv_data, "profile", "high", 0);
        ret |= av_opt_set(av_codec_ctx->priv_data, "rc", "vbr", 0);
        ret |= av_opt_set_int(av_codec_ctx->priv_data, "cq", 0, 0);
        // ret |= av_opt_set_int(av_codec_ctx->priv_data, "rc-lookahead", 0, 0);
        // ret |= av_opt_set_int(av_codec_ctx->priv_data, "qmin", 0, 0);
        // ret |= av_opt_set_int(av_codec_ctx->priv_data, "qmax", 1, 0);
        // ret |= av_opt_set_int(av_codec_ctx->priv_data, "qp", 1, 0);
        // ret |= av_opt_set(av_codec_ctx->priv_data, "forced-idr", "true", 0);
        // ret |= av_opt_set_int(av_codec_ctx->priv_data, "zerolatency", 1, 0);
        // ret |= av_opt_set_int(av_codec_ctx->priv_data, "delay", 0, 0);
        // ret |= av_opt_set_int(av_codec_ctx->priv_data, "temporal-aq", 1, 0);
        // ret |= av_opt_set_int(av_codec_ctx->priv_data, "spatial-aq", 1, 0);
        // ret |= av_opt_set_int(av_codec_ctx->priv_data, "aq-strength", 8, 0);

        if (ret != 0) {
            std::cerr << "Failed to set encoder settings\n";
            ILLIXR::abort(av_make_error(ret));
        } else {
            std::cout << "Successfully set encoder settings\n";
        }

        // Create hardware device context
        ret = av_hwdevice_ctx_create(&av_codec_ctx->hw_device_ctx, AV_HWDEVICE_TYPE_CUDA, NULL, NULL, 0);
        if (ret) {
            ILLIXR::abort("Failed to create hardware device context");
        } else {
            std::cout << "Created hardware device context\n";
        }

        // Create hardware frames context
        av_codec_ctx->hw_frames_ctx = av_hwframe_ctx_alloc(av_codec_ctx->hw_device_ctx);
        if (!av_codec_ctx->hw_frames_ctx) {
            ILLIXR::abort("Failed to create hardware frames context");
        } else {
            std::cout << "Created hardware frames context\n";
        }

        // Same as context settings
        AVHWFramesContext *frames_context = (AVHWFramesContext *) av_codec_ctx->hw_frames_ctx->data;
        frames_context->format = AV_PIX_FMT_CUDA;
        frames_context->sw_format = AV_PIX_FMT_YUV420P;
        frames_context->width = 1504;
        frames_context->height = 480;
        frames_context->device_ref = av_codec_ctx->hw_device_ctx;
        frames_context->device_ctx = (AVHWDeviceContext *) av_codec_ctx->hw_device_ctx->data;

        // Initialize hardware frames context
        ret = av_hwframe_ctx_init(av_codec_ctx->hw_frames_ctx);
        if (ret) {
            ILLIXR::abort("Failed to initialize hardware frames context");
        } else {
            std::cout << "Initialized hardware frames context\n";
        }

        // Initialize the VCodecContent
        ret = avcodec_open2(av_codec_ctx, av_codec, NULL);
        if (ret) {
            ILLIXR::abort("Failed to initialize context");
        } else {
            std::cout << "Initialized context\n";
        }

        // Allocate hw frame
        hw_frame = av_frame_alloc();
        hw_frame->format = av_codec_ctx->pix_fmt;
        hw_frame->width = av_codec_ctx->width;
        hw_frame->height = av_codec_ctx->height;

        // Attach frame to context
        ret = av_hwframe_get_buffer(av_codec_ctx->hw_frames_ctx, hw_frame, 0);
        if (ret) {
            ILLIXR::abort("Failed to attach frame to context");
        } else {
            std::cout << "Attached frame to context\n";
        }

        // Initialize the frame to be encoded
        img_frame = av_frame_alloc();
        img_frame->format = av_codec_ctx->sw_pix_fmt;
        img_frame->width = av_codec_ctx->width;
        img_frame->height = av_codec_ctx->height;
        if(img_frame == NULL){
            std::cerr << "Failed allocating a frame\n";
            ILLIXR::abort(av_make_error(ret));
        } else {
            std::cout << "Allocated img_frame\n";
        }
        frame_size = img_frame->width * img_frame->height;

        img_packet = av_packet_alloc();
        if (img_packet == NULL) {
            std::cerr << "Failed allocating a packet\n";
            ILLIXR::abort();
        } else {
            std::cout << "Allocated img_packet\n";
        }
        img_packet->size = 0;
        img_packet->data = NULL;

        // get ref-count buffer to save mem copy, but not sure if this works on non-CPU
        // av_frame_get_buffer(img_frame, 0);
        // if (ret < 0) {
        //     std::cerr << "Failed getting a frame buffer\n";
        //     ILLIXR::abort(av_make_error(ret));
        // }

        if (!std::filesystem::exists(data_path)) {
			if (!std::filesystem::create_directory(data_path)) {
				std::cerr << "Failed to create data directory.";
			}
		}
        // cam0_mat.open(data_path + "/cam0_mat.txt");
        // cam0_frame.open(data_path + "/cam0_frame.txt");

        // prewarm the encoder
        // cv::Mat cam0 = cv::imread(std::filesystem::current_path().string() + "/common/cam0.png", cv::IMREAD_GRAYSCALE);
        // encode(cam0, img_packet);

    }

    // av_err2str returns a temporary array. This doesn't work in gcc.
    // This function can be used as a replacement for av_err2str.
    static const char* av_make_error(int errnum) {
        static char str[AV_ERROR_MAX_STRING_SIZE];
        memset(str, 0, sizeof(str));
        return av_make_error_string(str, AV_ERROR_MAX_STRING_SIZE, errnum);
    }

    bool encode(cv::Mat& img, AVPacket *encoded) {
        int ret; // catch return errors
        int width = img.cols;
        int height = img.rows;
        std::cout << "encode image width " << width << " height " << height << std::endl;

        // convert img (cv::Mat) to img_frame (AVFrame)
        int cvLinesizes[1];
        cvLinesizes[0] = img.step1();
        ret = av_image_alloc(img_frame->data, img_frame->linesize, width, height, AVPixelFormat::AV_PIX_FMT_YUV420P, 1);
        if (ret < 0) {
            std::cerr << "Failed allocating a packet\n";
            ILLIXR::abort();
        } else {
            std::cout << "Allocated image buffer (size " << ret << "bytes)\n";
        }
        // img_frame->buf[0] = av_buffer_alloc(frame_size);
        SwsContext *conversion = sws_getContext(width, height, AVPixelFormat::AV_PIX_FMT_GRAY8, width, height, AVPixelFormat::AV_PIX_FMT_YUV420P, SWS_SPLINE, NULL, NULL, NULL);
        sws_scale(conversion, &img.data, cvLinesizes, 0, height, img_frame->data, img_frame->linesize);
        sws_freeContext(conversion);

        // cam0_mat << img.rowRange(0, 10) << "\n";
        img_frame->data[0] = (uint8_t *)img.data;
        // std::cout << "img_frame width after sws_scale: " << img_frame->width << "\n";
        // std::string space;
        // for (int i = 0; i < height; i++) {
        //     for (int j = 0; j < width; j++) {
        //         cam0_mat << img.row(i).col(j) << ", ";
        //         space = +(*(img_frame->data[0] + i*width + j)) < 100 ? " " : "";
        //         cam0_frame << "[" << space << +(*(img_frame->data[0] + i*width + j)) << "], ";
        //     }
        //     cam0_mat << "\n";
        //     cam0_frame << "\n";
        // }

        // Confirmed that after sws_scale, the U and V planes are with 128, and the remaining planes are NULL
        // if (img_frame->data[2] != NULL) {
        //     for (int i = 0; i < 10; i++) {
        //         for (int j = 0; j < width; j++) {
        //             space = +(*(img_frame->data[2] + i*width + j)) < 100 ? " " : "";
        //             cam0_frame << "[" << space << +(*(img_frame->data[2] + i*width + j)) << "], ";
        //         }
        //         cam0_frame << "\n";
        //     }
        // } else {
        //     std::cout << "data[2] is NULL\n";
        // }
        // assert(img_frame->data[3] == NULL);

        ret = av_hwframe_transfer_data(hw_frame, img_frame, 0);
        if(ret < 0){
            ILLIXR::abort("sw->hw transfer not successful\n");
        }

        ret = avcodec_send_frame(av_codec_ctx, hw_frame);
        if (ret < 0) {
            std::cerr << "Failed sending a frame\n";
            ILLIXR::abort(av_make_error(ret));
        }

        ret = avcodec_send_frame(av_codec_ctx, NULL);
        if (ret < 0) {
            std::cerr << "Failed sending a NULL to flush the buffer\n";
            ILLIXR::abort(av_make_error(ret));
        }
        ret = avcodec_receive_packet(av_codec_ctx, img_packet);
        if(ret == AVERROR(EAGAIN) || ret == AVERROR_EOF) {
            std::cout << "EAGAIN or EOF\n";
        }
        // while (1) {
        //     ret = avcodec_receive_packet(av_codec_ctx, img_packet);
        //     if(ret == AVERROR(EAGAIN) || ret == AVERROR_EOF) {
        //         std::cout << "EAGAIN or EOF\n";
        //         avcodec_send_frame(av_codec_ctx, hw_frame);
        //         continue;
        //     } else if (ret < 0) {
        //         std::cout << "Error during encoding\n";
        //         ILLIXR::abort(av_make_error(ret));
        //     } else { // success
        //         break;
        //     }
        // }
        // if (img_packet->side_data != NULL)
        //     std::cout << "The compressed packet has side data! The size is " << img_packet->side_data->size << "\n";
        // else
        //     std::cout << "The compressed packet has NO side data!\n";

        // CONVERSTION AVPACKET TO CV::MAT GIVES CORRUPTED IMAGES
        // cv::Mat img_encoded(height, width, CV_8UC1);
        // SwsContext *conversion_back = sws_getContext(width, height, AVPixelFormat::AV_PIX_FMT_YUV420P, width, height, AVPixelFormat::AV_PIX_FMT_GRAY8, SWS_BICUBIC, NULL, NULL, NULL);
        // sws_scale(conversion_back, &img_packet->data, img_frame->linesize, 0, height, &img_encoded.data, cvLinesizes);
        // sws_freeContext(conversion_back);

        // std::cout << "img_packet size is " << img_packet->size << "\n";
        // cv::Mat img_encoded(height, width, CV_8UC1, (void*)img_packet->data);
        // cv::imshow("img_encoded", img_encoded);
        // cv::waitKey(1);

        // OUTPUT AVPACKET TO FILES
        // std::string seq_num = std::to_string(image_count++);
        // std::string final_loc = data_path + "/" + seq_num + ".png";
        // f.open(final_loc.c_str(), std::ios::out | std::ios::binary);
        // f << *(img_packet->data) << std::endl;
        // f.close();

        // if (encoded) av_packet_unref(encoded);
        ret = av_packet_ref(encoded, img_packet);
        avcodec_flush_buffers(av_codec_ctx);
        if (ret < 0) {
            std::cerr << "Failed add ref to the encoded packet\n";
            ILLIXR::abort(av_make_error(ret));
        } else {
            std::cout << "SUCCESS: Encoded one frame\n";
            return true;
        }

        return false;
    }

    ~encoder() {
        av_packet_free(&img_packet);
        av_frame_free(&img_frame);
        av_frame_free(&hw_frame);
        avcodec_free_context(&av_codec_ctx);
    }

private:

    const AVCodec* av_codec;
    AVCodecContext* av_codec_ctx;
    AVFrame *img_frame, *hw_frame;
    AVPacket *img_packet;
    size_t frame_size;
    
    // int image_count = 0;
    const std::string data_path = std::filesystem::current_path().string() + "/encoded_images";
    std::ofstream cam0_mat;
    std::ofstream cam0_frame;
    // std::ofstream f;

};

}
