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

class decoder : public phonebook::service {

public:
    decoder() {
        int ret;

        // Find and create decoder
        av_codec = avcodec_find_decoder_by_name("h264_cuvid");
        if (!av_codec) {
            ILLIXR::abort("Could not find decoder");
        } else {
            std::cout << "Found decoder " << av_codec->name << "\n";
        }

        // Create context
        av_codec_ctx = avcodec_alloc_context3(av_codec);
        if (!av_codec_ctx) {
            ILLIXR::abort("Failed to create context");
        } else {
            std::cout << "Created " << avcodec_get_name(av_codec_ctx->codec_id) << "\n";
        }

        av_codec_ctx->width = 1504;
        av_codec_ctx->height = 480;
        av_codec_ctx->framerate = (AVRational) {20, 1};

        // yihan 
        av_codec_ctx->codec_id = AV_CODEC_ID_H264;
        // av_codec_ctx->hw_device_ctx = av_buffer_ref(device_ref);
        // av_codec_ctx->pix_fmt = static_cast<AVPixelFormat>(static_cast<int>(AV_PIX_FMT_NONE)+120);

        av_codec_ctx->pkt_timebase = (AVRational) {1, 1};

        // CUDA acceleration with yuv420 fallback
        av_codec_ctx->pix_fmt = AV_PIX_FMT_CUDA;
        av_codec_ctx->sw_pix_fmt = AV_PIX_FMT_YUV420P;

        // Create hardware device context
        // ret = av_hwdevice_ctx_create(&av_codec_ctx->hw_device_ctx, AV_HWDEVICE_TYPE_CUDA, NULL, NULL, 0);
        // if (ret) {
        //     ILLIXR::abort("Failed to create hardware device context");
        // } else {
        //     std::cout << "Created hardware device context\n";
        // }

        // AVPixelFormat decoded_fmt = av_codec_ctx->get_format(av_codec_ctx, formats);
        // if (decoded_fmt == AV_PIX_FMT_NONE) {
        //     std::cerr << "No proper format for decoder\n";
        //     ILLIXR::abort();
        // } else {
        //     std::cout << "Format for decoder " << decoded_fmt << std::endl;
        //     std::cout << "AV_PIX_FMT_CUDA " << AV_PIX_FMT_CUDA << std::endl;
        //     std::cout << "AV_PIX_FMT_YUV420P " << AV_PIX_FMT_YUV420P << std::endl;
        // }

        // // Create hardware frames context
        // av_codec_ctx->hw_frames_ctx = av_hwframe_ctx_alloc(av_codec_ctx->hw_device_ctx);
        // if (!av_codec_ctx->hw_frames_ctx) {
        //     ILLIXR::abort("Failed to create hardware frames context");
        // } else {
        //     std::cout << "Created hardware frames context\n";
        // }

        // // Same as context settings
        // AVHWFramesContext *frames_context = (AVHWFramesContext *) av_codec_ctx->hw_frames_ctx->data;
        // frames_context->format = AV_PIX_FMT_CUDA;
        // frames_context->sw_format = AV_PIX_FMT_YUV420P;
        // frames_context->width = 752;
        // frames_context->height = 480;
        // frames_context->device_ref = av_codec_ctx->hw_device_ctx;
        // frames_context->device_ctx = (AVHWDeviceContext *) av_codec_ctx->hw_device_ctx->data;

        // // Initialize hardware frames context
        // ret = av_hwframe_ctx_init(av_codec_ctx->hw_frames_ctx);
        // if (ret) {
        //     ILLIXR::abort("Failed to initialize hardware frames context");
        // } else {
        //     std::cout << "Initialized hardware frames context\n";
        // }

        // Initialize the VCodecContent
        ret = avcodec_open2(av_codec_ctx, av_codec, NULL);
        if (ret) {
            ILLIXR::abort("Failed to initialize context");
        } else {
            std::cout << "Initialized context\n";
        }

        decoded_frame = av_frame_alloc();
        if (decoded_frame == NULL) {
            std::cerr << "Failed allocating the decoded frame\n";
            ILLIXR::abort();
        }

        // // Attach frame to context
        // ret = av_hwframe_get_buffer(av_codec_ctx->hw_frames_ctx, decoded_frame, 0);
        // if (ret) {
        //     ILLIXR::abort("Failed to attach frame to context");
        // } else {
        //     std::cout << "Attached frame to context\n";
        // }

        sw_frame = av_frame_alloc();
        if (sw_frame == NULL) {
            std::cerr << "Failed allocating the sw frame\n";
            ILLIXR::abort();
        }
        sw_frame->format = av_codec_ctx->sw_pix_fmt;
        sw_frame->width = av_codec_ctx->width;
        sw_frame->height = av_codec_ctx->height;

        if (!std::filesystem::exists(data_path)) {
			if (!std::filesystem::create_directory(data_path)) {
				std::cerr << "Failed to create data directory.";
			}
		}
        cam0_decoded.open(data_path + "/cam0_decoded.txt");
        cam0_scaled.open(data_path + "/cam0_scaled.txt");
    }

    ~decoder() {
        av_frame_free(&decoded_frame);
        av_frame_free(&sw_frame);
        avcodec_free_context(&av_codec_ctx);
    }

    // av_err2str returns a temporary array. This doesn't work in gcc.
    // This function can be used as a replacement for av_err2str.
    static const char* av_make_error(int errnum) {
        static char str[AV_ERROR_MAX_STRING_SIZE];
        memset(str, 0, sizeof(str));
        return av_make_error_string(str, AV_ERROR_MAX_STRING_SIZE, errnum);
    }

    std::unique_ptr<cv::Mat> decode(AVPacket *input_pkt) {
        int width = 1504;
        int height = 480;

        int ret;
        // ret = av_image_alloc(decoded_frame->data, decoded_frame->linesize, width, height, AVPixelFormat::AV_PIX_FMT_YUV420P, 1);

        ret = avcodec_send_packet(av_codec_ctx, input_pkt);
        if (ret < 0) {
            std::cerr << "Failed sending a packet\n";
            ILLIXR::abort(av_make_error(ret));
        }
        ret = avcodec_send_packet(av_codec_ctx, NULL);
        if (ret < 0) {
            std::cerr << "Failed sending a NULL to flush the buffer\n";
            ILLIXR::abort(av_make_error(ret));
        }
        ret = avcodec_receive_frame(av_codec_ctx, decoded_frame);
        if(ret == AVERROR(EAGAIN) || ret == AVERROR_EOF) {
            std::cout << "EAGAIN or EOF in decoder\n";
        }
        // while (1) {
        //     ret = avcodec_receive_frame(av_codec_ctx, decoded_frame);
        //     if(ret == AVERROR(EAGAIN) || ret == AVERROR_EOF) {
        //         std::cout << "EAGAIN or EOF in decoder\n";
        //         avcodec_send_packet(av_codec_ctx, input_pkt);
        //         continue;
        //     } else if (ret < 0) {
        //         std::cout << "Error during encoding\n";
        //         ILLIXR::abort(av_make_error(ret));
        //     } else { // success
        //         break;
        //     }
        // }

        // ret = av_hwframe_transfer_data(sw_frame, decoded_frame, 0);
        // if(ret < 0){
        //     ILLIXR::abort("hw->sw transfer not successful\n");
        // }

        char fmt_str[100];
        av_get_pix_fmt_string(fmt_str, 100, static_cast<AVPixelFormat>(decoded_frame->format));
        std::cout << "frame received format string : " << fmt_str << "\n";
        // if (decoded_frame->format != AVPixelFormat::AV_PIX_FMT_CUDA) {
        //     std::cerr << "The format of the decoded frame is not CUDA, but " << decoded_frame->format << "\n";
        //     ILLIXR::abort();
        // }

        cv::Mat img_decoded_padding(height, width+32, CV_8UC1);
        int cvLinesizes[1];
        cvLinesizes[0] = img_decoded_padding.step1();
        SwsContext *conversion_back = sws_getContext(width, height, (AVPixelFormat)decoded_frame->format, width, height, AVPixelFormat::AV_PIX_FMT_GRAY8, SWS_SPLINE, NULL, NULL, NULL);
        sws_scale(conversion_back, decoded_frame->data, decoded_frame->linesize, 0, height, &img_decoded_padding.data, cvLinesizes);
        sws_freeContext(conversion_back);

        // unsigned long long start_copy = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
        // std::cout << "decoded_frame width after sws_scale: " << decoded_frame->width << "\n";
        // int padding_width = width + 32;
        // for (int r = 0; r < height; r++) {
        //     for (int c = 0; c < padding_width; c++) {
        //         if (c < width)
        //             img_decoded_padding.row(r).col(c) = *(decoded_frame->data[0] + r*padding_width + c);
        //     }
        // }
        // cam0_scaled << img_decoded_padding << std::endl;
        img_decoded_padding.data = decoded_frame->data[0];
        // if (count++ < 10)
        //     cam0_decoded << img_decoded_padding << std::endl;
        auto img_decoded = std::unique_ptr<cv::Mat>(new cv::Mat(img_decoded_padding.colRange(0, 1504)));
        // std::cout << "Decoder Mat Copy Time: " << std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch()).count() - start_copy << "\n";

        // img_decoded->data = decoded_frame->data[0];
        // std::string space;
        // for (int i = 0; i < height; i++) {
        //     for (int j = 0; j < width; j++) {
        //         cam0_scaled << img_decoded->row(i).col(j) << ", ";
        //         space = +(*(decoded_frame->data[0] + i*width + j)) < 100 ? " " : "";
        //         cam0_decoded << "[" << space << +(*(decoded_frame->data[0] + i*width + j)) << "], ";
        //     }
        //     cam0_scaled << "\n";
        //     cam0_decoded << "\n";
        // }

        // if (count++ % 2 == 0) {
        //     cv::imshow("img0_decoded", *img_decoded);
        //     cv::waitKey(1);
        // } else {
        //     cv::imshow("img1_decoded", *img_decoded);
        //     cv::waitKey(1);
        // }

        // cv::imshow("merged_decoded", *img_decoded);
        // cv::waitKey(1);

        avcodec_flush_buffers(av_codec_ctx);

        return img_decoded;
    }
private:
    const AVCodec* av_codec;
    AVCodecContext* av_codec_ctx;
    AVFrame *decoded_frame, *sw_frame;
    // const AVPixelFormat formats[3] = {AV_PIX_FMT_CUDA, AV_PIX_FMT_YUV420P, AV_PIX_FMT_NONE};
    // int count = 0;

    const std::string data_path = std::filesystem::current_path().string() + "/encoded_images";
    std::ofstream cam0_decoded;
    std::ofstream cam0_scaled;

};

}