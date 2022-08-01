#include <ecal/ecal.h>
#include <ecal/msg/protobuf/publisher.h>
#include <opencv2/core/mat.hpp>
#include <opencv2/opencv.hpp>
#include "common/data_format.hpp"
#include "common/phonebook.hpp"
#include "common/plugin.hpp"
#include "common/switchboard.hpp"

#include "vio_input.pb.h"
// ffmpeg
extern "C" {
    #include "libavcodec/avcodec.h"
    #include "libavutil/avutil.h"
    #include "libavutil/frame.h"
    #include "libavutil/hwcontext.h"
    #include "libavutil/hwcontext_cuda.h"
    #include "libavutil/imgutils.h"
    #include "libavutil/opt.h"
    #include "libavutil/pixfmt.h"
    #include "libswscale/swscale.h"
}

using namespace ILLIXR;

class offload_writer : public plugin {
public:
    offload_writer(std::string name_, phonebook* pb_)
        : plugin{name_, pb_}
        , sb{pb->lookup_impl<switchboard>()}
    {
        int ret;

        // Find encoder
        const AVCodec *encoder = avcodec_find_encoder_by_name("h264_nvenc");
        if (!encoder) {
            ILLIXR::abort("Could not find encoder");
        } else {
            std::cout << "\n Found encoder " << encoder->name << "\n";
        }

        // Create context
        context = avcodec_alloc_context3(encoder);
        if (!context) {
            ILLIXR::abort("Failed to create context");
        } else {
            std::cout << "Created " << avcodec_get_name(context->codec_id) << "\n";
        }

        // EuRoC is 752x480 at 20 fps
        context->codec_type = AVMEDIA_TYPE_VIDEO;
        context->width = 752;
        context->height = 480;
        context->framerate = (AVRational) {20, 1};
        context->time_base = (AVRational) {1, 20};

        // Let's aim for 1.1 Mbps with no B-Frames
        //context->bit_rate = 1100000;
        context->bit_rate = 1000000;
        //context->bit_rate = 800000;
        //context->bit_rate = 600000;
        //context->bit_rate = 400000;
        //context->bit_rate = 200000;
        //context->bit_rate = 100000;
        //context->bit_rate = 50000;
        //0 indicates intra-only coding(iAVC)
        context->gop_size = 0;
        context->max_b_frames = 0;

        // CUDA acceleration with yuv420 fallback
        context->pix_fmt = AV_PIX_FMT_CUDA;
        context->sw_pix_fmt = AV_PIX_FMT_YUV420P;

        // Encoder settings. Refer to https://www.ffmpeg.org/doxygen/3.2/nvenc__h264_8c_source.html for details
        ret = av_opt_set(context->priv_data, "preset", "medium", 0);
        ret |= av_opt_set(context->priv_data, "rc", "vbr", 0);
        ret |= av_opt_set(context->priv_data, "forced-idr", "true", 0);
        ret |= av_opt_set_int(context->priv_data, "zerolatency", 1, 0);
        ret |= av_opt_set_int(context->priv_data, "delay", 0, 0);
        ret |= av_opt_set_int(context->priv_data, "temporal-aq", 1, 0);
        ret |= av_opt_set_int(context->priv_data, "spatial-aq", 1, 0);
        ret |= av_opt_set_int(context->priv_data, "aq-strength", 8, 0);

        if (ret) {
            ILLIXR::abort("Failed to set encoder settings");
        } else {
            std::cout << "Successfully set encoder settings\n";
        }

        // Create hardware device context
        ret = av_hwdevice_ctx_create(&context->hw_device_ctx, AV_HWDEVICE_TYPE_CUDA, NULL, NULL, 0);
        if (ret) {
            ILLIXR::abort("Failed to create hardware device context");
        } else {
            std::cout << "Created hardware device context\n";
        }

        // Create hardware frames context
        context->hw_frames_ctx = av_hwframe_ctx_alloc(context->hw_device_ctx);
        if (!context->hw_frames_ctx) {
            ILLIXR::abort("Failed to create hardware frames context");
        } else {
            std::cout << "Created hardware frames context\n";
        }

        // Same as context settings
        AVHWFramesContext *frames_context = (AVHWFramesContext *) context->hw_frames_ctx->data;
        frames_context->format = AV_PIX_FMT_CUDA;
        frames_context->sw_format = AV_PIX_FMT_YUV420P;
        frames_context->width = 752;
        frames_context->height = 480;
        frames_context->device_ref = context->hw_device_ctx;
        frames_context->device_ctx = (AVHWDeviceContext *) context->hw_device_ctx->data;

        // Initialize hardware frames context
        ret = av_hwframe_ctx_init(context->hw_frames_ctx);
        if (ret) {
            ILLIXR::abort("Failed to initialize hardware frames context");
        } else {
            std::cout << "Initialized hardware frames context\n";
        }

        // Initialize the VCodecContent
        ret = avcodec_open2(context, encoder, NULL);
        if (ret) {
            ILLIXR::abort("Failed to initialize context");
        } else {
            std::cout << "Initialized context\n";
        }

        // Allocate frame
        frame = av_frame_alloc();
        frame->format = context->pix_fmt;
        frame->width = context->width;
        frame->height = context->height;

        // Attach frame to context
        ret = av_hwframe_get_buffer(context->hw_frames_ctx, frame, 0);
        if (ret) {
            ILLIXR::abort("Failed to attach frame to context");
        } else {
            std::cout << "Attached frame to context\n";
        }

        //Allocate av packet
        img0_pkt = av_packet_alloc();
        
        //Initialize temp buffer for img0
        img0_swframe = av_frame_alloc();
        img0_swframe->format = context->sw_pix_fmt;
        img0_swframe->width = context->width;
        img0_swframe->height = context->height;
        if(ret < 0){
            std::cout<<"can not get new buffer\n";
            exit(1);
        }
        
        //testing: Initialize a decoder
        //const AVCodec *decoder = avcodec_find_decoder_by_name("h264_cuvid");
        //dec_context = avcodec_alloc_context3(decoder);
        //// Create hardware device context
        //ret = av_hwdevice_ctx_create(&dec_context->hw_device_ctx, AV_HWDEVICE_TYPE_CUDA, NULL, NULL, 0);
        //if (ret) {
        //    ILLIXR::abort("Failed to create hardware device context");
        //} else {
        //    std::cout << "Created hardware device context\n";
        //}
        //ret = avcodec_open2(dec_context, decoder, NULL);
        //if (ret) {
        //    ILLIXR::abort("Failed to open context");
        //} else {
        //    std::cout << "opened codec context\n";
        //}
        
        // Initialize eCAL
        eCAL::Initialize(0, NULL, "VIO Device Transmitter");
        publisher = eCAL::protobuf::CPublisher<vio_input_proto::IMUCamVec>("vio_input");
        publisher.SetLayerMode(eCAL::TLayer::tlayer_udp_mc, eCAL::TLayer::smode_off);
        publisher.SetLayerMode(eCAL::TLayer::tlayer_tcp, eCAL::TLayer::smode_auto);
    }

    virtual ~offload_writer() override {
        
        av_frame_free(&frame);
        avcodec_free_context(&context);
    }

    virtual void start() override {
        plugin::start();

        sb->schedule<imu_cam_type_prof>(id, "imu_cam", [this](switchboard::ptr<const imu_cam_type_prof> datum, std::size_t) {
            this->send_imu_cam_data(datum);
        });
    }

    void send_imu_cam_data(switchboard::ptr<const imu_cam_type_prof> datum) {
        // Ensures that slam doesnt start before valid IMU readings come in
        if (datum == nullptr) {
            assert(previous_timestamp == 0);
            return;
        }


        //pyh add encoding
        //modified from https://gist.github.com/foowaa/1d296a9dee81c7a2a52f291c95e55680
        vio_input_proto::IMUCamData* imu_cam_data = data_buffer->add_imu_cam_data();

        if(datum->img0.has_value())
        {
            int width = datum->img0->cols;
            int height = datum->img0->rows;
            
            int cvLinesizes[1];
            cvLinesizes[0] = datum->img0->step1();
            //img0_swframe = av_frame_alloc();
            
            ////allocate an image with size w and h and pixel format pix_fmt and fill pointers and linesizes accordingly
            ////0RGB32 corresponds to ARGB, 1 means align the value to use for buffer size alignment
            av_image_alloc(img0_swframe->data, img0_swframe->linesize, width, height, AVPixelFormat::AV_PIX_FMT_YUV420P, 1);
            ////sws -> ffmpeg color conversion and scaling libbrary?
            ////allocate and return an SwsContext, one need it to perform scaling/conversion operations using sws_scale
            ////parameters: src_width, src_height, srcFormat, dest_width, dest_height, dest_format, algo to rescale, src_filter, dst_filter, extra param
            SwsContext *conversion = sws_getContext(width,height,AVPixelFormat::AV_PIX_FMT_GRAY8,width,height, (AVPixelFormat)AV_PIX_FMT_YUV420P,SWS_BICUBIC,NULL,NULL,NULL);
            ////scale the image slice in srcSlice, and put the resulting scaled slice in the image in dst
            ////params:
            ////  1. swscontext
            ////  2. srcSlice(array containing the pointers to the planes of source slice)
            ////  3. array containing the strides for each plane of the source image
            ////  4. the position in the source image of the slice to process, that is the number (coutned starting from zero) in the image of the first row of the slice
            ////  5. the height of the source slice, that is the number of rows in the slice,
            ////  6. dst, the array containing the pointers to the plane of the destination image
            ////  7. dstStride, the array containing the strides of each plane of the destination image
            //sws_scale(conversion, &datum->img0->data, cvLinesizes, 0, height, img0_swframe->data, img0_swframe->linesize);
            //dumping img1
            sws_scale(conversion, &datum->img1->data, cvLinesizes, 0, height, img0_swframe->data, img0_swframe->linesize);
            sws_freeContext(conversion);
         
            int ret;
            fflush(stdout);
            ret = av_frame_is_writable(frame);
            
            if(ret < 0){
                std::cout<<"frame not writeable\n";
                exit(1);
            } 
            //dump frame to image
            std::string seq_num = std::to_string(image_count);
            //std::string convert_loc =  image_convert_loc + seq_num + ".yuv";
            //f_convert = fopen(convert_loc.c_str(),"wb"); 
            //for(int i=0; i < img0_swframe->height; i++)
            //{        
            //    fwrite(img0_swframe->data[0] + i * img0_swframe->linesize[0], 1, img0_swframe->width, f_convert);        
            //}
            //fclose(f_convert);
            
            //convert img0_swframe back to cv::Mat for sanity check
            //std::string ori_loc =  image_ori_loc + seq_num + ".png";
            //cv::Mat image(img0_swframe->height, img0_swframe->width, CV_8UC3);
            //SwsContext* conversion = sws_getContext(img0_swframe->width, img0_swframe->height, (AVPixelFormat) img0_swframe->format, img0_swframe->width, img0_swframe->height, AVPixelFormat::AV_PIX_FMT_BGR24, SWS_FAST_BILINEAR, NULL, NULL, NULL);  
            //int cvLineSizes[1];
            //cvLineSizes[0] = image.step1();
            //sws_scale(conversion, img0_swframe->data, img0_swframe->linesize, 0, img0_swframe->height, &image.data,cvLineSizes);  
            //sws_freeContext(conversion);
            //cv::imwrite(ori_loc,image);
            
            //transfer from or to a hw surface, first param dst
            ret = av_hwframe_transfer_data(frame,img0_swframe,0);
            if(ret < 0){
                std::cout<<"sw->hw transfer not successful\n";
                exit(1);
            }
            
            //encode every frame of video
            //supply a raw video frame to encoder
            ret = avcodec_send_frame(context,frame);
            if(ret < 0)
            {
                std::cout<<"error sending a frame for encoding\n";
                exit(1);
            }

            //std::cout<<"received a packet\n";
            std::string final_loc =  image_generating_loc + seq_num + ".h264";
            std::cout<<"writing encoded images to "<<final_loc<<"\n";
            f = fopen(final_loc.c_str(),"wb"); 
            
            //read encoded data from the coder
            while( ret>=0)
            {
                ret = avcodec_receive_packet(context,img0_pkt);
                if(ret == AVERROR(EAGAIN) || ret == AVERROR_EOF)
                    continue;
                else if (ret < 0){
                    std::cout<<"Error during encoding\n";
                }
                printf("Write packet %d (size=%5d)\n", image_count, img0_pkt->size);
                encoded_size+=img0_pkt->size;
                //pyh add pkt to ecal message
                imu_cam_data->set_img0_pkt((void*) img0_pkt->data, img0_pkt->size);
                fwrite(img0_pkt->data, 1, img0_pkt->size, f);
                fclose(f);
                //av_packet_unref(img0_pkt);

            }
            //pyh now img0_pkt suppose to have the encoded image
            //every imu reading generates an image
            image_count++;
            //fclose(f);
            //printf("one frame done\n"); 

        }   
        
        assert(datum->time.time_since_epoch().count() > previous_timestamp);
        previous_timestamp = datum->time.time_since_epoch().count();

        imu_cam_data->set_timestamp(datum->time.time_since_epoch().count());

        vio_input_proto::Vec3* angular_vel = new vio_input_proto::Vec3();
        angular_vel->set_x(datum->angular_v.x());
        angular_vel->set_y(datum->angular_v.y());
        angular_vel->set_z(datum->angular_v.z());
        imu_cam_data->set_allocated_angular_vel(angular_vel);

        vio_input_proto::Vec3* linear_accel = new vio_input_proto::Vec3();
        linear_accel->set_x(datum->linear_a.x());
        linear_accel->set_y(datum->linear_a.y());
        linear_accel->set_z(datum->linear_a.z());
        imu_cam_data->set_allocated_linear_accel(linear_accel);

        if (!datum->img0.has_value() && !datum->img1.has_value()) {
           imu_cam_data->set_rows(-1);
           imu_cam_data->set_cols(-1);

        } else {
           cv::Mat img0{(datum->img0.value()).clone()};
           cv::Mat img1{(datum->img1.value()).clone()};

           imu_cam_data->set_rows(img0.rows);
           imu_cam_data->set_cols(img0.cols);

           imu_cam_data->set_img0_data((void*) img0.data, img0.rows * img0.cols);
           imu_cam_data->set_img1_data((void*) img1.data, img1.rows * img1.cols);


            data_buffer->set_real_timestamp(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch()).count());
            data_buffer->set_dataset_timestamp(datum->dataset_time.time_since_epoch().count());
            data_buffer->set_frame_id(frame_id);
            frame_id++;

            
            
            //this is where ecal send
            
            publisher.Send(*data_buffer);
            delete data_buffer;
            data_buffer = new vio_input_proto::IMUCamVec();
            std::cout<<"current encoded total size: "<<encoded_size<<"\n";
        }
    
    }

private:
    long previous_timestamp = 0;
    int frame_id = 0;
    vio_input_proto::IMUCamVec* data_buffer = new vio_input_proto::IMUCamVec();

    const std::shared_ptr<switchboard> sb;
    eCAL::protobuf::CPublisher<vio_input_proto::IMUCamVec> publisher;

    AVFrame *img0_swframe, *img0_hwframe;
    AVPacket *img0_pkt;

    AVCodecContext *context;
    AVCodecContext *dec_context;
    AVFrame *frame;

    int encoded_size=0;
    int image_count=0;
    std::string image_generating_loc = "encoded_images/";
    std::string image_ori_loc = "original_images/";
    std::string image_convert_loc = "converted_images/";
    FILE *f;
    FILE *f_convert;
};

PLUGIN_MAIN(offload_writer)
