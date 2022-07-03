#include "common/plugin.hpp"
#include "common/switchboard.hpp"
#include "common/data_format.hpp"
#include "common/phonebook.hpp"

#include <ecal/ecal.h>
#include <ecal/msg/protobuf/publisher.h>
#include <opencv2/core/mat.hpp>

#include "vio_input.pb.h"

//pyh libraries for ffmpeg
#include <cuda.h>
extern "C" {
    #include "libavutil/avutil.h"
    #include "libavcodec/avcodec.h"
    #include "libavformat/avformat.h"
    #include "libswscale/swscale.h"
    #include "libavutil/imgutils.h"
    #include "libavutil/hwcontext.h"
    #include "libavutil/hwcontext_cuda.h"
    #include "libavutil/opt.h"
}

using namespace ILLIXR;


class offload_writer : public plugin {
public:
    offload_writer(std::string name_, phonebook* pb_)
		: plugin{name_, pb_}
		, sb{pb->lookup_impl<switchboard>()}
    { 
        //pyh setup encoding environment
        //Encode raw frame
        //find encoder
        
        //initialize
        hw_device_ctx=NULL; img0_swframe=NULL; img0_hwframe=NULL; c=NULL; codec=NULL;
        int ret;
        
        //open a device of the specified type and create an AVHWDevice Context for it
        //hw_device_ctx describes the device which will be used by a hardware encoder/decoder

        //codec = encoder+decoder 
        codec = avcodec_find_encoder_by_name("h264_nvenc");
        //codec = avcodec_find_encoder_by_name("h264_vaapi");
        if(!codec){
            fprintf(stderr, "Codec h264_nvenc not found\n");
            exit(1);
        }
        
        c = avcodec_alloc_context3(codec);
        if(!c){
            std::cout<<"Could not allocate video codec context\n";
            exit(1);
        }
        //our parameters
        c->bit_rate = 15000000;
        c->width = 640;
        c->height = 480;
        c->codec_type=AVMEDIA_TYPE_VIDEO;
        //1/framerate
        c->time_base = (AVRational){1,30};
        c->framerate = (AVRational){30,1};
        //emit one intra frame every 6 frames
        c->gop_size = 6;
        c->max_b_frames = 1;
        c->pix_fmt = AVPixelFormat::AV_PIX_FMT_CUDA;
        //c->pix_fmt = AVPixelFormat::AV_PIX_FMT_VAAPI;
        c->sw_pix_fmt = AVPixelFormat::AV_PIX_FMT_YUV420P;
        
        //set fild of obj to a given value
        //set preset to low latency high quality
        ret = av_opt_set(c->priv_data,"preset","llhq",0);
        //set constant bitrate low delay high quality mode
        ret = av_opt_set(c->priv_data, "rc", "cbr_ld_hq",0);
        //set to enable zero latency operation
        ret = av_opt_set_int(c->priv_data,"zerolatency",1,0);
        //set delay # of frame to 0
        ret = av_opt_set_int(c->priv_data,"delay",0,0);
        //pyh didn't add instantaneous decoder refresh 
        
        enum AVHWDeviceType type;
        type = av_hwdevice_find_type_by_name("cuda");
        if(type == AV_HWDEVICE_TYPE_NONE)
        {
            while((type = av_hwdevice_iterate_types(type)) != AV_HWDEVICE_TYPE_NONE)
                std::cout<<"device avail: "<<av_hwdevice_get_type_name(type)<<"\n";
        }
        
        ret = av_hwdevice_ctx_create(&hw_device_ctx, type, "Nvidia 3090", NULL, 0);
        
        if(ret < 0)
        {
            std::cout<<"hwdevice_ctx_create failed\n";
            exit(1);
        }

        
        
        
        AVBufferRef *hw_frames_ref;
        AVHWFramesContext *frames_ctx;
        
        if(!(hw_frames_ref = av_hwframe_ctx_alloc(hw_device_ctx))){
            std::cout<<"failed to create CUDA frame content\n";
            exit(1);
        }
        frames_ctx = (AVHWFramesContext *)(hw_frames_ref->data);
        //pyh this is every single format that has HWACCEL flag
        frames_ctx->format = AVPixelFormat::AV_PIX_FMT_CUDA; //mmal not supported
        //frames_ctx->format = AVPixelFormat::AV_PIX_FMT_VAAPI; //vaapi_moco not supported
        //frames_ctx->format = AVPixelFormat::AV_PIX_FMT_D3D11VA_VLD; //qsv
        //frames_ctx->format = AVPixelFormat::AV_PIX_FMT_DXVA2_VLD; //yuv444p16le
        //frames_ctx->format = AVPixelFormat::AV_PIX_FMT_VIDEOTOOLBOX; //ayuv64le
        //frames_ctx->format = AVPixelFormat::AV_PIX_FMT_VDPAU; // yuva444p16be
        //frames_ctx->format = AVPixelFormat::AV_PIX_FMT_QSV; //gbrap16be
        //frames_ctx->format = AVPixelFormat::AV_PIX_FMT_MMAL; //gbrap16le 
        //frames_ctx->format = AVPixelFormat::AV_PIX_FMT_MEDIACODEC;//gbrap10be
        //frames_ctx->format = AVPixelFormat::AV_PIX_FMT_D3D11;//p016le
        //frames_ctx->format = AVPixelFormat::AV_PIX_FMT_DRM_PRIME;//gbrapf32be
        //frames_ctx->format = AVPixelFormat::AV_PIX_FMT_OPENCL; //gbrapf32le
        //frames_ctx->format = AVPixelFormat::AV_PIX_FMT_VULKAN;// nv24
        //frames_ctx->format = AVPixelFormat::AV_PIX_FMT_XVMC; //bayer_grbg16le
        
        frames_ctx->sw_format = AVPixelFormat::AV_PIX_FMT_YUV420P;
        frames_ctx->width=640;
        frames_ctx->height=480;
        frames_ctx->initial_pool_size=20;
        frames_ctx->device_ref = hw_device_ctx;
        frames_ctx->device_ctx = (AVHWDeviceContext *)hw_device_ctx->data;


        //const enum AVPixelFormat *pix_fmt;
        //AVHWFramesContext *ctx = (AVHWFramesContext*)hw_frames_ref->data;
        //for (pix_fmt = ctx->internal->hw_type->pix_fmts; *pix_fmt != AV_PIX_FMT_NONE; pix_fmt++) {
        //    std::cout<<"found format: "<<av_get_pix_fmt_name(ctx->format)<<"\n";
        //}

        ret = av_hwframe_ctx_init(hw_frames_ref);
      
        if(ret < 0)
        {
            std::cout<<"av_hwframe_ctx_init failed\n";
            exit(1);
        }
        std::cout<<"stopped here\n";
        exit(1);
        ////allocate an AVHWFramesContext tied to a device context
        //c->hw_frames_ctx = av_hwframe_ctx_alloc(c->hw_device_ctx);

        //AVHWFramesContext *img0_frames_ctx = (AVHWFramesContext *) (c->hw_frames_ctx->data);
        //
        //img0_frames_ctx->format = AVPixelFormat::AV_PIX_FMT_CUDA;
        //img0_frames_ctx->sw_format = AVPixelFormat::AV_PIX_FMT_YUV420P;
        //img0_frames_ctx->width=640;
        //img0_frames_ctx->height=480;
        //img0_frames_ctx->initial_pool_size=20;
        ////img0_frames_ctx->device_ref = c->hw_device_ctx;
        ////img0_frames_ctx->device_ctx = (AVHWDeviceContext *)c->hw_device_ctx->data;
        ////failed here
        //ret = av_hwframe_ctx_init(c->hw_frames_ctx);
      
        //if(ret < 0)
        //{
        //    std::cout<<"av_hwframe_ctx_init failed\n";
        //    exit(1);
        //}


        ////initialize teh VCodecConent to use the given AVCodec
        //ret = avcodec_open2(c,codec,NULL);
        //if(ret < 0){
        //    std::cout<<"could not open codec\n";
        //    exit(1);
        //}

        //
        //
        //
        ////AVPacket stores compressed data
        //AVPacket *pkt;
        //pkt = av_packet_alloc();

       



        //
        //

        //
        //img0_frame = av_frame_alloc();
        //if(!img0_frame)
        //{
        //    std::cout<<"could not allocate video frame\n";
        //}
        //img0_frame->format = c->pix_fmt;
        //img0_frame->width = c->width;
        //img0_frame->height = c->height;

        ////allocate new buffers for video data, 2nd parameter align
        //ret = av_hwframe_get_buffer(c->hw_frames_ctx,img0_frame,0);
        //
        //if(ret < 0){
        //    std::cout<<"could not allocate the hw video frame data\n";
        //}

        eCAL::Initialize(0, NULL, "VIO Device Transmitter");
		publisher = eCAL::protobuf::CPublisher<vio_input_proto::IMUCamVec>("vio_input");
		publisher.SetLayerMode(eCAL::TLayer::tlayer_udp_mc, eCAL::TLayer::smode_off);
		publisher.SetLayerMode(eCAL::TLayer::tlayer_tcp, eCAL::TLayer::smode_auto);
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
        
        if(datum->img0.has_value())
        {
            //int width = datum->img0->cols;
            //int height = datum->img0->rows;
            //
            //int cvLinesizes[1];
            //cvLinesizes[0] = datum->img0->step1();

            ////allocate an image with size w and h and pixel format pix_fmt and fill pointers and linesizes accordingly
            ////0RGB32 corresponds to ARGB, 1 means align the value to use for buffer size alignment
            //av_image_alloc(img0_frame->data, img0_frame->linesize, width, height, AVPixelFormat::AV_PIX_FMT_YUV420P, 1); 
            //
            ////sws -> ffmpeg color conversion and scaling libbrary?
            ////allocate and return an SwsContext, one need it to perform scaling/conversion operations using sws_scale
            ////parameters: src_width, src_height, srcFormat, dest_width, dest_height, dest_format, algo to rescale, src_filter, dst_filter, extra param
            //SwsContext *conversion = sws_getContext(width,height,AVPixelFormat::AV_PIX_FMT_GRAY8,width,height, (AVPixelFormat)img0_frame->format,SWS_BICUBIC,NULL,NULL,NULL);
            //
            ////scale the image slice in srcSlice, and put the resulting scaled slice in the image in dst
            ////params: 
            ////  1. swscontext
            ////  2. srcSlice(array containing the pointers to the planes of source slice)
            ////  3. array containing the strides for each plane of the source image
            ////  4. the position in the source image of the slice to process, that is the number (coutned starting from zero) in the image of the first row of the slice
            ////  5. the height of the source slice, that is the number of rows in the slice, 
            ////  6. dst, the array containing the pointers to the plane of the destination image
            ////  7. dstStride, the array containing the strides of each plane of the destination image
            //sws_scale(conversion, &datum->img0->data, cvLinesizes, 0, height, img0_frame->data, img0_frame->linesize);

            //sws_freeContext(conversion);
        }

        //encode every frame of video
        //supply a raw video frame to encoder
        //int ret = avcodec_send_frame(c,img0_frame);
        //if(ret < 0)
        //{
        //    std::cout<<"error sending a frame for encoding\n";
        //    exit(1);
        //}

        //read encoded data from the coder
        //while( ret>=0)
        //{
        //    ret = avcodec_receive_packet(c,img0_pkt);
        //    if(ret == AVERROR(EAGAIN) || ret == AVERROR_EOF)
        //        return;
        //    else if (ret < 0){
        //        std::cout<<"Error during encoding\n";
        //    }
        //}
        //

		assert(datum->time.time_since_epoch().count() > previous_timestamp);
		previous_timestamp = datum->time.time_since_epoch().count();

		vio_input_proto::IMUCamData* imu_cam_data = data_buffer->add_imu_cam_data();
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
			
			publisher.Send(*data_buffer);
			delete data_buffer;
			data_buffer = new vio_input_proto::IMUCamVec();
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
    AVCodecContext *c;
    const AVCodec *codec;
    AVBufferRef *hw_device_ctx;

};

PLUGIN_MAIN(offload_writer)
