#include "common/plugin.hpp"
#include "common/switchboard.hpp"
#include "common/data_format.hpp"
#include "common/phonebook.hpp"

#include <ecal/ecal.h>
#include <ecal/msg/protobuf/subscriber.h>
#include <opencv2/imgcodecs.hpp>
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
    #include "libavformat/avformat.h"
    #include "libswscale/swscale.h"
}

using namespace ILLIXR;

class server_reader : public plugin {
public:
	server_reader(std::string name_, phonebook* pb_)
		: plugin{name_, pb_}
		, sb{pb->lookup_impl<switchboard>()}
		, _m_imu_cam{sb->get_writer<imu_cam_type_prof>("imu_cam")}
    { 
		int ret;
        //TODO right now I am using only one decoder/encoder engine for both images, in the future, it may worth investigating that whether
        //using separate decoder/encoder engine for each image

        //open the hardware device
        ret = av_hwdevice_ctx_create(&device_ref, AV_HWDEVICE_TYPE_CUDA,"auto",NULL,0);
        if(ret){
            ILLIXR::abort("Failed to create hardware device context");
        }else{
            std::cout<<"created hw device context\n";
        }
        
        //Find Decoder
        const AVCodec *decoder = avcodec_find_decoder_by_name("h264_cuvid"); 
        if(!decoder){
            ILLIXR::abort("Could not find decoder");
        }else{
            std::cout<<"\n Found decoder "<<decoder->name<<"\n";
        }
        
        //Create context        
        context = avcodec_alloc_context3(decoder);
        if(!context){
            ILLIXR::abort("Failed to create context");
        }else{
            std::cout<<"Created "<<avcodec_get_name(context->codec_id)<<"\n";
        }
        
        //assume the compressed format is h264, wierd but working ffmpeg assignment
        context->codec_id = AV_CODEC_ID_H264; 
        context->hw_device_ctx = av_buffer_ref(device_ref);
        context->pix_fmt = static_cast<AVPixelFormat>(static_cast<int>(AV_PIX_FMT_NONE)+120);
        
        //test if the pixel format is correct
        char test[100];
        av_get_pix_fmt_string(test,100,static_cast<AVPixelFormat>(context->pix_fmt)); 
        std::cout<<" context format "<<test<<"\n";

        //open decoder
        ret = avcodec_open2(context,NULL,NULL);
        if (ret){
            ILLIXR::abort("Failed to initialize context");
        }else{
            std::cout<< "Initialized context\n";
        }

        //allocate both images
        img0_hwframe = av_frame_alloc();
        img0_swframe = av_frame_alloc();
        
        img1_hwframe = av_frame_alloc();
        img1_swframe = av_frame_alloc();
        
        //some ecal left_over
        eCAL::Initialize(0, NULL, "VIO Server Reader");
		subscriber = eCAL::protobuf::CSubscriber<vio_input_proto::IMUCamVec>("vio_input");
		subscriber.AddReceiveCallback(std::bind(&server_reader::ReceiveVioInput, this, std::placeholders::_2));
	}

private:
    AVCodecContext *context;
    AVFormatContext *input_ctx=NULL;
    int count=0;
    int actual_count=0;
	AVBufferRef *device_ref=NULL;
    //Let's Assume the images are the same for testing purpose
    std::string image_receiving_loc = "received_images/";
    std::string encoded_loc = "encoded_images/";
    std::string decoded_loc = "decoded_images/";
    AVFrame *img0_hwframe, *img0_swframe;
    AVFrame *img1_hwframe, *img1_swframe;
    FILE *f;
    
    void ReceiveVioInput(const vio_input_proto::IMUCamVec& vio_input) {	
		unsigned long long curr_time = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
		double sec_to_trans = (curr_time - vio_input.real_timestamp()) / 1e9;
		std::cout << vio_input.frame_id() << ": Seconds to receive pose (ms): " << sec_to_trans * 1e3 << std::endl;
        int ret;
        uint8_t *buffer_cam0, *buffer_cam1;
        size_t read_size;
        AVPacket *img0_pkt;
        AVPacket *img1_pkt;
        img0_pkt = av_packet_alloc();
        img1_pkt = av_packet_alloc();
        //TODO use something more than hardcoding 
        int height =480;
        int width = 752;
		// Loop through all IMU values first then the cam frame	
		for (int i = 0; i < vio_input.imu_cam_data_size(); i++) {
			vio_input_proto::IMUCamData curr_data = vio_input.imu_cam_data(i);

			std::optional<cv::Mat> cam0 = std::nullopt;
			std::optional<cv::Mat> cam1 = std::nullopt;
            
			if (curr_data.rows() != -1 && curr_data.cols() != -1) {
                //TODO this probably can be removed after cleanup
				// Must do a deep copy of the received data (in the form of a string of bytes)
                auto img0_copy = std::make_shared<std::string>(std::string(curr_data.img0_data()));
				auto img1_copy = std::make_shared<std::string>(std::string(curr_data.img1_data()));

				cv::Mat img0(height,width,CV_8UC1);
				cv::Mat img1(height,width,CV_8UC1);

                
                //read from h264 file
                //TODO you will need 2 file descriptor
                fflush(stdout);
                std::string seq_num = std::to_string(count);
                std::string img_location = encoded_loc + seq_num + ".h264";
                std::cout<<"read file: "<<img_location<<std::endl;
                f = fopen(img_location.c_str(),"rb"); 
                fseek(f,0,SEEK_END);
                //TODO need to get retrived packet size and the received tcp data
                long lSize_cam0 = ftell(f);
                buffer_cam0 = (uint8_t*) malloc (sizeof(uint8_t)*lSize_cam0);
                //rewind  back to beginning
                rewind(f);
                read_size = fread(buffer_cam0, 1, lSize_cam0,f);
                std::cout<<"cam0 size: "<<read_size<<std::endl;

                //reconstruct AVPackets from file
                ret = av_packet_from_data(img0_pkt,buffer_cam0,lSize_cam0);
                if(ret < 0){
                     ILLIXR::abort("Failed to initalize packet from data");   
                }else{
                    printf("Reconstruct cam0 packet %d (size=%5d)\n", count, img0_pkt->size);
                }
                
                //send cam0 first to decoder
                ret = avcodec_send_packet(context,img0_pkt);
                if( ret == AVERROR(EAGAIN)){
                    ILLIXR::abort("eagain");
                }
                else if (ret == AVERROR_EOF){
                    ILLIXR::abort("EOF");
                }
                else if(ret == AVERROR(EINVAL)){
                    ILLIXR::abort("EINVAL");
                }
                else if (ret==AVERROR(ENOMEM)){
                    ILLIXR::abort("decoding error");
                }
                else{
                    std::cout<<"packet sent to decoder\n";
                }
                
                //try to receive cam0 frame
                while(1)
                {
                    ret = avcodec_receive_frame(context,img0_hwframe);
                    if(ret == AVERROR(EAGAIN)){
                        //send additional frame
                        std::cout<<"EAGAIN\n";
                        ret = avcodec_send_packet(context,NULL);
                        if(ret < 0){
                            ILLIXR::abort("error sending packet");
                        }
                        else{
                            std::cout<<"packet resent\n";
                        }
                        continue;
                    }
                    else if( ret == AVERROR_EOF){
                        std::cout<< "EOF\n";
                        break;
                    }
                    else if (ret < 0){
                        ILLIXR::abort("Error during decoding");
                    }
                    
                    char test[100];
                    av_get_pix_fmt_string(test,10,static_cast<AVPixelFormat>(img0_hwframe->format));
                    std::cout<<"frame received format string : "<<test<<"\n";
                    if(img0_hwframe->format == context->pix_fmt){
                        std::cout<<"hw format match decoder format\n";
                        int width = img0_hwframe->width;
                        int height = img0_hwframe->height;
                        cv::Mat image(height,width,CV_8UC1);
                        int cvLinesizes[1];
                        cvLinesizes[0] = image.step1();
                        SwsContext *conversion = sws_getContext(width,height,(AVPixelFormat)img0_hwframe->format,width,height,AVPixelFormat::AV_PIX_FMT_GRAY8,SWS_BICUBIC,NULL,NULL,NULL);
                        sws_scale(conversion,img0_hwframe->data, img0_hwframe->linesize, 0, height, &image.data,cvLinesizes);
                        sws_freeContext(conversion);
                        //write img out to test correctness
                        std::string img_dump_location = decoded_loc + seq_num + ".png";
                        cv::imwrite(img_dump_location,image);
                        image.release();
                        std::cout<<"cam0 frame has written\n";
                    }
                    else{
                        std::cout<<"hardware frame does not match decoder format?\n";
                    }
                    avcodec_flush_buffers(context);
                    break;
                } 
                 
                // TODO, just implement thie cam1 the same as acm0
                /*                
                //send cam1 to decoder
                ret = avcodec_send_packet(context,img1_pkt);
                if( ret == AVERROR(EAGAIN)){
                    ILLIXR::abort("eagain");
                }
                else if (ret == AVERROR_EOF)
                {
                    ILLIXR::abort("EOF");
                }
                else if(ret == AVERROR(EINVAL))
                {
                    ILLIXR::abort("EINVAL");
                }
                else if (ret==AVERROR(ENOMEM))
                {
                    ILLIXR::abort("decoding error");
                }
                else{
                    std::cout<<"packet sent to decoder\n";
                }
                
                //try to receive cam0 frame
                while(1)
                {
                    ret = avcodec_receive_frame(context,img1_hwframe);
                    if(ret == AVERROR(EAGAIN)){
                        //send additional frame
                        std::cout<<"EAGAIN\n";
                        ret = avcodec_send_packet(context,NULL);
                        if(ret < 0){
                            ILLIXR::abort("error sending packet");
                        }
                        else{
                            std::cout<<"packet resent\n";
                        }
                        continue;
                    }
                    else if( ret == AVERROR_EOF){
                        std::cout<< "EOF\n";
                        break;
                    }
                    else if (ret < 0){
                        ILLIXR::abort("Error during decoding");
                    }
                    
                    char test[100];
                    av_get_pix_fmt_string(test,10,static_cast<AVPixelFormat>(img1_hwframe->format));
                    std::cout<<"frame received format string : "<<test<<"\n";
                    if(img1_hwframe->format == context->pix_fmt){
                        std::cout<<"hw format match decoder format\n";
                        int width = img1_hwframe->width;
                        int height = img1_hwframe->height;
                        int cvLinesizes[1];
                        cvLinesizes[0] = img1.step1();
                        SwsContext *conversion = sws_getContext(width,height,(AVPixelFormat)img1_hwframe->format,width,height,AVPixelFormat::AV_PIX_FMT_GRAY8,SWS_BICUBIC,NULL,NULL,NULL);
                        sws_scale(conversion,img1_hwframe->data, img1_hwframe->linesize, 0, height, &img1.data,cvLinesizes);
                        sws_freeContext(conversion);
                        std::cout<<"cam0 frame has written\n";
                    }
                    else{
                        std::cout<<"hardware frame does not match decoder format?\n";
                    }
                    avcodec_flush_buffers(context);
                    break;
                } 
                */
                printf("decoded frame %d \n", count);
            
                count++;
                
			}
            


			_m_imu_cam.put(_m_imu_cam.allocate<imu_cam_type_prof>(
				imu_cam_type_prof {
					vio_input.frame_id(),
					time_point{std::chrono::nanoseconds{curr_data.timestamp()}},
					time_point{std::chrono::nanoseconds{vio_input.real_timestamp()}}, // Timestamp of when the device sent the packet
					time_point{std::chrono::nanoseconds{curr_time}}, // Timestamp of receive time of the packet
					time_point{std::chrono::nanoseconds{vio_input.dataset_timestamp()}}, // Timestamp of the sensor data
					Eigen::Vector3f{curr_data.angular_vel().x(), curr_data.angular_vel().y(), curr_data.angular_vel().z()},
					Eigen::Vector3f{curr_data.linear_accel().x(), curr_data.linear_accel().y(), curr_data.linear_accel().z()},
					cam0,
					cam1
				}
			));	
		}

		// unsigned long long after_time = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
		// double sec_to_push = (after_time - curr_time) / 1e9;
		// std::cout << vio_input.frame_id() << ": Seconds to push data (ms): " << sec_to_push * 1e3 << std::endl;
	}

    const std::shared_ptr<switchboard> sb;
	switchboard::writer<imu_cam_type_prof> _m_imu_cam;

	eCAL::protobuf::CSubscriber<vio_input_proto::IMUCamVec> subscriber;
};

PLUGIN_MAIN(server_reader)
