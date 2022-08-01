#include "common/plugin.hpp"
#include "common/switchboard.hpp"
#include "common/data_format.hpp"
#include "common/phonebook.hpp"

#include <ecal/ecal.h>
#include <ecal/msg/protobuf/subscriber.h>

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

class server_reader : public plugin {
public:
	server_reader(std::string name_, phonebook* pb_)
		: plugin{name_, pb_}
		, sb{pb->lookup_impl<switchboard>()}
		, _m_imu_cam{sb->get_writer<imu_cam_type_prof>("imu_cam")}
    { 
		int ret;
        
        //Initialize Decoder
        const AVCodec *decoder = avcodec_find_decoder_by_name("h264_cuvid"); 
        if(!decoder){
            ILLIXR::abort("Could not find decoder");
        }else{
            std::cout<<"\n Found decoder "<<decoder->name<<"\n";
        }
        
        context = avcodec_alloc_context3(decoder);
        if(!context){
            ILLIXR::abort("Failed to create context");
        }else{
            std::cout<<"Created "<<avcodec_get_name(context->codec_id)<<"\n";
        }
        
        ret = av_hwdevice_ctx_create(&context->hw_device_ctx, AV_HWDEVICE_TYPE_CUDA,NULL,NULL,0);
        if(ret){
            ILLIXR::abort("Failed to create hardware device context");
        }else{
            std::cout<<"created hw device context\n";
        }

        img0_pkt = av_packet_alloc();

        //context->get_format = get_h264_format;

        //context->hw_device_ctx = av_buffer_ref(hw_device_ctx);
        //if(!context->hw_device_ctx)
        //{
        //    ILLIXR::abort("A hardware device reference create failed");
        //}
        //open decoder
        ret = avcodec_open2(context,decoder,NULL);
        if (ret){
            ILLIXR::abort("Failed to initialize context");
        }else{
            std::cout<< "Initialized context\n";
        }

        eCAL::Initialize(0, NULL, "VIO Server Reader");
		subscriber = eCAL::protobuf::CSubscriber<vio_input_proto::IMUCamVec>("vio_input");
		subscriber.AddReceiveCallback(std::bind(&server_reader::ReceiveVioInput, this, std::placeholders::_2));
	}

private:
    AVPacket *img0_pkt;
    AVCodecContext *context;
    int count=0;
	AVBufferRef *hw_device_ctx=NULL;
    std::string image_receiving_loc = "received_images/";
    std::string encoded_loc = "encoded_images/";
    FILE *f;
    static enum AVPixelFormat get_h264_format(AVCodecContext *ctx, const enum AVPixelFormat *pix_fmts)
    {
        const enum AVPixelFormat *p;
        for(p = pix_fmts; *p != AV_PIX_FMT_NONE; p++)
        {
            if(*p == AV_PIX_FMT_NV12)
                return *p;
        }
        std::cout<<"unable to decode\n";
        return AV_PIX_FMT_NONE;
    } 
    
    void ReceiveVioInput(const vio_input_proto::IMUCamVec& vio_input) {	
		unsigned long long curr_time = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
		double sec_to_trans = (curr_time - vio_input.real_timestamp()) / 1e9;
		std::cout << vio_input.frame_id() << ": Seconds to receive pose (ms): " << sec_to_trans * 1e3 << std::endl;
        int ret;
        uint8_t *buffer=NULL;
		// Loop through all IMU values first then the cam frame	
		for (int i = 0; i < vio_input.imu_cam_data_size(); i++) {
			vio_input_proto::IMUCamData curr_data = vio_input.imu_cam_data(i);

			std::optional<cv::Mat> cam0 = std::nullopt;
			std::optional<cv::Mat> cam1 = std::nullopt;
            
			if (curr_data.rows() != -1 && curr_data.cols() != -1) {

				// Must do a deep copy of the received data (in the form of a string of bytes)
				auto img0_copy = std::make_shared<std::string>(std::string(curr_data.img0_data()));
				auto img1_copy = std::make_shared<std::string>(std::string(curr_data.img1_data()));

				cv::Mat img0(curr_data.rows(), curr_data.cols(), CV_8UC1, img0_copy->data());
				cv::Mat img1(curr_data.rows(), curr_data.cols(), CV_8UC1, img1_copy->data());

				cam0 = std::make_optional<cv::Mat>(img0);
				cam1 = std::make_optional<cv::Mat>(img1);
                
                //reconstruct ecal AV Packet 
                //deep copy of the pkt
                //auto packet_copy = std::make_shared<std::string>(std::string(curr_data.img0_pkt()));
                //std::string test = std::string(curr_data.img0_pkt());
                //auto converted_packet= reinterpret_cast<uint8_t *>(&packet_copy);
                ////from byte to Packet
                //ret = av_packet_from_data(img0_pkt,converted_packet,packet_copy->size());
                //if(ret < 0){
                //     ILLIXR::abort("Failed to initalize packet from data");   
                //}else{
                //    //std::cout<<"reconstruct a AVPacket successfully\n";
                //    printf("Reconstruct packet %d (size=%5d)\n", count, img0_pkt->size);
                //}
                //std::string seq_num = std::to_string(count);
                //std::string final_loc =  image_receiving_loc + seq_num + ".h264";
                //std::cout<<"writing receieved encoded images to "<<final_loc<<"\n";
                //f = fopen(final_loc.c_str(),"wb");
                //fwrite(img0_pkt->data, 1, img0_pkt->size, f);
                //fclose(f);
                
                //read from h264 file
                std::string seq_num = std::to_string(count);
                std::string final_loc =  encoded_loc + seq_num + ".h264";
                f = fopen(final_loc.c_str(),"rb");
                 
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
