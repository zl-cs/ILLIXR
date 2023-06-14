#include "common/threadloop.hpp"
#include "common/plugin.hpp"

#include "common/data_format.hpp"
#include "common/phonebook.hpp"
#include "common/stoplight.hpp"
#include "common/switchboard.hpp"
#include "sr_input.pb.h"

#include <opencv/cv.hpp>
#include <opencv2/core/mat.hpp>
#include <filesystem>
#include <fstream>
#include <ctime>
#include <cstdlib>

//#include "video_encoder.h"
//#include <boost/lockfree/spsc_queue.hpp>
#include "common/network/socket.hpp"
#include "common/network/timestamp.hpp"
#include "common/network/net_config.hpp"

using namespace ILLIXR;

class offload_writer : public threadloop {
private:
    //boost::lockfree::spsc_queue<uint64_t> queue {1000};
    //std::vector<int32_t> sizes;
    //std::mutex mutex;
    //std::condition_variable cv;
    //GstMapInfo img0;
    //GstMapInfo img1;
    bool img_ready = false;

public:
    offload_writer(std::string name_, phonebook* pb_)
		: threadloop{name_, pb_}
		, sb{pb->lookup_impl<switchboard>()}
		, _m_clock{pb->lookup_impl<RelativeClock>()}
		, server_addr(SERVER_IP, SERVER_PORT_1)
    { 
		if (!filesystem::exists(data_path)) {
			if (!std::filesystem::create_directory(data_path)) {
				std::cerr << "Failed to create data directory.";
			}
		}
		
		enc_latency.open(data_path + "/enc_latency.csv");
		//compression_csv.open(data_path + "/compression_info.csv");
		//compression_csv << "compression ratio" << "," << "size"<< "," << "average size" << std::endl;

		socket.set_reuseaddr();
		socket.bind(Address(CLIENT_IP, CLIENT_PORT_1));
		initial_timestamp();

		std::srand(std::time(0));
	}

    virtual void start() override {
        threadloop::start();

        //encoder = std::make_unique<video_encoder>([this](const GstMapInfo& img0, const GstMapInfo& img1) {
        //    queue.consume_one([&](uint64_t& timestamp) {
        //        uint64_t curr = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
        //        // std::cout << "=== latency: " << (curr - timestamp) / 1000000.0 << std::endl;
        //    });
        //    {
        //        std::lock_guard<std::mutex> lock{mutex};
        //        this->img0 = img0;
        //        this->img1 = img1;
        //        img_ready = true;
        //    }
        //    cv.notify_one();
        //});
        //encoder->init();

		cout << "TEST: Connecting to " << server_addr.str(":") << endl;
		socket.connect(server_addr);
		cout << "Connected to " << server_addr.str(":") << endl;	

        sb->schedule<scene_recon_type>(id, "ScanNet_RGB_Data", [this](switchboard::ptr<const scene_recon_type> datum, std::size_t) {
        //sb->schedule<scene_recon_type>(id, "ScanNet_Data", [this](switchboard::ptr<const scene_recon_type> datum, std::size_t) {
			this->send_scene_recon_data(datum);
		});
	}

protected:
    void _p_thread_setup() override {

    }

    void _p_one_iteration() override {
		while (!_m_stoplight->check_should_stop()) {
        	std::this_thread::sleep_for(std::chrono::milliseconds(1000));
		}
//        _encoder->start();
    }

public:

    void send_scene_recon_data(switchboard::ptr<const scene_recon_type> datum) {
        std::cout<<"send frame: "<< frame_id <<std::endl;
        outgoing_payload = new sr_input_proto::SRSendData();
        sr_input_proto::Pose* pose = outgoing_payload->mutable_input_pose();
        
        //set pose
        //std::cout<<"test pose: x: "<<datum->pose.position.x()<<" y: "<<datum->pose.position.y()<<" z: "<<datum->pose.position.z()<<" ox: "<<datum->pose.orientation.x() <<" oy: "<< datum->pose.orientation.y() <<" oz: "<<datum->pose.orientation.z()<<" ow: "<<datum->pose.orientation.w()<<std::endl;
        pose->set_p_x(datum->pose.position.x());
        pose->set_p_y(datum->pose.position.y());
        pose->set_p_z(datum->pose.position.z());

        pose->set_o_x(datum->pose.orientation.x());
        pose->set_o_y(datum->pose.orientation.y());
        pose->set_o_z(datum->pose.orientation.z());
        pose->set_o_w(datum->pose.orientation.w());

        cv::Mat cur_depth = datum->depth.clone();
        cv::Mat cur_rgb = datum->rgb.clone();
        
        sr_input_proto::ImgData* depth_img = outgoing_payload->mutable_depth_img_data(); 
        sr_input_proto::ImgData* rgb_img = outgoing_payload->mutable_rgb_img_data();
        
        depth_img->set_rows(cur_depth.rows);
        depth_img->set_columns(cur_depth.cols);
        rgb_img->set_rows(cur_rgb.rows);
        rgb_img->set_columns(cur_rgb.cols);
        
        double cur_depth_size = cur_depth.total() *  cur_depth.elemSize();
        std::cout<<"depth total:  "<< cur_depth.total()<<" depth element size: "<<cur_depth.elemSize()<<std::endl;
        double cur_rgb_size = cur_rgb.total() *  cur_rgb.elemSize();
        std::cout<<"rgb total:  "<< cur_rgb.total()<<" rgb element size: "<<cur_rgb.elemSize()<<std::endl;
        //std::cout<<"rgb: size "<<cur_rgb.size<<" calculated: "<<cur_rgb_size<<std::endl;
        depth_img->set_img_data((void*) cur_depth.data, cur_depth_size);
        rgb_img->set_img_data((void*) cur_rgb.data, cur_rgb_size);
        depth_img->set_size(cur_depth_size);
        rgb_img->set_size(cur_rgb_size);

        outgoing_payload->set_id(frame_id); 
           
        
        //cv::Mat img0 = (datum->img0.value()).clone();
        //cv::Mat img1 = (datum->img1.value()).clone();

        //// size of img0
        //double img0_size = img0.total() * img0.elemSize();

        /** WITH COMPRESSION END **/

        /** NO COMPRESSION **/
        //imu_cam_data->set_img0_size(img0_size);
        //imu_cam_data->set_img1_size(img0_size);

        //imu_cam_data->set_img0_data((void*) img0.data, img0_size);
        //imu_cam_data->set_img1_data((void*) img1.data, img0_size);
        /** No compression END **/

        //data_buffer->set_real_timestamp(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch()).count());
        //data_buffer->set_dataset_timestamp(datum->dataset_time.time_since_epoch().count());
        //data_buffer->set_frame_id(frame_id);
        //data_buffer->set_cam_time(datum->time.time_since_epoch().count());

        // Prepare data delivery
        string delimitter = "EEND!";

        //string data_to_be_sent = data_buffer->SerializeAsString();
        string data_to_be_sent = outgoing_payload->SerializeAsString();
        
        socket.write(data_to_be_sent + delimitter);
        
        frame_id++;
        delete outgoing_payload;
        //delete data_buffer;
        //data_buffer = new vio_input_proto::IMUCamVec();
    }

	// virtual void stop() override{
	// 	for (size_t i = 0; i < ratios.size(); i++) {
	// 		compression_csv << ratios[i] << "," << sizes[i] << "," << sizes_avg[i] << std::endl;
	// 	}
	// }

private:
    //std::unique_ptr<video_encoder> encoder = nullptr;
	long previous_timestamp = 0;
	int frame_id = 0;
	//vio_input_proto::IMUCamVec* data_buffer = new vio_input_proto::IMUCamVec();
    sr_input_proto::SRSendData* outgoing_payload;
    const std::shared_ptr<switchboard> sb;
	const std::shared_ptr<RelativeClock> _m_clock;

	TCPSocket socket;
	Address server_addr;

	const std::string data_path = filesystem::current_path().string() + "/recorded_data";
	std::ofstream enc_latency;
	// std::vector<int> ratios;
	// std::vector<int> sizes_avg;
	//std::ofstream compression_csv;
};

PLUGIN_MAIN(offload_writer)
