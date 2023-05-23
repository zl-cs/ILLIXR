#include "common/threadloop.hpp"
#include "common/switchboard.hpp"
#include "common/data_format.hpp"
#include "common/phonebook.hpp"
#include "common/switchboard.hpp"
#include "sr_input.pb.h"

#include <boost/lockfree/spsc_queue.hpp>
#include <opencv2/highgui.hpp>
#include <filesystem>
#include <fstream>

#include "video_decoder.h"

#include "common/network/socket.hpp"
#include "common/network/net_config.hpp"
#include "common/network/timestamp.hpp"

using namespace ILLIXR;

class server_reader : public threadloop {
private:
    std::unique_ptr<video_decoder> decoder;

    boost::lockfree::spsc_queue<uint64_t> queue {1000};
    std::mutex mutex;
    std::condition_variable cv;
    cv::Mat img0;
    cv::Mat img1;
    bool img_ready = false;
public:
	server_reader(std::string name_, phonebook* pb_)
		: threadloop{name_, pb_}
		, sb{pb->lookup_impl<switchboard>()}
		, _m_scannet{sb->get_writer<scene_recon_type>("ScanNet_Data")}
		, _conn_signal{sb->get_writer<connection_signal>("connection_signal")}
		, server_addr(SERVER_IP, SERVER_PORT_1)
		, buffer_str("")
    { 
		if (!filesystem::exists(data_path)) {
			if (!std::filesystem::create_directory(data_path)) {
				std::cerr << "Failed to create data directory.";
			}
		}
		
		receive_time.open(data_path + "/receive_time.csv");
		// hashed_data.open(data_path + "/hash_server_rx.txt");
		dec_latency.open(data_path + "/dec.csv");
		socket.set_reuseaddr();
		socket.bind(server_addr);
	}

	virtual skip_option _p_should_skip() override {
		return skip_option::run;
    }

	void _p_one_iteration() override {
		if (read_socket == NULL) {
			_conn_signal.put(_conn_signal.allocate<connection_signal>(
				connection_signal{true}
			));
			socket.listen();
			cout << "server_rx: Waiting for connection!" << endl;
			read_socket = new TCPSocket( FileDescriptor( SystemCall( "accept", ::accept( socket.fd_num(), nullptr, nullptr) ) ) ); /* Blocking operation, waiting for client to connect */
			cout << "server_rx: Connection is established with " << read_socket->peer_address().str(":") << endl;
		} else {
			auto now = timestamp();
			string delimitter = "EEND!";
			string recv_data = read_socket->read(); /* Blocking operation, wait for the data to come */
			buffer_str = buffer_str + recv_data;
			if (recv_data.size() > 0) {
				string::size_type end_position = buffer_str.find(delimitter);
				while (end_position != string::npos) {
					string before = buffer_str.substr(0, end_position);
					buffer_str = buffer_str.substr(end_position + delimitter.size());
					// cout << "Complete response = " << before.size() << endl;
					// process the data
                    sr_input_proto::SRSendData sr_input;
                    //vio_input_proto::IMUCamVec vio_input;
					//bool success = vio_input.ParseFromString(before);
					bool success = sr_input.ParseFromString(before);
					if (!success) {
						cout << "Error parsing the protobuf, vio input size = " << before.size() << endl;
					} else {
						// cout << "Received the protobuf data!" << endl;
						// hash<std::string> hasher;
						// auto hash_result = hasher(before);
						// hashed_data << vio_input.frame_id() << "\t" << hash_result << endl;
						// cout << "Receive frame id = " << vio_input.frame_id() << endl;
						ReceiveSRInput(sr_input);
					}
					end_position = buffer_str.find(delimitter);
				}
				// cout << "Recv time = " << timestamp() - now << ", size = " << recv_data.size() << endl;
			}
		}
	}

	~server_reader() {
		delete read_socket;
	}

    void start() override {
        threadloop::start();

        decoder = std::make_unique<video_decoder>([this](cv::Mat&& img0, cv::Mat&& img1) {
            // std::cout << "callback" << std::endl;

            // show img0
            // cv::imshow("img0", img0);
            // cv::waitKey(1);

            queue.consume_one([&](uint64_t& timestamp) {
                uint64_t curr = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
                // std::cout << "=== latency: " << (curr - timestamp) / 1000000.0 << std::endl;
            });
            {
                std::lock_guard<std::mutex> lock{mutex};
                this->img0 = std::forward<cv::Mat>(img0);
                this->img1 = std::forward<cv::Mat>(img1);
                img_ready = true;
            }
            // std::cout << "notify" << std::endl;
            cv.notify_one();
        });
        decoder->init();
    }

private:
	void ReceiveSRInput(const sr_input_proto::SRSendData& sr_input) {
        // std::cout << "Received VIO input" << std::endl;

		// Logging
		//unsigned long long curr_time = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
		//double sec_to_trans = (curr_time - vio_input.real_timestamp()) / 1e9;
		// receive_time << vio_input.frame_id() << "," << vio_input.real_timestamp() << "," << sec_to_trans * 1e3 << std::endl;
		//receive_time << vio_input.frame_id() << "," << vio_input.cam_time() << "," << sec_to_trans * 1e3 << std::endl;

		// Loop through all IMU values first then the cam frame	
        Eigen::Vector3f incoming_position{sr_input.input_pose().p_x(), sr_input.input_pose().p_y(), sr_input.input_pose().p_z()};
        Eigen::Quaternionf incoming_orientation{sr_input.input_pose().o_x(), sr_input.input_pose().o_y(), sr_input.input_pose().o_z(), sr_input.input_pose().o_x()};
        pose_type pose = {time_point{}, incoming_position, incoming_orientation};

		// Must do a deep copy of the received data (in the form of a string of bytes)
		auto depth_copy = std::string(sr_input.depth_img_data().img_data());
		auto rgb_copy = std::string(sr_input.rgb_img_data().img_data());
		cv::Mat img_depth(sr_input.depth_img_data().rows(), sr_input.depth_img_data().columns(), CV_16UC1, depth_copy.data());
		cv::Mat img_rgb(sr_input.rgb_img_data().rows(), sr_input.rgb_img_data().columns(), CV_8UC3, rgb_copy.data());
        std::string test_depth = std::to_string(sr_input.id()) + "_depth.pgm";
        std::string color_depth = std::to_string(sr_input.id()) + "_color.png";
		// unsigned long long after_time = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
		// double sec_to_push = (after_time - curr_time) / 1e9;
		// std::cout << vio_input.frame_id() << ": Seconds to push data (ms): " << sec_to_push * 1e3 << std::endl;
	}

private:

    const std::shared_ptr<switchboard> sb;
	switchboard::writer<scene_recon_type> _m_scannet;
	switchboard::writer<connection_signal> _conn_signal;

	TCPSocket socket;
	TCPSocket * read_socket = NULL;
	Address server_addr;
	string buffer_str;

	const std::string data_path = filesystem::current_path().string() + "/recorded_data";
	std::ofstream receive_time;
	// std::ofstream hashed_data;
	std::ofstream dec_latency;
};

PLUGIN_MAIN(server_reader)
