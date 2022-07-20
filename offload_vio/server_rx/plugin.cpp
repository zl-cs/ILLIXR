#include <filesystem>
#include <fstream>

#include <opencv2/opencv.hpp>
#include <iostream>

#include "common/threadloop.hpp"
#include "common/switchboard.hpp"
#include "common/data_format.hpp"
#include "common/phonebook.hpp"
#include "common/relative_clock.hpp"

#include <filesystem>
#include <fstream>

#include "vio_input.pb.h"

#include "common/network/socket.hpp"
#include "common/network/net_config.hpp"
#include "common/network/timestamp.hpp"

using namespace ILLIXR;

class server_reader : public threadloop {
public:
	server_reader(std::string name_, phonebook* pb_)
		: threadloop{name_, pb_}
		, sb{pb->lookup_impl<switchboard>()}
		, _m_clock{pb->lookup_impl<RelativeClock>()}
		, _m_imu_cam{sb->get_writer<imu_cam_type_prof>("imu_cam")}
		, server_addr(SERVER_IP, SERVER_PORT_1)
		, buffer_str("")
    { 
		if (!filesystem::exists(data_path)) {
			if (!std::filesystem::create_directory(data_path)) {
				std::cerr << "Failed to create data directory.";
			}
		}

		if (!filesystem::exists(cam0_dir)) {
			if (!std::filesystem::create_directory(cam0_dir)) {
				std::cerr << "Failed to create data directory.";
			}
		}

		if (!filesystem::exists(cam1_dir)) {
			if (!std::filesystem::create_directory(cam1_dir)) {
				std::cerr << "Failed to create data directory.";
			}
		}
		
		receive_time.open(data_path + "/receive_time.csv");
		hashed_data.open(data_path + "/hash_server_rx.txt");
		imu_recv.open(data_path + "/imu_recv.csv");
		socket.set_reuseaddr();
		socket.bind(server_addr);
	}

	virtual skip_option _p_should_skip() override {
		return skip_option::run;
    }

	void _p_one_iteration() override {
		if (read_socket == NULL) {
			socket.listen();
			cout << "Waiting for connection!" << endl;
			read_socket = new TCPSocket( FileDescriptor( SystemCall( "accept", ::accept( socket.fd_num(), nullptr, nullptr) ) ) ); /* Blocking operation, waiting for client to connect */
			cout << "Connection is established with " << read_socket->peer_address().str(":") << endl;
		} else {
			// auto now = timestamp();
			string delimitter = "END!";
			// auto before_read = timestamp();
			string recv_data = read_socket->read(); /* Blocking operation, wait for the data to come */
			// cout << "Block time = " << timestamp() - before_read << endl;
			buffer_str = buffer_str + recv_data;
			if (recv_data.size() > 0) {
				// cout << "Inside if-statement\n";
				string::size_type end_position = buffer_str.find(delimitter);
				while (end_position != string::npos) {
					string before = buffer_str.substr(0, end_position);
					buffer_str = buffer_str.substr(end_position + delimitter.size());
					// cout << "Complete response = " << before.size() << endl;
					// process the data
					vio_input_proto::IMUCamVec vio_input;
					bool success = vio_input.ParseFromString(before);
					if (!success) {
						cout << "Error parsing the protobuf, vio input size = " << before.size() << endl;
					} else {
						// cout << "Received the protobuf data!" << endl;
						hash<std::string> hasher;
						auto hash_result = hasher(before);
						hashed_data << vio_input.frame_id() << "\t" << hash_result << endl;
						// cout << "vio_input: " << &(vio_input) << "\n";
						// cout << "Recv time = " << timestamp() - now << endl;
						// now = timestamp();
						ReceiveVioInput(vio_input);
						
					}
					end_position = buffer_str.find(delimitter);
					// cout << "buffer_str: " << &buffer_str << "\n";
				}
				// cout << "Recv time = " << timestamp() - now << endl;
			}
		}
	}

	~server_reader() {
		delete read_socket;
	}

private:
	void ReceiveVioInput(const vio_input_proto::IMUCamVec& vio_input) {	

		// Logging
		unsigned long long curr_time = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
		double sec_to_trans = (curr_time - vio_input.real_timestamp()) / 1e9;
		receive_time << vio_input.frame_id() << "," << vio_input.real_timestamp() << "," << sec_to_trans * 1e3 << std::endl;

		// Loop through all IMU values first then the cam frame	
		for (int i = 0; i < vio_input.imu_cam_data_size(); i++) {
			vio_input_proto::IMUCamData curr_data = vio_input.imu_cam_data(i);

			std::optional<cv::Mat> cam0 = std::nullopt;
			std::optional<cv::Mat> cam1 = std::nullopt;

			if (curr_data.rows() != -1 && curr_data.cols() != -1) {

				// Must do a deep copy of the received data (in the form of a string of bytes)
				// It turns out that the img0_copy, img0, and cam0 all point to the same data.
				// This results a data race in the socket-based implementation, but not manifest in the ecal implementation.
				auto img0_copy = std::make_shared<std::string>(std::string(curr_data.img0_data()));
				auto img1_copy = std::make_shared<std::string>(std::string(curr_data.img1_data()));
				// cout << "curr_data.img0_data(): " << &(curr_data.img0_data()) << "\n";
				// cout << "img0_copy " << (void*)img0_copy->data() << "\n";
				// cout << "img0 use count (1)" << img0_copy.use_count() << "\n";

				cv::Mat img0(curr_data.rows(), curr_data.cols(), CV_8UC1, (void*)img0_copy->data());
				cv::Mat img1(curr_data.rows(), curr_data.cols(), CV_8UC1, (void*)img1_copy->data());
				if (img0.u) cout << "img0 refcount (1) " << img0.u->refcount << "\n";
				// cout << "img0 " << (void*)(img0.data) << "\n";
				// cout << "img0 use count (2)" << img0_copy.use_count() << "\n";

				cam0 = std::make_optional<cv::Mat>(img0.clone());
				cam1 = std::make_optional<cv::Mat>(img1.clone());
				// cout << "img0 refcount (2) " << img0.u->refcount << "\n";
				cout << "cam0 refcount (3) " << cam0.value().u->refcount << "\n";
				// cout << "img0 use count (3)" << img0_copy.use_count() << "\n";
				// cout << "cam0 address " << &cam0 << "\n";
				// cout << "cam0 " << (void*)(cam0.value().data) << "\n";

				_m_imu_cam.put(_m_imu_cam.allocate<imu_cam_type_prof>(
					imu_cam_type_prof {
						_m_clock->now(),
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
				// cout << "img0 use count (4)" << img0_copy.use_count() << "\n";

				// cv::imwrite(cam0_dir + std::to_string(vio_input.frame_id()) + ".png", cam0.value());
				// cv::imwrite(cam1_dir + std::to_string(vio_input.frame_id()) + ".png", cam1.value());

			}
// time_point before_put = _m_clock->now();
else {
			_m_imu_cam.put(_m_imu_cam.allocate<imu_cam_type_prof>(
				imu_cam_type_prof {
					_m_clock->now(),
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
// std::cout << "put takes " << (_m_clock->now() - before_put).count() << " ns \n";
			// imu_recv << curr_data.timestamp() << "," << curr_data.angular_vel().x() << "," << curr_data.angular_vel().y() << "," << curr_data.angular_vel().z() << "," 
			// 		 << curr_data.linear_accel().x() << "," << curr_data.linear_accel().y() << "," << curr_data.linear_accel().z() << std::endl;
			

		}
		// std::this_thread::sleep_for(45ms);

		// unsigned long long after_time = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
		// double sec_to_push = (after_time - curr_time) / 1e9;
		// std::cout << vio_input.frame_id() << ": Seconds to push data (ms): " << sec_to_push * 1e3 << std::endl;
	}

    const std::shared_ptr<switchboard> sb;
	const std::shared_ptr<RelativeClock> _m_clock;
	switchboard::writer<imu_cam_type_prof> _m_imu_cam;

	TCPSocket socket;
	TCPSocket * read_socket = NULL;
	Address server_addr;
	string buffer_str;

	const std::string data_path = filesystem::current_path().string() + "/recorded_data";
	std::ofstream receive_time;
	std::ofstream hashed_data;
	std::ofstream imu_recv;

	std::string cam0_dir = data_path + "/cam0/";
	std::string cam1_dir = data_path + "/cam1/";

	uint64_t now = timestamp();
};

PLUGIN_MAIN(server_reader)