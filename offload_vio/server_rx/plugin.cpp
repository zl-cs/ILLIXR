#include "common/threadloop.hpp"
#include "common/switchboard.hpp"
#include "common/data_format.hpp"
#include "common/phonebook.hpp"

#include <filesystem>
#include <fstream>

#include "imu_features.pb.h"

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
		, _m_imu_buffer{sb->get_writer<imu_buffer>("imu_buffer")}
		, _conn_signal{sb->get_writer<connection_signal>("connection_signal")}
		, _m_feats_MSCKF{sb->get_writer<features>("feats_MSCKF")}
		, server_addr(SERVER_IP, SERVER_PORT_1)
		, buffer_str("")
    { 
		if (!filesystem::exists(data_path)) {
			if (!std::filesystem::create_directory(data_path)) {
				std::cerr << "Failed to create data directory.";
			}
		}
		
		receive_time.open(data_path + "/receive_time.csv");
		hashed_data.open(data_path + "/hash_server_rx.txt");
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
			string delimitter = "END!";
			string recv_data = read_socket->read(); /* Blocking operation, wait for the data to come */
			buffer_str = buffer_str + recv_data;
			if (recv_data.size() > 0) {
				string::size_type end_position = buffer_str.find(delimitter);
				while (end_position != string::npos) {
					string before = buffer_str.substr(0, end_position);
					buffer_str = buffer_str.substr(end_position + delimitter.size());
					// process the data
					imu_features_proto::IMUFeatures vio_input;
					bool success = vio_input.ParseFromString(before);
					if (!success) {
						cout << "Error parsing the protobuf, vio input size = " << before.size() << endl;
					} else {
						cout << "Received the protobuf data!" << endl;
						// hash<std::string> hasher;
						// auto hash_result = hasher(before);
						// hashed_data << vio_input.frame_id() << "\t" << hash_result << endl;
						// cout << "Receive frame id = " << vio_input.frame_id() << endl;
						ReceiveVioInput(vio_input);
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

private:
	void ReceiveVioInput(const imu_features_proto::IMUFeatures& vio_input) {

		// Logging
		unsigned long long curr_time = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
		double sec_to_trans = (curr_time - vio_input.real_timestamp()) / 1e9;
		receive_time << frame_id++ << "," << vio_input.real_timestamp() << "," << sec_to_trans * 1e3 << std::endl;

		std::cout << "There are " << vio_input.imu_data_size() << " imu samples in this package\n";
		time_point start = _m_clock->now();
		// Loop through all IMU values first then the cam frame	
		for (int i = 0; i < vio_input.imu_data_size(); i++) {
			imu_features_proto::IMUData curr_data = vio_input.imu_data(i);
			imus.emplace_back(time_point{std::chrono::nanoseconds{curr_data.timestamp()}}, 
							   Eigen::Vector3d{curr_data.angular_vel().x(), curr_data.angular_vel().y(), curr_data.angular_vel().z()},
							   Eigen::Vector3d{curr_data.linear_accel().x(), curr_data.linear_accel().y(), curr_data.linear_accel().z()});


			
		}
		std::vector<feature> feats_MSCKF;
		for (int i = 0; i < vio_input.feats_msckf_size(); i++) {
			imu_features_proto::Feature f = vio_input.feats_msckf(i);
			// reconstruct uv_map
			std::unordered_map<size_t, std::vector<Eigen::VectorXf>> uv_map;
			for (int j = 0; j < f.uv_map_size(); j++) {
				imu_features_proto::UVMapElem uv_map_elem = f.uv_map(j);
				std::vector<Eigen::VectorXf> vec;
				for (int k = 0; k < uv_map_elem.uv_size(); k++) {
					imu_features_proto::Vec2 v = uv_map_elem.uv(k);
					// vec.push_back(Eigen::VectorXf{v.x(), v.y()});
					Eigen::VectorXf vxf(2);
					vxf.x() = v.x();
					vxf.y() = v.y();
					vec.push_back(vxf);
				}
				uv_map.emplace(uv_map_elem.id(), vec);
			}
			// reconstruct uvn_map
			std::unordered_map<size_t, std::vector<Eigen::VectorXf>> uvn_map;
			for (int j = 0; j < f.uvn_map_size(); j++) {
				imu_features_proto::UVMapElem uvn_map_elem = f.uvn_map(j);
				std::vector<Eigen::VectorXf> vec;
				for (int k = 0; k < uvn_map_elem.uv_size(); k++) {
					imu_features_proto::Vec2 v = uvn_map_elem.uv(k);
					Eigen::VectorXf vxf(2);
					vxf.x() = v.x();
					vxf.y() = v.y();
					vec.push_back(vxf);
				}
				uvn_map.emplace(uvn_map_elem.id(), vec);
			}
			// reconstruct the timestamp map
			std::unordered_map<size_t, std::vector<double>> timestamp_map;
			for (int j = 0; j < f.ts_map_size(); j++) {
				imu_features_proto::TimestampMapElem ts_map_elem = f.ts_map(j);
				std::vector<double> vec;
				for (int k = 0; k < ts_map_elem.timestamps_size(); k++) {
					vec.push_back(ts_map_elem.timestamps(k));
				}
				timestamp_map.emplace(ts_map_elem.id(), vec);
			}
			feats_MSCKF.push_back(feature
						{f.featid(), f.to_delete(), 
						uv_map, uvn_map, timestamp_map, 
						f.anchor_cam_id(), f.anchor_clone_timestamp(), 
						Eigen::Vector3d{f.p_fina().x(), f.p_fina().y(), f.p_fina().z()},
						Eigen::Vector3d{f.p_fing().x(), f.p_fing().y(), f.p_fing().z()}
						});
		}
		std::cout << "Deserialization takes " << (_m_clock->now() - start).count() << "\n";
		_m_feats_MSCKF.put(_m_feats_MSCKF.allocate<features>(
			features{
				feats_MSCKF.size(),
				time_point{std::chrono::nanoseconds{vio_input.timestamp()}},
				feats_MSCKF
			}
		));

		_m_imu_buffer.put(_m_imu_buffer.allocate<imu_buffer>(
			imu_buffer{imus}
		));
		imus.clear();


		// unsigned long long after_time = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
		// double sec_to_push = (after_time - curr_time) / 1e9;
		// std::cout << vio_input.frame_id() << ": Seconds to push data (ms): " << sec_to_push * 1e3 << std::endl;
	}

    const std::shared_ptr<switchboard> sb;
	const std::shared_ptr<RelativeClock> _m_clock;
	switchboard::writer<imu_buffer> _m_imu_buffer;
	switchboard::writer<connection_signal> _conn_signal;
	switchboard::writer<features> _m_feats_MSCKF;

	TCPSocket socket;
	TCPSocket * read_socket = NULL;
	Address server_addr;
	string buffer_str;

	const std::string data_path = filesystem::current_path().string() + "/recorded_data";
	std::ofstream receive_time;
	std::ofstream hashed_data;

	std::vector<imu_type> imus;
	int frame_id = 0;
};

PLUGIN_MAIN(server_reader)
