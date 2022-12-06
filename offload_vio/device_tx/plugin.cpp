#include "common/plugin.hpp"
#include "common/switchboard.hpp"
#include "common/data_format.hpp"
#include "common/phonebook.hpp"

#include <opencv2/core/mat.hpp>
#include <filesystem>
#include <fstream>

#include "imu_features.pb.h"
#include "common/network/socket.hpp"
#include "common/network/timestamp.hpp"
#include "common/network/net_config.hpp"

using namespace ILLIXR;

class offload_writer : public plugin {
public:
    offload_writer(std::string name_, phonebook* pb_)
		: plugin{name_, pb_}
		, sb{pb->lookup_impl<switchboard>()}
		, _m_feats_MSCKF{sb->get_reader<features>("feats_MSCKF")}
		, _m_feats_slam_UPDATE{sb->get_reader<features>("feats_slam_UPDATE")}
		, _m_feats_slam_DELAYED{sb->get_reader<features>("feats_slam_DELAYED")}
		, server_addr(SERVER_IP, SERVER_PORT_1)
    { 
		socket.set_reuseaddr();
		socket.bind(Address(CLIENT_IP, CLIENT_PORT_1));
		initial_timestamp();

		if (!filesystem::exists(data_path)) {
			if (!filesystem::create_directory(data_path)) {
				std::cerr << "Failed to create data directory.";
			}
		}
		
		hashed_data.open(data_path + "/hash_device_tx.txt");
		frame_info.open(data_path + "/frame_info.csv");

		frame_info << "frame_id,created_to_sent_time_ms,duration_to_send_ms,is_dropped" << endl;
	}


    virtual void start() override {
        plugin::start();

		cout << "TEST: Connecting to " << server_addr.str(":") << endl;
		socket.connect(server_addr);
		cout << "Connected to " << server_addr.str(":") << endl;	

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

		assert(datum->time.time_since_epoch().count() > previous_timestamp);
		previous_timestamp = datum->time.time_since_epoch().count();

		imu_features_proto::IMUData* imu_data = data_buffer->add_imu_data();
		imu_data->set_timestamp(datum->time.time_since_epoch().count());

		imu_features_proto::Vec3* angular_vel = new imu_features_proto::Vec3();
		angular_vel->set_x(datum->angular_v.x());
		angular_vel->set_y(datum->angular_v.y());
		angular_vel->set_z(datum->angular_v.z());
		imu_data->set_allocated_angular_vel(angular_vel);

		imu_features_proto::Vec3* linear_accel = new imu_features_proto::Vec3();
		linear_accel->set_x(datum->linear_a.x());
		linear_accel->set_y(datum->linear_a.y());
		linear_accel->set_z(datum->linear_a.z());
		imu_data->set_allocated_linear_accel(linear_accel);

		if (datum->img0.has_value() && datum->img1.has_value()) {
			if (!cam_buffer_time.has_value()) {
				cam_buffer_time = datum->time.time_since_epoch().count();
				return;
			}
			auto feats_MSCKF = _m_feats_MSCKF.get_ro_nullable();
			auto feats_slam_UPDATE = _m_feats_slam_UPDATE.get_ro_nullable();
			auto feats_slam_DELAYED = _m_feats_slam_DELAYED.get_ro_nullable();

			if (feats_MSCKF != nullptr && feats_slam_UPDATE != nullptr && feats_slam_DELAYED != nullptr) {
				if (feats_MSCKF->timestamp == cam_buffer_time && feats_slam_UPDATE->timestamp == cam_buffer_time 
					&& feats_slam_DELAYED->timestamp == cam_buffer_time) {
						/* Serialize features */
					std::vector<feature> features_MSCKF = feats_MSCKF->feats;
					for (feature f : features_MSCKF) {
						imu_features_proto::Feature* ft = data_buffer->add_feats_msckf();
						ft->set_featid(f.featid);
						ft->set_to_delete(f.to_delete);
						ft->set_anchor_cam_id(f.anchor_cam_id);
						ft->set_anchor_clone_timestamp(f.anchor_clone_timestamp);
						imu_features_proto::Vec3* p_FinA = new imu_features_proto::Vec3();
						p_FinA->set_x(f.p_FinA.x());
						p_FinA->set_y(f.p_FinA.y());
						p_FinA->set_z(f.p_FinA.z());
						ft->set_allocated_p_fina(p_FinA);
						imu_features_proto::Vec3* p_FinG = new imu_features_proto::Vec3();
						p_FinG->set_x(f.p_FinG.x());
						p_FinG->set_y(f.p_FinG.y());
						p_FinG->set_z(f.p_FinG.z());
						ft->set_allocated_p_fing(p_FinG);
						
						for (auto uv_elem : f.uvs) {
							imu_features_proto::UVMapElem* uv_map_elem = ft->add_uv_map();
							uv_map_elem->set_id(uv_elem.first);
							for (auto uv: uv_elem.second) {
								imu_features_proto::Vec2* v = uv_map_elem->add_uv();
								v->set_x(uv.x());
								v->set_y(uv.y());
							}
						}
						for (auto uvn_elem : f.uvs_norm) {
							imu_features_proto::UVMapElem* uvn_map_elem = ft->add_uvn_map();
							uvn_map_elem->set_id(uvn_elem.first);
							for (auto uvn: uvn_elem.second) {
								imu_features_proto::Vec2* v = uvn_map_elem->add_uv();
								v->set_x(uvn.x());
								v->set_y(uvn.y());
							}
						}
						for (auto t_elem : f.timestamps) {
							imu_features_proto::TimestampMapElem* t_map_elem = ft->add_ts_map();
							t_map_elem->set_id(t_elem.first);
							for (auto t: t_elem.second) {
								t_map_elem->add_timestamps(t);
							}
						}
					}
					string data_to_be_sent = data_buffer->SerializeAsString();
					string delimitter = "END!";

					socket.write(data_to_be_sent + delimitter);

					delete data_buffer;
					data_buffer = new imu_features_proto::IMUFeatures();
				}
			}
		}
			
			// Prepare data delivery
		// 	string data_to_be_sent = data_buffer->SerializeAsString();
		// 	string delimitter = "END!";

		// 	float created_to_sent = (std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch()).count() - datum->created_time) / 1e6;
		// 	// For packet dropping experiments. To disable it, replace the condition with "false".
		// 	if (created_to_sent > 100) {
		// 		std::cout << "Created to Send > 100\n";
		// 		frame_info << frame_id << "," << created_to_sent << ",0,1" << endl;
		// 	} else {
		// 		auto start = timestamp();
		// 		socket.write(data_to_be_sent + delimitter);
		// 		auto send_duration = timestamp() - start;
		// 		frame_info << frame_id << "," << created_to_sent << "," << send_duration << ",0" << endl;

		// 		hash<std::string> hasher;
		// 		auto hash_result = hasher(data_to_be_sent);
		// 		hashed_data << frame_id << "\t" << hash_result << "\t" << data_buffer->dataset_timestamp() << endl;
		// 	}

		// 	// auto start = timestamp();
		// 	// socket.write(data_to_be_sent + delimitter);
		// 	// auto send_duration = timestamp() - start;
		// 	// cam_data_created_to_send_time << created_to_sent << endl;
		// 	// cout << "Frame id = " << frame_id << ", send time = " << send_duration << ", created_to_send = " << created_to_sent << endl;

		// 	hash<std::string> hasher;
		// 	auto hash_result = hasher(data_to_be_sent);
		// 	hashed_data << frame_id << "\t" << hash_result << "\t" << data_buffer->dataset_timestamp() << endl;
			
		// 	frame_id++;

		// 	delete data_buffer;
		// 	data_buffer = new vio_input_proto::IMUCamVec();
		// }
		
    }

private:
	long previous_timestamp = 0;
	int frame_id = 0;
	imu_features_proto::IMUFeatures* data_buffer = new imu_features_proto::IMUFeatures();
    const std::shared_ptr<switchboard> sb;
	switchboard::reader<features> _m_feats_MSCKF;
	switchboard::reader<features> _m_feats_slam_UPDATE;
	switchboard::reader<features> _m_feats_slam_DELAYED;

	TCPSocket socket;
	Address server_addr;

	const string data_path = filesystem::current_path().string() + "/recorded_data";
	std::ofstream hashed_data;
	std::ofstream frame_info;

	std::optional<long> cam_buffer_time;
};

PLUGIN_MAIN(offload_writer)
