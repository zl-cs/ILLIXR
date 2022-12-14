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
		, _m_clock{pb->lookup_impl<RelativeClock>()}
		// , _m_feats_MSCKF{sb->get_reader<features>("feats_MSCKF")}
		// , _m_feats_slam_UPDATE{sb->get_reader<features>("feats_slam_UPDATE")}
		// , _m_feats_slam_DELAYED{sb->get_reader<features>("feats_slam_DELAYED")}
		, _m_left_pts{sb->get_reader<key_points>("left_pts")}
		, _m_right_pts{sb->get_reader<key_points>("right_pts")}
		, server_addr(SERVER_IP, SERVER_PORT_1)
		, feats_timestamp{time_point{}}
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

		// frame_info << "frame_id,created_to_sent_time_ms,duration_to_send_ms,is_dropped" << endl;
		frame_info << "frame_id,created_to_sent,created_to_sent_done,serialization_and_sent" << std::endl;
	}


    virtual void start() override {
        plugin::start();

		cout << "TEST: Connecting to " << server_addr.str(":") << endl;
		socket.connect(server_addr);
		cout << "Connected to " << server_addr.str(":") << endl;	

        sb->schedule<imu_buffer>(id, "imu_buffer", [this](switchboard::ptr<const imu_buffer> datum, std::size_t) {
			this->send_imu_cam_data(datum);
		});
	}


    void send_imu_cam_data(switchboard::ptr<const imu_buffer> datum) {
		// Ensures that slam doesnt start before valid IMU readings come in
        if (datum == nullptr) {
            assert(previous_timestamp == 0);
            return;
        }

		// assert(datum->time.time_since_epoch().count() > previous_timestamp);
		// previous_timestamp = datum->time.time_since_epoch().count();

		for (imu_type sample : datum->imus) {
			imu_features_proto::IMUData* imu_data = data_buffer->add_imu_data();
			imu_data->set_timestamp(sample.timestamp.time_since_epoch().count());

			imu_features_proto::Vec3* angular_vel = new imu_features_proto::Vec3();
			angular_vel->set_x(sample.wm.x());
			angular_vel->set_y(sample.wm.y());
			angular_vel->set_z(sample.wm.z());
			imu_data->set_allocated_angular_vel(angular_vel);

			imu_features_proto::Vec3* linear_accel = new imu_features_proto::Vec3();
			linear_accel->set_x(sample.am.x());
			linear_accel->set_y(sample.am.y());
			linear_accel->set_z(sample.am.z());
			imu_data->set_allocated_linear_accel(linear_accel);
		}

		// if (datum->img0.has_value() && datum->img1.has_value()) {
			/* TODO
			Would like to check the features' timestamps to make sure that they are well synced. 
			But the logic is a bit hard to design.
			*/
			// if (!cam_buffer_time.has_value()) {
			// 	cam_buffer_time = datum->time.time_since_epoch().count();
			// 	return;
			// }
			// auto feats_MSCKF = _m_feats_MSCKF.get_ro_nullable();
			// auto feats_slam_UPDATE = _m_feats_slam_UPDATE.get_ro_nullable();
			// auto feats_slam_DELAYED = _m_feats_slam_DELAYED.get_ro_nullable();

			auto left_pts = _m_left_pts.get_ro_nullable();
			auto right_pts = _m_right_pts.get_ro_nullable();

			if (left_pts != nullptr && right_pts != nullptr) {
				time_point start = _m_clock->now();
				data_buffer->set_timestamp(left_pts->timestamp.time_since_epoch().count());
				for (key_point kpt : left_pts->pts) {
					imu_features_proto::KeyPoint* kpt_proto = data_buffer->add_left_pts();
					kpt_proto->set_id(kpt.id);
					cv::KeyPoint cv_kpt = kpt.pt;
					kpt_proto->set_angle(cv_kpt.angle);
					kpt_proto->set_octave(cv_kpt.octave);
					kpt_proto->set_class_id(cv_kpt.class_id);
					kpt_proto->set_x(cv_kpt.pt.x);
					kpt_proto->set_y(cv_kpt.pt.y);
					kpt_proto->set_response(cv_kpt.response);
					kpt_proto->set_size(cv_kpt.size);
				}
				for (key_point kpt : right_pts->pts) {
					imu_features_proto::KeyPoint* kpt_proto = data_buffer->add_right_pts();
					kpt_proto->set_id(kpt.id);
					cv::KeyPoint cv_kpt = kpt.pt;
					kpt_proto->set_angle(cv_kpt.angle);
					kpt_proto->set_octave(cv_kpt.octave);
					kpt_proto->set_class_id(cv_kpt.class_id);
					kpt_proto->set_x(cv_kpt.pt.x);
					kpt_proto->set_y(cv_kpt.pt.y);
					kpt_proto->set_response(cv_kpt.response);
					kpt_proto->set_size(cv_kpt.size);
				}
				data_buffer->set_real_timestamp(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch()).count());

				string data_to_be_sent = data_buffer->SerializeAsString();
				std::cout << "data_to_be_sent size " << data_to_be_sent.length() << "\n";
				string delimitter = "END!";

				std::cout << "Serializationtakes " << (_m_clock->now() - start).count() << " ms\n";
				long created_to_sent = (_m_clock->now() - left_pts->timestamp).count();
				
				socket.write(data_to_be_sent + delimitter);
				time_point end = _m_clock->now();
				frame_info << frame_id << "," << created_to_sent << "," << (end - left_pts->timestamp).count() << "," << (end - start).count() << std::endl;

				delete data_buffer;
				data_buffer = new imu_features_proto::KeyPoints();
			}

			// if (feats_MSCKF != nullptr && feats_slam_UPDATE != nullptr && feats_slam_DELAYED != nullptr) {
			// 	time_point start = _m_clock->now();
			// 	if (feats_MSCKF->timestamp != feats_timestamp) { // && feats_slam_UPDATE->timestamp == cam_buffer_time && feats_slam_DELAYED->timestamp == cam_buffer_time
			// 		feats_timestamp = feats_MSCKF->timestamp;
			// 		// std::cout << "Start serializing features: \n";
			// 			/* Serialize features */
			// 		data_buffer->set_timestamp(feats_MSCKF->timestamp.time_since_epoch().count());
			// 		std::vector<feature> features_MSCKF = feats_MSCKF->feats;
			// 		for (feature f : features_MSCKF) {
			// 			imu_features_proto::Feature* ft = data_buffer->add_feats_msckf();
			// 			ft->set_featid(f.featid);
			// 			ft->set_to_delete(f.to_delete);
			// 			ft->set_anchor_cam_id(f.anchor_cam_id);
			// 			ft->set_anchor_clone_timestamp(f.anchor_clone_timestamp);
			// 			imu_features_proto::Vec3* p_FinA = new imu_features_proto::Vec3();
			// 			p_FinA->set_x(f.p_FinA.x());
			// 			p_FinA->set_y(f.p_FinA.y());
			// 			p_FinA->set_z(f.p_FinA.z());
			// 			ft->set_allocated_p_fina(p_FinA);
			// 			imu_features_proto::Vec3* p_FinG = new imu_features_proto::Vec3();
			// 			p_FinG->set_x(f.p_FinG.x());
			// 			p_FinG->set_y(f.p_FinG.y());
			// 			p_FinG->set_z(f.p_FinG.z());
			// 			ft->set_allocated_p_fing(p_FinG);
						
			// 			for (auto uv_elem : f.uvs) {
			// 				imu_features_proto::UVMapElem* uv_map_elem = ft->add_uv_map();
			// 				uv_map_elem->set_id(uv_elem.first);
			// 				for (auto uv: uv_elem.second) {
			// 					imu_features_proto::Vec2* v = uv_map_elem->add_uv();
			// 					v->set_x(uv.x());
			// 					v->set_y(uv.y());
			// 				}
			// 			}
			// 			for (auto uvn_elem : f.uvs_norm) {
			// 				imu_features_proto::UVMapElem* uvn_map_elem = ft->add_uvn_map();
			// 				uvn_map_elem->set_id(uvn_elem.first);
			// 				for (auto uvn: uvn_elem.second) {
			// 					imu_features_proto::Vec2* v = uvn_map_elem->add_uv();
			// 					v->set_x(uvn.x());
			// 					v->set_y(uvn.y());
			// 				}
			// 			}
			// 			for (auto t_elem : f.timestamps) {
			// 				imu_features_proto::TimestampMapElem* t_map_elem = ft->add_ts_map();
			// 				t_map_elem->set_id(t_elem.first);
			// 				for (auto t: t_elem.second) {
			// 					t_map_elem->add_timestamps(t);
			// 				}
			// 			}
			// 		}
			// 		data_buffer->set_real_timestamp(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch()).count());
			// 		string data_to_be_sent = data_buffer->SerializeAsString();
			// 		std::cout << "data_to_be_sent size " << data_to_be_sent.length() << "\n";
			// 		string delimitter = "END!";

			// 		std::cout << "Serializationtakes " << (_m_clock->now() - start).count() << " ms\n";
			// 		long created_to_sent = (_m_clock->now() - feats_timestamp).count();
					
			// 		socket.write(data_to_be_sent + delimitter);
			// 		time_point end = _m_clock->now();
			// 		frame_info << frame_id << "," << created_to_sent << "," << (end - feats_timestamp).count() << "," << (end - start).count() << std::endl;
			// 		// std::cout << "Serialization and Sending takes " << (_m_clock->now() - start).count() << " ms\n";
			// 		// std::cout << "feature timestamp is " << feats_timestamp.time_since_epoch().count() << " last imu timestamp is " << duration2double((datum->imus).at((datum->imus).size() - 1).timestamp.time_since_epoch()) << "\n";

			// 		delete data_buffer;
			// 		data_buffer = new imu_features_proto::IMUFeatures();
			// 	}
			// 	// cam_buffer_time = datum->time.time_since_epoch().count();
			// }
		// }
		
    }

private:
	long previous_timestamp = 0;
	int frame_id = 0;
	// imu_features_proto::IMUFeatures* data_buffer = new imu_features_proto::IMUFeatures();
	imu_features_proto::KeyPoints* data_buffer = new imu_features_proto::KeyPoints();
    const std::shared_ptr<switchboard> sb;
	const std::shared_ptr<RelativeClock> _m_clock;
	// switchboard::reader<features> _m_feats_MSCKF;
	// switchboard::reader<features> _m_feats_slam_UPDATE;
	// switchboard::reader<features> _m_feats_slam_DELAYED;

	switchboard::reader<key_points> _m_left_pts;
	switchboard::reader<key_points> _m_right_pts; 

	TCPSocket socket;
	Address server_addr;

	const string data_path = filesystem::current_path().string() + "/recorded_data";
	std::ofstream hashed_data;
	std::ofstream frame_info;

	// std::optional<long> cam_buffer_time;
	time_point feats_timestamp;
};

PLUGIN_MAIN(offload_writer)
