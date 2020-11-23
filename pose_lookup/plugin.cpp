#include <shared_mutex>
#include "common/phonebook.hpp"
#include "common/pose_prediction.hpp"
#include "common/data_format.hpp"
#include "common/plugin.hpp"

#include "utils.hpp"
#include "data_loading.hpp"

// Activate ALIGN when collecting texture image with aligned trajectory
// Set path_to_alignment to alignment file when activate ALIGN
// Deactivate ALIGN when running original ILLIXR starting from position (0, 0, 0)

// #define ALIGN

using namespace ILLIXR;

class pose_lookup_impl : public pose_prediction {
public:
	pose_lookup_impl(const phonebook* const pb)
		: sb{pb->lookup_impl<switchboard>()}
	, _m_sensor_data{load_data()}
	, _m_sensor_data_it{_m_sensor_data.cbegin()}
	, dataset_first_time{_m_sensor_data_it->first}
	, _m_start_of_time{std::chrono::high_resolution_clock::now()}
	, _m_vsync_estimate{sb->subscribe_latest<time_type>("vsync_estimate")}
	{
		load_align_parameters(path_to_alignment);
		// raw position data for first frame
		init_pos_offset = _m_sensor_data_it->second.position;

		auto newoffset = correct_pose(_m_sensor_data_it->second).orientation;
		set_offset(newoffset);
	}

	virtual fast_pose_type get_fast_pose() const override {
	return get_fast_pose( std::chrono::system_clock::now() );
	}

	virtual pose_type get_true_pose() const override {
	throw std::logic_error{"Not Implemented"};
	}


	virtual bool fast_pose_reliable() const override {
	return true;
	}

	virtual bool true_pose_reliable() const override {
	return false;
	}

	virtual void set_offset(const Eigen::Quaternionf& raw_o_times_offset) override{
	std::unique_lock lock {offset_mutex};
	Eigen::Quaternionf raw_o = raw_o_times_offset * offset.inverse();
	//std::cout << "pose_prediction: set_offset" << std::endl;
	offset = raw_o.inverse();
	}

	Eigen::Quaternionf apply_offset(const Eigen::Quaternionf& orientation) const {
	std::shared_lock lock {offset_mutex};
	return orientation * offset;
	}
	virtual fast_pose_type get_fast_pose([[maybe_unused]] time_type time) const override {
		const time_type* estimated_vsync = _m_vsync_estimate->get_latest_ro();
		time_type vsync;
		if(estimated_vsync == nullptr) {
			std::cerr << "Vsync estimation not valid yet, returning fast_pose for now()" << std::endl;
			vsync = std::chrono::system_clock::now();
		} else {
			vsync = *estimated_vsync;
		}

		ullong lookup_time = std::chrono::nanoseconds(vsync - _m_start_of_time ).count() + dataset_first_time;
		ullong  nearest_timestamp = 0;

		if(lookup_time <= _m_sensor_data.begin()->first){
			std::cerr << "Lookup time before first datum" << std::endl;
			nearest_timestamp=_m_sensor_data.begin()->first;
		}
		else if (lookup_time >= _m_sensor_data.rbegin()->first){
			std::cerr << "Lookup time after last datum" << std::endl;
			nearest_timestamp=_m_sensor_data.rbegin()->first;
		}
		else{
			std::map<ullong, sensor_types>::const_iterator _m_sensor_data_it_local;
			_m_sensor_data_it_local = _m_sensor_data.cbegin();
			ullong prev_timestamp = _m_sensor_data_it_local->first;
			while ( _m_sensor_data_it_local != _m_sensor_data.end() ){
				ullong cur_timestamp = _m_sensor_data_it_local->first;
				if(lookup_time > cur_timestamp){
					prev_timestamp = cur_timestamp;
					++_m_sensor_data_it_local;
				}
				else{
					if((lookup_time - prev_timestamp) >= (cur_timestamp - lookup_time) ){
						nearest_timestamp = cur_timestamp;
						break;
					}
					else{
						nearest_timestamp = prev_timestamp;
						break;
					}
				}
			}
			// Assert while loop "break"ed out (not normal exit)
			assert(nearest_timestamp);
		}


		auto looked_up_pose = _m_sensor_data.find(nearest_timestamp)->second;
		looked_up_pose.sensor_time = _m_start_of_time + std::chrono::nanoseconds{nearest_timestamp - dataset_first_time};

		return fast_pose_type{
			.pose = correct_pose(looked_up_pose),
			.predict_computed_time = std::chrono::system_clock::now(),
			.predict_target_time = vsync
		};

	}


private:
	const std::shared_ptr<switchboard> sb;

	Eigen::Vector3f init_pos_offset {0};
	mutable Eigen::Quaternionf offset {Eigen::Quaternionf::Identity()};
	mutable std::shared_mutex offset_mutex;

	const std::map<ullong, sensor_types> _m_sensor_data;
	std::map<ullong, sensor_types>::const_iterator _m_sensor_data_it;
	ullong dataset_first_time;
	time_type _m_start_of_time;
	std::unique_ptr<reader_latest<time_type>> _m_vsync_estimate;


	Eigen::Matrix3f align_rot;
	Eigen::Vector3f	align_trans;
	Eigen::Vector4f	align_quat;
	double align_scale;
	// change path to alignment file here
	std::string path_to_alignment = "/home/huzaifa2/all_scratch/ILLIXR-Evaluation/alignTrajectory/alignMatrix.txt";


	void load_align_parameters(std::string path)
	{
		std::ifstream infile;
		infile.open(path);
		if (!infile.is_open())
			std::cout << "Open align file failed !!!" << std::endl;

		std::string in;
		std::deque<float> parameters;
		while (std::getline(infile, in))
		{
			parameters.resize(0);
			read_line(in, parameters);

			if (strstr(in.c_str(), "Rotation"))
			{ assign_matrix(parameters, align_rot); continue; }

			if (strstr(in.c_str(), "Translation"))
			{ assign_matrix(parameters, align_trans); continue; }

			if (strstr(in.c_str(), "Quaternion"))
			{ assign_matrix(parameters, align_quat); continue; }

			if (strstr(in.c_str(), "Scale"))
			{ align_scale = parameters[0]; continue; }
		}
		infile.close();
	}


	pose_type correct_pose(pose_type pose) const {
		pose_type output_pose;

		// step 1: correct starting point to (0, 0, 0), pos only
		pose.position(0) -= init_pos_offset(0);
		pose.position(1) -= init_pos_offset(1);
		pose.position(2) -= init_pos_offset(2);

#ifdef ALIGN
		// step 2: apply estimated align transformation parameters
		// step 2.1: position alignment
		pose.position = align_scale * align_rot * pose.position + align_trans;

		// step 2.2: orientation alignment
		Eigen::Vector4f quat_in = {pose.orientation.x(), pose.orientation.y(), pose.orientation.z(), pose.orientation.w()};
		Eigen::Vector4f quat_out = ori_multiply(quat_in, ori_inv(align_quat));
		pose.orientation.x() = quat_out(0);
		pose.orientation.y() = quat_out(1);
		pose.orientation.z() = quat_out(2);
		pose.orientation.w() = quat_out(3);
#endif

		// step 3: swap axis for both position and orientation
		// step 3.1: swap for position
		output_pose.position.x() = -pose.position.y();
		output_pose.position.y() = pose.position.z();
		output_pose.position.z() = -pose.position.x();

		// step3.2: swap for orientation
		Eigen::Quaternionf raw_o (pose.orientation.w(), -pose.orientation.y(), pose.orientation.z(), -pose.orientation.x());
		output_pose.orientation = apply_offset(raw_o);

		return output_pose;
	}
};

class pose_lookup_plugin : public plugin {
public:
	pose_lookup_plugin(const std::string& name, phonebook* pb)
		: plugin{name, pb}
	{
		pb->register_impl<pose_prediction>(
			std::static_pointer_cast<pose_prediction>(
				std::make_shared<pose_lookup_impl>(pb)
			)
		);
	}
};

PLUGIN_MAIN(pose_lookup_plugin);
