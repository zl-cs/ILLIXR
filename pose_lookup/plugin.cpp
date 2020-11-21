#include <shared_mutex>
#include "common/phonebook.hpp"
#include "common/pose_prediction.hpp"
#include "common/data_format.hpp"
#include "common/plugin.hpp"

/*pyh: reusing data_loading from ground_truth_slam*/
#include "data_loading.hpp"

#define ALIGN

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
    	auto newoffset = correct_pose(_m_sensor_data_it->second).orientation;
    	set_offset(newoffset);
	init_pos_offset = correct_pose(_m_sensor_data_it->second).position;

	raw_init_pos_offset = _m_sensor_data_it->second.position;

	// could write a line to automatically read align matrix from file later
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
	Eigen::Vector3f raw_init_pos_offset {0};
	mutable Eigen::Quaternionf offset {Eigen::Quaternionf::Identity()};
	mutable std::shared_mutex offset_mutex;

	/*pyh: reusing data_loading from ground_truth_slam*/
	const std::map<ullong, sensor_types> _m_sensor_data;
	std::map<ullong, sensor_types>::const_iterator _m_sensor_data_it;
	ullong dataset_first_time;
	time_type _m_start_of_time;
	std::unique_ptr<reader_latest<time_type>> _m_vsync_estimate;

        Eigen::Matrix<float, 3, 3> skew_x(const Eigen::Matrix<float, 3, 1> &w) const
	{
		Eigen::Matrix<float, 3, 3> w_x;
		w_x << 0, -w(2), w(1),
			w(2), 0, -w(0),
			-w(1), w(0), 0;
		return w_x;
	}

	Eigen::Matrix<float, 4, 1> quat_inv(Eigen::Matrix<float, 4, 1> q) const
	{
		Eigen::Matrix<float, 4, 1> qinv;
		qinv.block(0, 0, 3, 1) = -q.block(0, 0, 3, 1);
		qinv(3, 0) = q(3, 0);
		return qinv;
	}

        Eigen::Matrix<float, 4, 1> quat_multiply(const Eigen::Matrix<float, 4, 1> &q, const Eigen::Matrix<float, 4, 1> &p) const
	{
		Eigen::Matrix<float, 4, 1> q_t;
		Eigen::Matrix<float, 4, 4> Qm;
		// create big L matrix
		Qm.block(0, 0, 3, 3) = q(3, 0) * Eigen::MatrixXf::Identity(3, 3) - skew_x(q.block(0, 0, 3, 1));
		Qm.block(0, 3, 3, 1) = q.block(0, 0, 3, 1);
		Qm.block(3, 0, 1, 3) = -q.block(0, 0, 3, 1).transpose();
		Qm(3, 3) = q(3, 0);
		q_t = Qm * p;
		// ensure unique by forcing q_4 to be >0
		if (q_t(3, 0) < 0)
		{
			q_t *= -1;
		}
		// normalize and return
		return q_t / q_t.norm();
	}

	pose_type correct_pose(const pose_type pose) const {
		pose_type swapped_pose;
		pose_type aligned_pose;


#ifdef ALIGN
		std::cout << "Aligned pose lookup !!!" << std::endl;

		pose_type temp_pose;

		temp_pose.position(0) = pose.position.x() - raw_init_pos_offset(0);
		temp_pose.position(1) = pose.position.y() - raw_init_pos_offset(1);
		temp_pose.position(2) = pose.position.z() - raw_init_pos_offset(2);

		Eigen::Matrix3f R_ESTtoGT;

		R_ESTtoGT(0, 0) = 0.92096;
		R_ESTtoGT(0, 1) = -0.389658;
		R_ESTtoGT(0, 2) = 0;
		R_ESTtoGT(1, 0) = 0.389658;
		R_ESTtoGT(1, 1) = 0.92096;
		R_ESTtoGT(1, 2) = 0;
		R_ESTtoGT(2, 0) = 0;
		R_ESTtoGT(2, 1) = 0;
		R_ESTtoGT(2, 2) = 1;

		Eigen::Vector3f t_ESTinGT = {0.0645829, 0.00112691, 0.160771};
		Eigen::Vector4f q_ESTtoGT = {0, 0, -0.198797, 0.980041};

		float s_ESTtoGT = 1.0;

		// enforce the trajectories start from (0, 0, 0)
		// Eigen::Vector3f zero_offset = {-0.0282369, 0.0144951, -0.150525};
		/////////////////////////////////////////////////////////////////////////////////////////////////////




		Eigen::Matrix<float,7,1> pose_ESTinGT;
		aligned_pose.position = s_ESTtoGT * R_ESTtoGT * temp_pose.position + t_ESTinGT;//  + zero_offset;
		// aligned_pose.position = s_ESTtoGT * R_ESTtoGT * temp_pose.position + t_ESTinGT + zero_offset;


		Eigen::Vector4f quat = {pose.orientation.x(), pose.orientation.y(), pose.orientation.z(), pose.orientation.w()};
		// Eigen::Vector4f quat = {pose.orientation.w(), pose.orientation.x(), pose.orientation.y(), pose.orientation.z()};
		Eigen::Vector4f quat_out;

		// pose_ESTinGT.block(3,0,4,1) = quat_multiply(pose.orientation, quat_inv(q_ESTtoGT));

		// pose_ESTinGT.block(3,0,4,1) = quat_multiply(quat, quat_inv(q_ESTtoGT));
		quat_out = quat_multiply(quat, quat_inv(q_ESTtoGT));

		// std::cout << "quat: " << quat_out(0) << " " << quat_out(1) << std::endl;
		/*
		aligned_pose.orientation.w() = quat_out(0);
		aligned_pose.orientation.x() = quat_out(1);
		aligned_pose.orientation.y() = quat_out(2);
		aligned_pose.orientation.z() = quat_out(3);
		*/
		aligned_pose.orientation.x() = quat_out(0);
		aligned_pose.orientation.y() = quat_out(1);
		aligned_pose.orientation.z() = quat_out(2);
		aligned_pose.orientation.w() = quat_out(3);

		swapped_pose.position.x() = -aligned_pose.position.y();
		swapped_pose.position.y() = aligned_pose.position.z();
		swapped_pose.position.z() = -aligned_pose.position.x();

		Eigen::Quaternionf raw_o (aligned_pose.orientation.w(), -aligned_pose.orientation.y(), aligned_pose.orientation.z(), -aligned_pose.orientation.x());

		swapped_pose.orientation = apply_offset(raw_o);
#endif


#ifndef ALIGN
		std::cout << "No alignment !!!" << std::endl;
		// This uses the OpenVINS standard output coordinate system.
		// This is a mapping between the OV coordinate system and the OpenGL system.
		swapped_pose.position.x() = -pose.position.y();
		swapped_pose.position.y() = pose.position.z();
		swapped_pose.position.z() = -pose.position.x();


		// There is a slight issue with the orientations: basically,
		// the output orientation acts as though the "top of the head" is the
		// forward direction, and the "eye direction" is the up direction.
		Eigen::Quaternionf raw_o (pose.orientation.w(), -pose.orientation.y(), pose.orientation.z(), -pose.orientation.x());

		swapped_pose.orientation = apply_offset(raw_o);


		swapped_pose.position(0) -= init_pos_offset(0);
		swapped_pose.position(1) -= init_pos_offset(1);
		swapped_pose.position(2) -= init_pos_offset(2);


#endif

		return swapped_pose;
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
