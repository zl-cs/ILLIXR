#include "common/switchboard.hpp"
#include "common/data_format.hpp"
#include "data_loading.hpp"
#include "common/data_format.hpp"
#include "common/threadloop.hpp"
#include "common/global_module_defs.hpp"
#include <cassert>

using namespace ILLIXR;

const record_header imu_cam_record {
	"imu_cam",
	{
		{"iteration_no", typeid(std::size_t)},
		{"has_depth", typeid(bool)},
		{"has_color", typeid(bool)},
	},
};

class offline_TUM : public ILLIXR::threadloop {
public:
	offline_TUM(std::string name_, phonebook* pb_)
		: threadloop{name_, pb_}
		, _m_sensor_data{load_data()}
		, _m_sensor_data_it{_m_sensor_data.cbegin()}
		, _m_sb{pb->lookup_impl<switchboard>()}
		, _m_rgb_depth{_m_sb->get_writer<rgb_depth_type>("rgb_depth")} 
		, dataset_first_time{_m_sensor_data_it->first}
		, imu_cam_log{record_logger_}
		, camera_cvtfmt_log{record_logger_}
	{}

protected:

	virtual skip_option _p_should_skip() override {
		if (_m_sensor_data_it != _m_sensor_data.end()) {
			dataset_now = _m_sensor_data_it->first;
			//std::cout<<"sleep for : "<<(dataset_now - dataset_prev) <<std::endl;
			//dataset_prev = dataset_now;
			std::this_thread::sleep_for(
				std::chrono::nanoseconds{dataset_now - dataset_first_time}
				+ real_first_time
				- std::chrono::high_resolution_clock::now()
			);
            		//in the TUM case, since we have no actual imu reading we use depth image to filter
			if (_m_sensor_data_it->second.cam0) {
				return skip_option::run;
			} else {
				++_m_sensor_data_it;
				return skip_option::skip_and_yield;
			}

		} else {
			return skip_option::stop;
		}
	}

	virtual void _p_one_iteration() override {
	    RAC_ERRNO_MSG("offline_TUM at start of _p_one_iteration");
		assert(_m_sensor_data_it != _m_sensor_data.end());
#ifndef NDEBUG
        std::chrono::time_point<std::chrono::nanoseconds> tp_dataset_now{std::chrono::nanoseconds{dataset_now}};
		std::cerr << " IMU time: " << tp_dataset_now.time_since_epoch().count() << std::endl;
#endif /// NDEBUG
//		time_type real_now = real_first_time + std::chrono::nanoseconds{dataset_now - dataset_first_time};
//		std::cout<<"dataset now: "<<dataset_now<<" count: "<<count<<std::endl;
		count++;
		const sensor_types& sensor_datum = _m_sensor_data_it->second;
		++_m_sensor_data_it;

		imu_cam_log.log(record{imu_cam_record, {
			{iteration_no},
			{bool(sensor_datum.cam0)},
			{bool(sensor_datum.cam1)},
		}});

		std::optional<cv::Mat> depth = sensor_datum.cam0
			? std::make_optional<cv::Mat>(*(sensor_datum.cam0.value().unmodified_load().release()))
			: std::nullopt
			;
		RAC_ERRNO_MSG("offline_TUM after depth");
		std::optional<cv::Mat> rgb = sensor_datum.cam1
			? std::make_optional<cv::Mat>(*(sensor_datum.cam1.value().modified_load().release()))
			: std::nullopt
			;
		RAC_ERRNO_MSG("offline_TUM after rgb");
		
        
        _m_rgb_depth.put(_m_rgb_depth.allocate<rgb_depth_type>(
            rgb_depth_type {
                rgb,
                depth,
                dataset_now
            }
        ));
		RAC_ERRNO_MSG("offline_TUM at bottom of iteration");
	}

public:
	virtual void _p_thread_setup() override {
		// this is not done in the constructor, because I want it to
		// be done at thread-launch time, not load-time.
		auto now = std::chrono::system_clock::now();
		real_first_time = std::chrono::time_point_cast<std::chrono::seconds>(now);
	}

private:
	const std::map<ullong, sensor_types> _m_sensor_data;
	std::map<ullong, sensor_types>::const_iterator _m_sensor_data_it;
	const std::shared_ptr<switchboard> _m_sb;
	switchboard::writer<rgb_depth_type> _m_rgb_depth;

	// Timestamp of the first IMU value from the dataset
	ullong dataset_first_time;
	// UNIX timestamp when this component is initialized
	time_type real_first_time;
	// Current IMU timestamp
	ullong dataset_now;
    //ullong dataset_prev=0;
    //track triggered 
    unsigned count=0;
	record_coalescer imu_cam_log;
	record_coalescer camera_cvtfmt_log;
};

PLUGIN_MAIN(offline_TUM)
