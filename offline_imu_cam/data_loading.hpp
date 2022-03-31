#include <map>
#include <fstream>
#include <string>
#include <optional>
#include <cassert>

#include <opencv2/core/mat.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <eigen3/Eigen/Dense>

#include "cam_dataset.hpp"
#include "euroc_mav.hpp"
#include "imu_dataset.hpp"
#include "tum_vie.hpp"


typedef unsigned long long ullong;

typedef struct {
	Eigen::Vector3d angular_v;
	Eigen::Vector3d linear_a;
} raw_imu_type;

class lazy_load_image {
public:
	lazy_load_image(const std::string& path)
		: _m_path(path)
	{ }
	std::unique_ptr<cv::Mat> load() const {
		auto img = std::unique_ptr<cv::Mat>{new cv::Mat{cv::imread(_m_path, cv::IMREAD_GRAYSCALE)}};
		assert(!img->empty());
		return img;
	}

private:
	std::string _m_path;
};

typedef struct {
	std::optional<raw_imu_type> imu0;
	std::optional<lazy_load_image> cam0;
	std::optional<lazy_load_image> cam1;
} sensor_types;

static
std::map<ullong, sensor_types>
load_data() {
	const char* illixr_data_c_str = std::getenv("ILLIXR_DATA");
	if (!illixr_data_c_str) {
		std::cerr << "Please define ILLIXR_DATA" << std::endl;
		ILLIXR::abort();
	}
	std::string illixr_data = std::string{illixr_data_c_str};
	imu_dataset imu_loaded {load_euroc_mav_imu(illixr_data)};
	cam_dataset cam_loaded {load_euroc_mav_cam(illixr_data)};
	static constexpr double nano = 1e-9;
	std::map<ullong, sensor_types> data;
	for (const auto& imu : imu_loaded) {
		data[imu.time / nano].imu0 = {{imu.gyro_x, imu.gyro_y, imu.gyro_z},
                               				{imu.accel_x, imu.accel_y, imu.accel_z}};
	}
	for (const auto& cam : cam_loaded.first) {
		data[cam.time / nano].cam0 = {cam.path};
	}
	for (const auto& cam : cam_loaded.second) {
		data[cam.time / nano].cam1 = {cam.path};
	}
	return data;
}
