#pragma once

#include <string>
#include <vector>

struct imu_element {
	double time;
	double gyro_x;
	double gyro_y;
	double gyro_z;
	double accel_x;
	double accel_y;
	double accel_z;
};

using imu_data = std::vector<imu_element>;

struct cam_element {
    double time;
    std::string path;
};

using cam_data = std::pair<std::vector<cam_element>, std::vector<cam_element>>;

struct pose_element {
	double time;
	double pos_x;
	double pos_y;
	double pos_z;
	double orient_x;
	double orient_y;
	double orient_z;
	double orient_w;
};

using pose_data = std::vector<pose_element>;
