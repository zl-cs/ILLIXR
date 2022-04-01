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
