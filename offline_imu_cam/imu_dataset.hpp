#pragma once

#include <vector>

struct imu_data {
	double time;
	double gyro_x;
	double gyro_y;
	double gyro_z;
	double accel_x;
	double accel_y;
	double accel_z;
};

using imu_dataset = std::vector<imu_data>;
