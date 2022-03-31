#pragma once

#include <fstream>
#include <string>
#include <sstream>
#include <iomanip>

#include "imu_dataset.hpp"
#include "common/error_util.hpp"

imu_dataset load_tum_vie_imu(const std::string& dataset_path) {
	const std::string imu_subpath = "/imu_data.txt";
	std::ifstream imu_file {dataset_path + imu_subpath};
	if (!imu_file.good()) {
		std::cerr << dataset_path
                  << imu_subpath
                  << " is not a good path"
                  << std::endl;
        ILLIXR::abort();
	}
	static constexpr double micro {1e-6};
	imu_dataset dataset;
    std::string tmp;
    while (std::getline(imu_file, tmp)) {
        imu_data data;
        imu_file >> data.time;
        data.time *= micro;
        imu_file >> data.gyro_x;
        imu_file >> data.gyro_y;
        imu_file >> data.gyro_z;
        imu_file >> data.accel_x;
        imu_file >> data.accel_y;
        imu_file >> data.accel_z;
        dataset.emplace_back(std::move(data));
    }
	return dataset;
}

cam_dataset load_tum_vie_cam(const std::string& dataset_path) {
    const std::string cam0_subpath = "/left_images/image_timestamps_left.txt";
	std::ifstream cam0_file {dataset_path + cam0_subpath};
	if (!cam0_file.good()) {
		std::cerr << dataset_path
				  << cam0_subpath
				  << " is not a good path"
				  << std::endl;
        ILLIXR::abort();
	}
	const std::string cam1_subpath = "/right_images/image_timestamps_right.txt";
	std::ifstream cam1_file {dataset_path + cam1_subpath};
	if (!cam1_file.good()) {
		std::cerr << dataset_path
				  << cam1_subpath
				  << " is not a good path"
				  << std::endl;
        ILLIXR::abort();
	}
	static constexpr double micro {1e-6};
    cam_dataset dataset;
    std::string tmp;
    std::size_t count {0};
    while (std::getline(cam0_file, tmp)) {
        cam_data data;
        cam0_file >> data.time;
        data.time *= micro;
        std::stringstream ss;
        ss << std::setw(5) << std::setfill('0') << count++;
        data.path = dataset_path + "/left_images/" + ss.str() + ".jpg";
        dataset.first.emplace_back(std::move(data));
    }
    count = 0;
    while (std::getline(cam1_file, tmp)) {
        cam_data data;
        cam1_file >> data.time;
        data.time *= micro;
        std::stringstream ss;
        ss << std::setw(5) << std::setfill('0') << count++;
        data.path = dataset_path + "/right_images/" + ss.str() + ".jpg";
        dataset.second.emplace_back(std::move(data));
    }
    return dataset;
}
