#pragma once

#include <fstream>
#include <string>
#include <sstream>
#include <iomanip>

#include "dataset.hpp"
#include "../error_util.hpp"

imu_data load_tum_vie_imu(const std::string& dataset_path) {
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
	imu_data data;
    std::string tmp;
    while (std::getline(imu_file, tmp)) {
        imu_element elem;
        imu_file >> elem.time;
        elem.time *= micro;
        imu_file >> elem.gyro_x;
        imu_file >> elem.gyro_y;
        imu_file >> elem.gyro_z;
        imu_file >> elem.accel_x;
        imu_file >> elem.accel_y;
        imu_file >> elem.accel_z;
        data.emplace_back(std::move(elem));
    }
	return data;
}

cam_data load_tum_vie_cam(const std::string& dataset_path) {
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
    cam_data data;
    std::string tmp;
    std::size_t count {0};
    while (std::getline(cam0_file, tmp)) {
        cam_element elem;
        cam0_file >> elem.time;
        elem.time *= micro;
        std::stringstream ss;
        ss << std::setw(5) << std::setfill('0') << count++;
        elem.path = dataset_path + "/left_images/" + ss.str() + ".jpg";
        data.first.emplace_back(std::move(elem));
    }
    count = 0;
    while (std::getline(cam1_file, tmp)) {
        cam_element elem;
        cam1_file >> elem.time;
        elem.time *= micro;
        std::stringstream ss;
        ss << std::setw(5) << std::setfill('0') << count++;
        elem.path = dataset_path + "/right_images/" + ss.str() + ".jpg";
        data.second.emplace_back(std::move(elem));
    }
    return data;
}

pose_data load_tum_vie_pose(const std::string& dataset_path) {
	const std::string pose_subpath {"/mocap_data.txt"};
	std::ifstream pose_file {dataset_path + pose_subpath};
	if (!pose_file.good()) {
		std::cerr << dataset_path
				  << pose_subpath
				  <<  " is not a good path"
				  << std::endl;
        ILLIXR::abort();
	}
	static constexpr double micro {1e-6};
	pose_data data;
    std::string tmp;
    while (std::getline(pose_file, tmp)) {
        pose_element elem;
        pose_file >> elem.time;
        elem.time *= micro;
        pose_file >> elem.pos_x;
        pose_file >> elem.pos_y;
        pose_file >> elem.pos_z;
        pose_file >> elem.orient_x;
        pose_file >> elem.orient_y;
        pose_file >> elem.orient_z;
		pose_file >> elem.orient_w;
        data.emplace_back(std::move(elem));
    }
	return data;
}
