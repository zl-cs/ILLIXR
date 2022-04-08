#pragma once

#include <fstream>
#include <string>
#include <sstream>
#include <iomanip>

#include "dataset.hpp"
#include "../error_util.hpp"

imu_data load_tum_rgbd_imu(const std::string& dataset_path) {
	const std::string imu_subpath = "/accelerometer.txt";
	std::ifstream imu_file {dataset_path + imu_subpath};
	if (!imu_file.good()) {
		std::cerr << dataset_path
                  << imu_subpath
                  << " is not a good path"
                  << std::endl;
        ILLIXR::abort();
	}
	imu_data data;
    std::string tmp;
    while (std::getline(imu_file, tmp)) {
        imu_element elem;
        imu_file >> elem.time;
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

cam_data load_tum_rgbd_cam(const std::string& dataset_path) {
    const std::string cam0_subpath = "/rgb.txt";
	std::ifstream cam0_file {dataset_path + cam0_subpath};
	if (!cam0_file.good()) {
		std::cerr << dataset_path
				  << cam0_subpath
				  << " is not a good path"
				  << std::endl;
        ILLIXR::abort();
	}
	const std::string cam1_subpath = "/rgb.txt";
	std::ifstream cam1_file {dataset_path + cam1_subpath};
	if (!cam1_file.good()) {
		std::cerr << dataset_path
				  << cam1_subpath
				  << " is not a good path"
				  << std::endl;
        ILLIXR::abort();
	}
    cam_data data;
    std::string tmp;
    std::size_t count {0};
    while (std::getline(cam0_file, tmp)) {
        cam_element elem;
        cam0_file >> elem.time;
		cam0_file >> elem.path;
		data.first.emplace_back(std::move(elem));
    }
    count = 0;
    while (std::getline(cam1_file, tmp)) {
		cam_element elem;
        cam1_file >> elem.time;
		cam1_file >> elem.path;
		data.first.emplace_back(std::move(elem));
    }
    return data;
}

pose_data load_tum_rgbd_pose(const std::string& dataset_path) {
	const std::string pose_subpath {"/groundtruth.txt"};
	std::ifstream pose_file {dataset_path + pose_subpath};
	if (!pose_file.good()) {
		std::cerr << dataset_path
				  << pose_subpath
				  <<  " is not a good path"
				  << std::endl;
        ILLIXR::abort();
	}
	pose_data data;
    std::string tmp;
    while (std::getline(pose_file, tmp)) {
        pose_element elem;
        pose_file >> elem.time;
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
