#pragma once

#include <fstream>
#include <string>

#include "dataset.hpp"
#include "csv_iterator.hpp"
#include "common/error_util.hpp"

imu_data load_euroc_mav_imu(const std::string& dataset_path) {
	const std::string imu0_subpath = "/imu0/data.csv";
	std::ifstream imu0_file {dataset_path + imu0_subpath};
	if (!imu0_file.good()) {
		std::cerr << dataset_path
				  << imu0_subpath
				  << " is not a good path"
				  << std::endl;
		ILLIXR::abort();
	}
	static constexpr double nano = 1e-9;
	imu_data data;
	for (CSVIterator row{imu0_file, 1}; row != CSVIterator{}; ++row) {
		imu_element elem {std::stod(row[0]) * nano,
			              std::stod(row[1]),
						  std::stod(row[2]),
						  std::stod(row[3]),
						  std::stod(row[4]),
						  std::stod(row[5]),
						  std::stod(row[6])};
		data.emplace_back(std::move(elem));
	}
	return data;
}

cam_data load_euroc_mav_cam(const std::string& dataset_path) {
	const std::string cam0_subpath = "/cam0/data.csv";
	std::ifstream cam0_file {dataset_path + cam0_subpath};
	if (!cam0_file.good()) {
		std::cerr << dataset_path
				  << cam0_subpath
				  << " is not a good path"
				  << std::endl;
        ILLIXR::abort();
	}
	const std::string cam1_subpath = "/cam1/data.csv";
	std::ifstream cam1_file {dataset_path + cam1_subpath};
	if (!cam1_file.good()) {
		std::cerr << dataset_path
				  << cam1_subpath
				  << " is not a good path"
				  << std::endl;
        ILLIXR::abort();
	}
	static constexpr double nano = 1e-9;
	cam_data data;
	for (CSVIterator row{cam0_file, 1}; row != CSVIterator{}; ++row) {
		cam_element elem {std::stod(row[0]) * nano,
					      dataset_path + "/cam0/data/" + row[1]};
		data.first.emplace_back(std::move(elem));
	}
	for (CSVIterator row{cam1_file, 1}; row != CSVIterator{}; ++row) {
		cam_element elem {std::stod(row[0]) * nano,
					      dataset_path + "/cam1/data/" + row[1]};
		data.second.emplace_back(std::move(elem));
	}
	return data;
}

pose_data load_euroc_mav_pose(const std::string& dataset_path) {
	const std::string pose_subpath {"/state_groundtruth_estimate0/data.csv"};
	std::ifstream pose_file {dataset_path + pose_subpath};
	if (!pose_file.good()) {
		std::cerr << dataset_path
				  << pose_subpath
				  <<  " is not a good path"
				  << std::endl;
        ILLIXR::abort();
	}
	static constexpr double nano = 1e-9;
	pose_data data;
	for(CSVIterator row{pose_file, 1}; row != CSVIterator{}; ++row) {
		pose_element elem {std::stod(row[0]) * nano,
			               std::stod(row[1]),
						   std::stod(row[2]),
						   std::stod(row[3]),
						   std::stod(row[4]),
						   std::stod(row[5]),
						   std::stod(row[6]),
		                   std::stod(row[7])};
		data.emplace_back(std::move(elem));
	}
	return data;
}
