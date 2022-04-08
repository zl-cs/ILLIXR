#include <map>
#include <fstream>
#include <string>
#include <optional>
#include <math.h>

#include <opencv2/core/mat.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <eigen3/Eigen/Dense>

#include "common/dataset_tools/euroc_mav.hpp"
#include "common/dataset_tools/tum_vie.hpp"
#include "common/dataset_tools/tum_rgbd.hpp"
#include "common/error_util.hpp"


// timestamp
// p_RS_R_x [m], p_RS_R_y [m], p_RS_R_z [m]
// q_RS_w [], q_RS_x [], q_RS_y [], q_RS_z []
// v_RS_R_x [m s^-1], v_RS_R_y [m s^-1], v_RS_R_z [m s^-1]
// b_w_RS_S_x [rad s^-1], b_w_RS_S_y [rad s^-1], b_w_RS_S_z [rad s^-1]
// b_a_RS_S_x [m s^-2], b_a_RS_S_y [m s^-2], b_a_RS_S_z [m s^-2]

using namespace ILLIXR;

typedef pose_type sensor_types;

static
std::map<ullong, sensor_types>
load_data() {
	const char* illixr_data_c_str = std::getenv("ILLIXR_DATA");
	if (!illixr_data_c_str) {
        ILLIXR::abort("Please define ILLIXR_DATA");
	}
	std::string illixr_data = std::string{illixr_data_c_str};

	std::map<ullong, sensor_types> data;

	pose_data pose_loaded {load_euroc_mav_pose(illixr_data)};

	static constexpr double nano = 1e-9;
	for (const auto& pose : pose_loaded) {
		data[pose.time / nano] = {{},
								  {pose.pos_x, pose.pos_y, pose.pos_z},
								  {pose.orient_x, pose.orient_y, pose.orient_z, pose.orient_w}};
	}

	return data;
}
