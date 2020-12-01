#include "phonebook.hpp"
#include "data_format.hpp"
#include "math_util.hpp"

namespace ILLIXR {

	// The scene looks good from this angle
	static const Eigen::Quaternionf target_initial_ori = Eigen::Quaternionf{1, 0, 0, 0};
	// Pulled from first row of groundtruth
	static const Eigen::Quaternionf actual_initial_ori = Eigen::Quaternionf{0.161996,0.789985,-0.205376,0.554528};
	static const Eigen::Quaternionf ori_offset = target_initial_ori * actual_initial_ori.inverse();
	static const auto ori_offset_mat = ori_offset.toRotationMatrix();
	// The scene looks good from this position
	static const Eigen::Vector3f target_initial_pos = Eigen::Vector3f{0.5, 0.5, 0.5};
	// Pulled from first row of groundtruth
	static const Eigen::Vector3f actual_initial_pos = Eigen::Vector3f{0.515356,1.996773,0.971104};
	static const Eigen::Vector3f pos_offset = target_initial_pos - ori_offset_mat * actual_initial_pos;

	// Note that Eigen prints as xyzw, but this constructor is wxyz
	static const Eigen::Quaternionf q_ESTtoGT (0.977528687278213, -0, -0, -0.210802432500513);
	static const Eigen::Vector3f t_ESTinGT (0.58844, 2.02848, 0.962197);
	static bool first_time_gt = true;
	static bool first_time_ov = true;

	// These values are taken by:n
	// - Enable OV.
	// - Enable slam_logger.
	// - Run ILLIXR.
	// - Run align-trajectory, using gt from EuRoC and poses.csv.
	// - Copy values from alignMatrix.txt to here.
	static const float s_ESTtoGT = 1.0;
	static Eigen::Matrix3f R_ESTtoGT() {
		Eigen::Matrix3f ret;
		ret << 0.911125, 0.412131, 0.0,
			-0.412131, 0.911125, 0.0,
			0.0, 0.0, 1.0;
		return ret;
	};

class pose_prediction : public phonebook::service {
public:
	virtual fast_pose_type get_fast_pose() const = 0;
	virtual pose_type get_true_pose() const = 0;
	virtual fast_pose_type get_fast_pose(time_type future_time) const = 0;
	virtual bool fast_pose_reliable() const = 0;
	virtual bool true_pose_reliable() const = 0;
	virtual ~pose_prediction() { }

protected:

	/**
	 * Transforms from "ground truth" frame into frame which displays better
	 */
	static pose_type gt_transform(const pose_type& pose) {
		if (first_time_gt) {
			using namespace math_util;

			// Assert that these are normalized
			assert(is_close(actual_initial_ori.norm(), 1));
			assert(is_close(target_initial_ori.norm(), 1));

			// Assert that the initial pose is the "target" initial pose
			assert(all_close(pos_offset + ori_offset_mat * actual_initial_pos, target_initial_pos));
			assert(all_close(ori_offset * actual_initial_ori, target_initial_ori));

			first_time_gt = false;
		}

		return {
			.sensor_time = pose.sensor_time,
			.position = pos_offset + ori_offset_mat * pose.position,
			.orientation = ori_offset * pose.orientation,
			.dataset_time = pose.dataset_time,
		};
	}

	/**
	 * Transforms from OpenVINS output frame to the "ground truth" frame
	 *
	 * Uses parameters from "align-trajectories".
	 *
	 * It makes sense to rectify the pose in two steps (ov frame -> gt
	 * frame -> display good frame) because we may want to compare
	 * pose error, which requires both poses to be in the gt frame. We
	 * also want to reuse the exact same "good frame" parameters
	 * between gt SLAM and ov SLAM, so it makes sense to put both
	 * SLAMs into a common frame, and translate both of those.
	 */
	static pose_type ov_transform(const pose_type& pose) {
		if (first_time_ov) {
			using namespace math_util;

			// Assert that the orientation is getting the same rotational transformation that the position is.
			assert(allclose(q_ESTtoGT.toRotationMatrix(), R_ESTtoGT));

			first_time_ov = false;
		}

		return {
			.sensor_time = pose.sensor_time,
			.position = s_ESTtoGT*R_ESTtoGT()*pose.position+t_ESTinGT,
			.orientation = pose.orientation * q_ESTtoGT.inverse(),
			// .orientation = q_ESTtoGT * pose.orientation,
			.dataset_time = pose.dataset_time,
		};
	}

};
}
