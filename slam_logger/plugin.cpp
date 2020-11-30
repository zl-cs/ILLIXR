#include <fstream>
#include "common/phonebook.hpp"
#include "common/plugin.hpp"
#include "common/switchboard.hpp"
#include "common/data_format.hpp"

using namespace ILLIXR;

// Inherit from `plugin` if you don't need the threadloop
class slam_logger : public plugin {
public:
    slam_logger(std::string name_, phonebook* pb_)
        : plugin{name_, pb_}
		, sb{pb->lookup_impl<switchboard>()}
		, _m_output_file {"poses.csv"}
		, _m_frame_no{0}
	{
		_m_output_file
			<< "frame_no" << ","
			<< "dataset_time" << ","
			<< "position_x" << ","
			<< "position_y" << ","
			<< "position_z" << ","
			<< "orientation_w" << ","
			<< "orientation_x" << ","
			<< "orientation_y" << ","
			<< "orientation_z"
			<< std::endl;
		sb->schedule<pose_type>(id, "slow_pose", [&](const pose_type *datum) {
			this->feed_pose(datum);
		});
	}

	void feed_pose(const pose_type *datum) {
		_m_output_file
			<< _m_frame_no++ << ","
			<< datum->dataset_time << ","
			<< datum->position.x() << ","
			<< datum->position.y() << ","
			<< datum->position.z() << ","
			<< datum->orientation.w() << ","
			<< datum->orientation.x() << ","
			<< datum->orientation.y() << ","
			<< datum->orientation.z()
			<< std::endl;
	}

private:
	const std::shared_ptr<switchboard> sb;
	std::ofstream _m_output_file;
	std::size_t _m_frame_no;

};

// This line makes the plugin importable by Spindle
PLUGIN_MAIN(slam_logger);
