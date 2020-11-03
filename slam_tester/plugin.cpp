#include <map>
#include "common/phonebook.hpp"
#include "common/plugin.hpp"
#include "common/switchboard.hpp"
#include "common/data_format.hpp"
#include "data_loading.hpp"

using namespace ILLIXR;

// Inherit from `plugin` if you don't need the threadloop
class slam_tester : public plugin {
public:
    slam_tester(std::string name_, phonebook* pb_)
        : plugin{name_, pb_}
		, sb{pb->lookup_impl<switchboard>()}
		, _m_sensor_data{load_data()}
	{
		sb->schedule<pose_type>(id, "slow_pose", [&](const pose_type *datum) {
			this->feed_pose(datum);
		});
	}

	void feed_pose(const pose_type *datum) {
		ullong dataset_time = datum->dataset_time;
		auto it = _m_sensor_data.lower_bound(dataset_time);

		if (it == _m_sensor_data.cend()) {
			throw std::runtime_error{"True pose not found at timestamp: " + std::to_string(dataset_time)};
		} else if (dataset_time - it->first > 1000) {
			std::cout << "True pose to far away from timestamp: " + std::to_string(dataset_time) + " - " + std::to_string(it->first);
		} else {

			const pose_type *gt = &it->second;
			position_abs_err += (gt->position - datum->position).squaredNorm();
			orientation_abs_err += (gt->orientation).angularDistance(datum->orientation);
			if (old_datum) {
				position_rel_err += (gt->position - old_datum->position).squaredNorm();
				orientation_rel_err += (gt->orientation).angularDistance(old_datum->orientation);
			}
		}
		old_datum = *datum;
	}

	virtual ~slam_tester() {
		std::cout
			<< "position_abs_err,angular_a_abs_err,position_rel_err,angular_a_rel_err\n"
			<< position_abs_err << ","
			<< position_rel_err << ","
			<< orientation_abs_err << ","
			<< orientation_rel_err << std::endl;
	}

private:
	const std::shared_ptr<switchboard> sb;
	const std::map<ullong, sensor_types> _m_sensor_data;
	std::optional<pose_type> old_datum;
	float position_abs_err = 0;
	float position_rel_err = 0;
	float orientation_abs_err = 0;
	float orientation_rel_err = 0;
};

// This line makes the plugin importable by Spindle
PLUGIN_MAIN(slam_tester);
