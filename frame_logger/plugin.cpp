#include <fstream>
#include "common/phonebook.hpp"
#include "common/plugin.hpp"
#include "common/switchboard.hpp"
#include "common/data_format.hpp"

using namespace ILLIXR;

// Inherit from `plugin` if you don't need the threadloop
class frame_logger : public plugin {
public:
    frame_logger(std::string name_, phonebook* pb_)
        : plugin{name_, pb_}
		, sb{pb->lookup_impl<switchboard>()}
		, _m_output_file {"frames.csv"}
		, _m_start{std::chrono::system_clock::now()}
	{
		_m_output_file
			<< "time"
			<< std::endl;
	}

	virtual void start() override {
		_m_start = std::chrono::system_clock::now();
		sb->schedule<time_type>(id, "m_vsync_estimate", [&](const time_type *datum) {
			this->feed_pose(datum);
		});
		plugin::start();
	}

	void feed_pose(const time_type *) {
		_m_output_file
			<< std::chrono::nanoseconds{std::chrono::system_clock::now() - _m_start}.count()
			<< std::endl;
	}

private:
	const std::shared_ptr<switchboard> sb;
	std::ofstream _m_output_file;
	std::chrono::system_clock::time_point _m_start;
};

// This line makes the plugin importable by Spindle
PLUGIN_MAIN(frame_logger);
