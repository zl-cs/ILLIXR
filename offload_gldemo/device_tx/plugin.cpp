#include <chrono>
#include <future>
#include <iostream>
#include <fstream> 
#include <thread>

#include "common/threadloop.hpp"
#include "common/switchboard.hpp"
#include "common/data_format.hpp"
#include "common/pose_prediction.hpp"
#include "common/relative_clock.hpp"

#include <ecal/ecal.h>
#include <ecal/msg/protobuf/publisher.h>

#include "gldemo_input.pb.h"


using namespace ILLIXR;

static constexpr duration VSYNC_PERIOD {freq2period(60.0)};
static constexpr duration VSYNC_DELAY_TIME {std::chrono::milliseconds{2}};
class client_writer : public threadloop {
public:

	client_writer(std::string name_, phonebook* pb_)
		: threadloop{name_, pb_}
		, sb{pb->lookup_impl<switchboard>()}
		, pp{pb->lookup_impl<pose_prediction>()}
		, _m_clock{pb->lookup_impl<RelativeClock>()}
		, _m_fast_pose{sb->get_writer<switchboard::event_wrapper<fast_pose_type>>("fast_pose")}
		, _m_vsync{sb->get_reader<switchboard::event_wrapper<time_point>>("vsync_estimate")}
	{ 
		eCAL::Initialize(0, NULL, "GLdemo Offloading Device-side Writer");
		publisher = eCAL::protobuf::CPublisher<gldemo_input_proto::Pose>("gldemo_input");
	}
	~client_writer() { }

	// Essentially, a crude equivalent of XRWaitFrame.
	// Should wait on the clent side on the server side? 
	void wait_vsync()
	{
		using namespace std::chrono_literals;
		switchboard::ptr<const switchboard::event_wrapper<time_point>> next_vsync = _m_vsync.get_ro_nullable();
		time_point now = _m_clock->now();

		time_point wait_time;

		if (next_vsync == nullptr) {
			// If no vsync data available, just sleep for roughly a vsync period.
			// We'll get synced back up later.
			std::this_thread::sleep_for(VSYNC_PERIOD);
			return;
		}

		bool hasRenderedThisInterval = (now - lastFrameTime) < VSYNC_PERIOD;

		// If less than one frame interval has passed since we last rendered...
		if (hasRenderedThisInterval)
		{
			// We'll wait until the next vsync, plus a small delay time.
			// Delay time helps with some inaccuracies in scheduling.
			wait_time = **next_vsync + VSYNC_DELAY_TIME; // why delay? 

			// If our sleep target is in the past, bump it forward
			// by a vsync period, so it's always in the future.
			while(wait_time < now)
			{
				wait_time += VSYNC_PERIOD;
			}

			// Perform the sleep.
			// TODO: Consider using Monado-style sleeping, where we nanosleep for
			// most of the wait, and then spin-wait for the rest?
			std::this_thread::sleep_for(wait_time-now);
		} else {
			// render immediately 
		}
	}

	void _p_thread_setup() override {
		RAC_ERRNO_MSG("offload gldemo at start of _p_thread_setup");

		RAC_ERRNO_MSG("offload gldemo at end of _p_thread_setup");
	}

	void _p_one_iteration() override {
		{
			using namespace std::chrono_literals;

			// Essentially, XRWaitFrame.
			wait_vsync();

			const fast_pose_type fast_pose = pp->get_fast_pose();
			_m_fast_pose.put(_m_fast_pose.allocate<switchboard::event_wrapper<fast_pose_type>>( // FIXME fast_pose_type is not inherited from switchboard::event 
				fast_pose_type{
					fast_pose.pose,
					fast_pose.predict_computed_time,
					fast_pose.predict_target_time
				}
			)); 
			// auto fast_pose_sample_time = _m_clock.now();
			// _m_fast_pose_sample_time.put(_m_fast_pose_sample_time.allocate<switchboard::event_wrapper<time_point>>(
			// 	fast_pose_sample_time
			// )); 
			pose_type pose = fast_pose.pose;

			// TODO SEND POSE TO THE SERVER SIDE 
			gldemo_input_proto::Vec3f* position = new gldemo_input_proto::Vec3f(); 
			position->set_x(pose.position(0));
			position->set_y(pose.position(1));
			position->set_z(pose.position(2));

			gldemo_input_proto::Quaternionf* orientation = new gldemo_input_proto::Quaternionf();
			orientation->set_w(pose.orientation.w()); 
			Eigen::Vector3f rot = pose.orientation.vec();
			gldemo_input_proto::Vec3f* rotation = new gldemo_input_proto::Vec3f();
			rotation->set_x(rot(0));
			rotation->set_y(rot(1));
			rotation->set_z(rot(2)); 
			orientation->set_allocated_vec(rotation); 

			gldemo_input_proto::Pose* pose_to_gldemo = new gldemo_input_proto::Pose(); 
			pose_to_gldemo->set_allocated_position(position);
			pose_to_gldemo->set_allocated_orientation(orientation); 
			publisher.Send(*pose_to_gldemo);
			delete pose_to_gldemo; 

            
			poses.push_back(pose); 

			lastFrameTime = _m_clock->now(); 
			// FIXME not exactly the lastFrameTime because the rendering has not finished; Can be used to estimate the offloading time
		}
	}

private:
	const std::shared_ptr<switchboard> sb;
	const std::shared_ptr<pose_prediction> pp;
	const std::shared_ptr<const RelativeClock> _m_clock;
	switchboard::writer<switchboard::event_wrapper<fast_pose_type>> _m_fast_pose; 
	const switchboard::reader<switchboard::event_wrapper<time_point>> _m_vsync;

	// Switchboard plug for application eye buffer.
	// We're not "writing" the actual buffer data,
	// we're just atomically writing the handle to the
	// correct eye/framebuffer in the "swapchain".
	// switchboard::writer<rendered_frame> _m_eyebuffer;

	time_point lastFrameTime;
	std::vector<pose_type> poses; 

	eCAL::protobuf::CPublisher<gldemo_input_proto::Pose> publisher;

public:
// 	/* compatibility interface */

// 	// Dummy "application" overrides _p_start to control its own lifecycle/scheduling.
	virtual void start() override {
		RAC_ERRNO_MSG("gldemo at start of start function");
		threadloop::start();

		RAC_ERRNO_MSG("gldemo at end of start()");
	}
};

PLUGIN_MAIN(client_writer)
