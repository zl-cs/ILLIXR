#include "common/plugin.hpp"
#include "common/switchboard.hpp"
#include "common/data_format.hpp"
#include "common/phonebook.hpp"

#include <ecal/ecal.h>
#include <ecal/msg/protobuf/subscriber.h>

#include "gldemo_output.pb.h"

using namespace ILLIXR;

class client_reader : public plugin {
public:
    client_reader(std::string name_, phonebook* pb_)
		: plugin{name_, pb_}
		, sb{pb->lookup_impl<switchboard>()}
		, _m_clock{pb->lookup_impl<RelativeClock>()}
		, _m_eyebuffer{sb->get_writer<rendered_frame>("eyebuffer")}
		, _m_fast_pose{sb->get_buffered_reader<fast_pose_type>("fast_pose")}
    { 

		eCAL::Initialize(0, NULL, "GLdemo Offloading Frame Reader");
		subscriber = eCAL::protobuf::CSubscriber<gldemo_output_proto::Rendered_frame>("gldemo_output");
		subscriber.AddReceiveCallback(
		std::bind(&client_reader::ReceiveGLdemoOutput, this, std::placeholders::_2));
	}

private:
    void ReceiveGLdemoOutput(const gldemo_output_proto::Rendered_frame& frame) {
		std::cout << "Received one rendered_frame\n";
		auto fast_pose = _m_fast_pose.dequeue(); 
		if (fast_pose == nullptr) return;
		_m_eyebuffer.put(_m_eyebuffer.allocate<rendered_frame>(
			rendered_frame{
                std::array<GLuint, 2>{ frame.th().left() , frame.th().right() },
                std::array<GLuint, 2>{ frame.si().front(), frame.si().back() },
                *fast_pose,
                fast_pose->predict_computed_time,
                _m_clock->now()
			}));
	}; // TODO 
    const std::shared_ptr<switchboard> sb; 
	const std::shared_ptr<const RelativeClock> _m_clock;
    switchboard::writer<rendered_frame> _m_eyebuffer; 
	switchboard::buffered_reader<fast_pose_type> _m_fast_pose; 

	eCAL::protobuf::CSubscriber<gldemo_output_proto::Rendered_frame> subscriber;

};
PLUGIN_MAIN(client_reader)