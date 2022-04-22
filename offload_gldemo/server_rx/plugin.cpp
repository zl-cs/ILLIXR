#include "common/plugin.hpp"
#include "common/switchboard.hpp"
#include "common/data_format.hpp"
#include "common/phonebook.hpp"
#include "common/relative_clock.hpp"

#include <ecal/ecal.h>
#include <ecal/msg/protobuf/subscriber.h>

#include "gldemo_input.pb.h"

using namespace ILLIXR;

class server_reader : public plugin {
public:
	server_reader(std::string name_, phonebook* pb_)
		: plugin{name_, pb_}
		, sb{pb->lookup_impl<switchboard>()}
		, _m_clock{pb->lookup_impl<RelativeClock>()}
		, _m_pose{sb->get_writer<pose_type>("pose_to_gldemo")}
    { 
		eCAL::Initialize(0, NULL, "GLdemo Offloading Server-side Reader");
		subscriber = eCAL::protobuf::CSubscriber<gldemo_input_proto::Pose>("gldemo_input");
		subscriber.AddReceiveCallback(
		std::bind(&server_reader::ReceiveGLdemoInput, this, std::placeholders::_2));
	}

private:
    void ReceiveGLdemoInput(const gldemo_input_proto::Pose& gldemo_input) {
		std::cout << "Received one pose_to_gldemo\n";
		_m_pose.put(_m_pose.allocate<pose_type>(
			pose_type{
				_m_clock->now(), // FIXME not used
				Eigen::Vector3f{gldemo_input.position().x(), 
								gldemo_input.position().y(), 
								gldemo_input.position().z()},
				Eigen::Quaternionf{gldemo_input.orientation().w(), 
									gldemo_input.orientation().vec().x(), 
									gldemo_input.orientation().vec().y(), 
									gldemo_input.orientation().vec().z()}
			}
		));
	} 
    const std::shared_ptr<switchboard> sb;
	const std::shared_ptr<RelativeClock> _m_clock; 
	switchboard::writer<pose_type> _m_pose;
	eCAL::protobuf::CSubscriber<gldemo_input_proto::Pose> subscriber;
};
PLUGIN_MAIN(server_reader)