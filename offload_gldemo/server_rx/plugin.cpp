#include "common/plugin.hpp"
#include "common/switchboard.hpp"
#include "common/data_format.hpp"
#include "common/phonebook.hpp"

#include <ecal/ecal.h>
#include <ecal/msg/protobuf/subscriber.h>

#include "gldemo_input.pb.h"

using namespace ILLIXR;

class server_reader : public plugin {
public:
	server_reader(std::string name_, phonebook* pb_)
		: plugin{name_, pb_}
		, sb{pb->lookup_impl<switchboard>()}
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
	} // TODO 
    const std::shared_ptr<switchboard> sb;
	switchboard::writer<pose_type> _m_pose;
	eCAL::protobuf::CSubscriber<gldemo_input_proto::Pose> subscriber;
};
PLUGIN_MAIN(server_reader)