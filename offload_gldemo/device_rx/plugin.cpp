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
		, _m_eyebuffer{sb->get_writer<rendered_frame>("eyebuffer")}
    { 

		eCAL::Initialize(0, NULL, "GLdemo Offloading Frame Reader");
		subscriber = eCAL::protobuf::CSubscriber
		<gldemo_output_proto::GLdemoOutput>("gldemo_output");
		subscriber.AddReceiveCallback(
		std::bind(&client_reader::ReceiveGLdemoOutput, this, std::placeholders::_2));
	}

private:
    void ReceiveGLdemoOutput() {}; // TODO 
    const std::shared_ptr<switchboard> sb; 
    switchboard::writer<rendered_frame> _m_eyebuffer; 

};
PLUGIN_MAIN(client_reader)