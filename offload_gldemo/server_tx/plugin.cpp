#include "common/plugin.hpp"
#include "common/switchboard.hpp"
#include "common/data_format.hpp"
#include "common/phonebook.hpp"

#include <ecal/ecal.h>
#include <ecal/msg/protobuf/publisher.h>

#include "gldemo_output.pb.h"

using namespace ILLIXR;

class server_writer : public plugin {
public:
    server_writer(std::string name_, phonebook* pb_)
		: plugin{name_, pb_}
		, sb{pb->lookup_impl<switchboard>()}
		, _m_frame{sb->get_reader<frame_from_gldemo>("frame")}
		, num{0}
    { 
		// Initialize eCAL and create a protobuf publisher
		eCAL::Initialize(0, NULL, "GLdemo Offloading Server-side Writer");
		publisher = eCAL::protobuf::CPublisher
		<gldemo_output_proto::count>("gldemo_output");
	}
    virtual void start() override {
        plugin::start();

        sb->schedule<frame_from_gldemo>(id, "frame", [this](switchboard::ptr<const frame_from_gldemo> datum, std::size_t) {
			this->send_gldemo_output(datum);
		});
	}

private:
    void send_gldemo_output(switchboard::ptr<const frame_from_gldemo> datum) {
		std::cout << "Received a frame\n"; 
		gldemo_output_proto::count* msg = new gldemo_output_proto::count();
		msg->set_num(num);
		publisher.Send(*msg);
		delete msg; 
	}; // TODO
    const std::shared_ptr<switchboard> sb;
	switchboard::reader<frame_from_gldemo> _m_frame;

	eCAL::protobuf::CPublisher<gldemo_output_proto::count> publisher;
	int num; 
};

PLUGIN_MAIN(server_writer)
