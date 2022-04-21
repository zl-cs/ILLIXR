#include "common/plugin.hpp"
#include "common/switchboard.hpp"
#include "common/data_format.hpp"
#include "common/phonebook.hpp"

// #include <ecal/ecal.h>
// #include <ecal/msg/protobuf/publisher.h>

// #include "gldemo_output.pb.h"

using namespace ILLIXR;

class server_writer : public plugin {
public:
    server_writer(std::string name_, phonebook* pb_)
		: plugin{name_, pb_}
		, sb{pb->lookup_impl<switchboard>()}
		, _m_eyebuffer{sb->get_reader<rendered_frame>("eyebuffer")}
    { 
		// Initialize eCAL and create a protobuf publisher
		eCAL::Initialize(0, NULL, "GLdemo Offloading Frame Writer");
		publisher = eCAL::protobuf::CPublisher
		<vio_output_proto::VIOOutput>("gldemo_output");
	}
    virtual void start() override {
        plugin::start();

        sb->schedule<rendered_frame>(id, "eyebuffer", [this](switchboard::ptr<const rendered_frame> datum, std::size_t) {
			this->send_gldemo_output(datum);
		});
	}

private:
    void send_gldemo_output(switchboard::ptr<const rendered_frame> datum) {}; // TODO
    const std::shared_ptr<switchboard> sb;
	switchboard::reader<rendered_frame> _m_eyebuffer;
};

PLUGIN_MAIN(server_writer)
