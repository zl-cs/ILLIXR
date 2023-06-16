#include "common/threadloop.hpp"
#include "common/switchboard.hpp"
#include "common/data_format.hpp"
#include "common/phonebook.hpp"
#include "common/switchboard.hpp"
#include "sr_output.pb.h"

#include <filesystem>
#include <fstream>

#include "common/network/socket.hpp"
#include "common/network/net_config.hpp"

using namespace ILLIXR;

class offload_reader : public threadloop {
public:
    offload_reader(std::string name_, phonebook* pb_)
		: threadloop{name_, pb_}
		, sb{pb->lookup_impl<switchboard>()}
		, _m_clock{pb->lookup_impl<RelativeClock>()}
		, _m_mesh{sb->get_writer<mesh_type>("compressed_scene")}
		, server_addr(SERVER_IP, SERVER_PORT_2)
    { 
		if (!filesystem::exists(data_path)) {
			if (!filesystem::create_directory(data_path)) {
				std::cerr << "Failed to create data directory.";
			}
		}
		socket.set_reuseaddr();
		socket.bind(Address(CLIENT_IP, CLIENT_PORT_2));
		is_socket_connected = false;
        payload_count=0;
	}

	virtual skip_option _p_should_skip() override {
        if (!is_socket_connected) {
			cout << "device_rx: Connecting to " << server_addr.str(":") << endl;
			socket.connect(server_addr);
			cout << "device_rx: Connected to " << server_addr.str(":") << endl;
			is_socket_connected = true;
		}
		return skip_option::run;
    }

	void _p_one_iteration() override {
		if (is_socket_connected) {
			auto now = timestamp();
			string delimitter = "END!";
			string recv_data = socket.read(); /* Blocking operation, wait for the data to come */
			if (recv_data.size() > 0) {
				buffer_str = buffer_str + recv_data;
				string::size_type end_position = buffer_str.find(delimitter);
				while (end_position != string::npos) {
					string before = buffer_str.substr(0, end_position);
					buffer_str = buffer_str.substr(end_position + delimitter.size());
			
					// process the data
					sr_output_proto::CompressMeshData sr_output;
					bool success = sr_output.ParseFromString(before);
					if (success) {
						// cout << "Received vio output (" << datagram.size() << " bytes) from " << client_addr.str(":") << endl;
						ReceiveSROutput(sr_output);
					} else {
						cout << "client_rx: Cannot parse VIO output!!" << endl;
					}
					end_position = buffer_str.find(delimitter);
				}
				// cout << "Recv time = " << timestamp() - now << ", size = " << recv_data.size() << endl;
			}
		}
	}

private:
	void ReceiveSROutput(const sr_output_proto::CompressMeshData& sr_output) {
        std::cout<<"received a server payload: "<<payload_count<<std::endl;        
        const std::string& dataString = sr_output.draco_data();
        std::vector<char> dataVector(dataString.begin(), dataString.end());
        _m_mesh.put(_m_mesh.allocate<mesh_type>(mesh_type{dataVector,true, payload_count}));
        payload_count++;
    }

    const std::shared_ptr<switchboard> sb;
	const std::shared_ptr<RelativeClock> _m_clock;
     switchboard::writer<mesh_type> _m_mesh;


	TCPSocket socket;
	bool is_socket_connected;
	Address server_addr;
	string buffer_str;
    unsigned payload_count;
	const string data_path = filesystem::current_path().string() + "/recorded_data";
	std::ofstream pose_transfer_csv;
	std::ofstream roundtrip_csv;
};

PLUGIN_MAIN(offload_reader)
