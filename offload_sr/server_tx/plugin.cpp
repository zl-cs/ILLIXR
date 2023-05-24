#include "common/plugin.hpp"

#include "common/data_format.hpp"
#include "common/phonebook.hpp"
#include "common/switchboard.hpp"
#include "sr_input.pb.h"

#include <filesystem>
#include <fstream>

#include "common/network/socket.hpp"
#include "common/network/timestamp.hpp"
#include "common/network/net_config.hpp"

using namespace ILLIXR;

class server_writer : public plugin {
public:
    server_writer(std::string name_, phonebook* pb_)
		: plugin{name_, pb_}
		, sb{pb->lookup_impl<switchboard>()}
		, _m_generated_mesh{sb->get_reader<scene_recon_type>("ScanNet_Data")}
		, client_addr(CLIENT_IP, CLIENT_PORT_2)
    { 
		socket.set_reuseaddr();
		socket.bind(Address(SERVER_IP, SERVER_PORT_2));
		//is_client_connected = false;

		if (!filesystem::exists(data_path)) {
			if (!std::filesystem::create_directory(data_path)) {
				std::cerr << "Failed to create data directory.";
			}
		}

		//receiver_to_sender.open(data_path + "/receiver_to_sender_time.csv");
		// hashed.open(data_path + "/hash_server_tx.txt");

		// last_send_time = timestamp();
	}


    // This schedule function cant go in the constructor because there seems to be an issue with
    // the callbeing being triggered before any data is written to slow_pose. This needs debugging.
    virtual void start() override {
        plugin::start();
        sb->schedule<scene_recon_type>(id, "ScanNet_Data", [this](switchboard::ptr<const scene_recon_type> datum, std::size_t) {
			this->send_sr_output(datum);
		});
		sb->schedule<connection_signal>(id, "connection_signal", [this](switchboard::ptr<const connection_signal> datum, std::size_t) {
			this->start_accepting_connection(datum);
		});
	}

	void start_accepting_connection(switchboard::ptr<const connection_signal> datum) {
		socket.listen();
		cout << "server_tx: Waiting for connection!" << endl;
		write_socket = new TCPSocket( FileDescriptor( SystemCall( "accept", ::accept( socket.fd_num(), nullptr, nullptr) ) ) ); /* Blocking operation, waiting for client to connect */
		cout << "server_tx: Connection is established with " << write_socket->peer_address().str(":") << endl;
	}


    void send_sr_output(switchboard::ptr<const scene_recon_type> datum) {
		// Check if a socket connection has been established
		if (write_socket != NULL) {
			/* Logging */
			//unsigned long long curr_time = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
			//double sec_to_trans = (curr_time - datum->rec_time.time_since_epoch().count()) / 1e9;
			//receiver_to_sender << datum->frame_id << "," << datum->start_time.time_since_epoch().count() << "," << sec_to_trans * 1e3 << std::endl;

			// Construct mesh for output

			// This will get the time elapsed of the full roundtrip loop

			//unsigned long long end_pose_time = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
			//vio_output_params->set_end_server_timestamp(end_pose_time);

			// Prepare data delivery
			//string data_to_be_sent = vio_output_params->SerializeAsString();
			string delimitter = "END!";

			// long int now = timestamp();
			//write_socket->write(data_to_be_sent + delimitter);
			// long int send_duration = timestamp() - now;
			// std::cout << "send_duration: " << send_duration << std::endl;
			// last_send_time = now;

			// hash<std::string> hasher;
			// auto hash_result = hasher(data_to_be_sent);

			// hashed << datum->frame_id << "\t" << hash_result << endl;

			//delete vio_output_params;
		} else {
			cout << "server_tx ERROR: write_socket is not yet created!" << endl;
		}
    }

private:
    const std::shared_ptr<switchboard> sb;
	switchboard::reader<scene_recon_type> _m_generated_mesh;
	
	TCPSocket socket;
	TCPSocket * write_socket = NULL;
	Address client_addr;

	// long int last_send_time;
	const std::string data_path = filesystem::current_path().string() + "/recorded_data";
    //std::ofstream receiver_to_sender;
	// std::ofstream hashed;

};

PLUGIN_MAIN(server_writer)
