#include "common/threadloop.hpp"

#include "common/data_format.hpp"
#include "common/phonebook.hpp"
#include "common/switchboard.hpp"
#include "common/utils/ansi_colors.hpp"
#include "../shared/packets.h"

#include <boost/asio.hpp>

using namespace ILLIXR;
using boost::asio::ip::tcp;

#define PREFIX BMAG << "[Rendering Offload Server] " << CRESET

class offload_rendering_server : public threadloop {
public:

    offload_rendering_server(std::string name_, phonebook* pb_)
        : threadloop{name_, pb_}
        , sb{pb->lookup_impl<switchboard>()}
        , _m_fast_pose{sb->get_writer<pose_type>("fast_pose")}
        , render_server_addr{std::getenv("RENDER_SERVER_ADDR")}
        , render_server_port{std::stoi(std::getenv("RENDER_SERVER_PORT"))} {
            start_server();
    }

private:

    void start_server() {
        std::cout << PREFIX << "Starting server on " << render_server_addr << ":" << render_server_port << "." << std::endl;

        // Create acceptor on ip and port
        tcp::acceptor acceptor(io_context, tcp::endpoint(tcp::v4(), render_server_port));
        socket = acceptor.accept();

        // Print out client ip and port
        std::cout << PREFIX << "Client connected from " << socket.remote_endpoint().address().to_string() << ":" << socket.remote_endpoint().port() << "." << std::endl;

        char packet_id = read<char, 1>();
        assert(packet_id == PING);
        std::cout << PREFIX << "Received ping from client." << std::endl;

        // Create buffer and send pong to client
        std::array<char, 1> pong_buf;
        pong_buf[0] = PONG;
        boost::asio::write(socket, boost::asio::buffer(pong_buf, 1));
    }

    void _p_thread_setup() override {
        // Start io_context worker threads
        boost::asio::executor_work_guard<boost::asio::io_context::executor_type> work_guard {io_context.get_executor()};
        for (int i = 0; i < 2; i++) {
            boost::asio::post(pool, [this] {
                io_context.run();
            });
        }
        io_context.post([this] { read_packet(); });
    }

    void _p_one_iteration() override {
        
    }

    void read_packet() {
        boost::asio::async_read(
            socket,
            boost::asio::buffer(pose_buf, sizeof(pose_type)),
            boost::asio::transfer_exactly(sizeof(pose_type)),
            boost::asio::bind_executor(read_strand, [this](boost::system::error_code ec, std::size_t bytes_transferred) {
                if (ec) {
                    std::cout << PREFIX << "Error reading from socket: " << ec.message() << std::endl;
                    return;
                }

                // Parse pose
                pose_type pose = *(pose_type*)pose_buf.data();
                std::cout << PREFIX << "Received pose from client." << std::endl;

                // Write pose to switchboard
                _m_fast_pose.put(_m_fast_pose.allocate<pose_type>(std::move(pose)));

                read_packet();
            })
        );
    }

    template <typename T, unsigned int bytes> T read() {
        std::array<char, bytes> buf;
        boost::asio::read(socket, boost::asio::buffer(buf, bytes), boost::asio::transfer_exactly(bytes));
        return *(T*)buf.data();
    }

private:
    const std::shared_ptr<switchboard> sb;
    const std::string render_server_addr;
    const int render_server_port;

    boost::asio::thread_pool pool {2};
    boost::asio::io_context io_context;
    tcp::socket socket {io_context};

    boost::asio::io_context::strand read_strand {io_context};
    boost::asio::io_context::strand write_strand {io_context};

    // read strand accessible
    std::array<char, sizeof(pose_type)> pose_buf;
    switchboard::writer<pose_type> _m_fast_pose;
};

PLUGIN_MAIN(offload_rendering_server)
