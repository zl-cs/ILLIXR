#include "common/threadloop.hpp"
#include "common/data_format.hpp"
#include "common/pose_prediction.hpp"
#include "common/phonebook.hpp"
#include "common/switchboard.hpp"
#include "common/utils/ansi_colors.hpp"
#include "../shared/packets.h"

#include <boost/asio.hpp>

using namespace ILLIXR;
using boost::asio::ip::tcp;

#define PREFIX BMAG << "[Rendering Offload Client] " << CRESET

class offload_rendering_client : public threadloop {
public:

    offload_rendering_client(std::string name_, phonebook* pb_)
        : threadloop{name_, pb_}
        , sb{pb->lookup_impl<switchboard>()}
        , pp{pb->lookup_impl<pose_prediction>()}
        , render_server_addr{std::getenv("RENDER_SERVER_ADDR")}
        , render_server_port{std::stoi(std::getenv("RENDER_SERVER_PORT"))} {
        // Connect to the render server
        connect();
    }

private:

    void _p_thread_setup() override {
        // Prevent io_context from exiting when there are no more work to do
        boost::asio::executor_work_guard<boost::asio::io_context::executor_type> work_guard {io_context.get_executor()};
        for (int i = 0; i < 2; i++) {
            boost::asio::post(pool, [this] {
                io_context.run();
            });
        }
        
        io_context.post([this] { read_packet(); });
    }

    void _p_one_iteration() override {
        if (write_in_progress) {
            return;
        } else {
            std::this_thread::sleep_for(std::chrono::milliseconds(500));

            write_in_progress = true;
            // sleep for 1 second
            io_context.post([this] { write_packet(); });
        }
    }

    void stop() override {
        io_context.stop();
        threadloop::stop();
    }

    void connect() {
        // All operations are blocking and will be executed on the main thread

        std::cout << "\n" << PREFIX << "Connecting to server " << render_server_addr << ":" << render_server_port << std::endl;

        // Res
        tcp::resolver resolver(io_context);
        tcp::resolver::results_type endpoints = resolver.resolve(render_server_addr, std::to_string(render_server_port));
        boost::asio::connect(socket, endpoints);
        
        std::cout << PREFIX << "Connected to server " << render_server_addr << ":" << render_server_port << std::endl;

        // Create streambuf for request
        std::array<char, 1> pong_buf;
        pong_buf[0] = PING;
        boost::asio::write(socket, boost::asio::buffer(pong_buf, 1));
        auto ping_time = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();

        std::cout << PREFIX << "Sent ping to server" << std::endl;

        // Receive pong from server
        auto response = read<char, 1>();
        assert(response == PONG);
        auto pong_time = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();

        // Print ping-pong time
        std::cout << PREFIX << "RTT: " << pong_time - ping_time << " ms" << std::endl;
    }

    void read_packet() {
        boost::asio::async_read(
            socket, 
            boost::asio::buffer(render_header_buf, 12), 
            boost::asio::transfer_exactly(12), 
            boost::asio::bind_executor(read_strand, [this](boost::system::error_code ec, std::size_t bytes_transferred) {
                if (ec) {
                    std::cout << PREFIX << "Error reading from socket: " << ec.message() << std::endl;
                    return;
                }

                // Read packet size
                ImageType type = (ImageType) render_header_buf[0];
                int size_left = *(int*) (render_header_buf.data() + 4);
                int size_right = *(int*) (render_header_buf.data() + 8);

                auto size = size_left + size_right;
                if (size > render_transfer_buf.size()) {
                    render_transfer_buf.resize(size);
                }

                boost::asio::read(socket, boost::asio::buffer(render_transfer_buf, size), boost::asio::transfer_exactly(size));

                read_packet();
            })
        );
    }

    void write_packet() {
        std::cout << PREFIX << "Sending pose to server" << std::endl;
        const pose_type fast_pose = pp->get_fast_pose().pose;

        // Create buffer and send pose to server
        boost::asio::async_write(
            socket, 
            boost::asio::buffer(&fast_pose, sizeof(fast_pose_type)), 
            boost::asio::transfer_exactly(sizeof(fast_pose_type)), 
            boost::asio::bind_executor(write_strand, [this](boost::system::error_code ec, std::size_t bytes_transferred) {
                if (ec) {
                    std::cout << PREFIX << "Error writing to socket: " << ec.message() << std::endl;
                    return;
                }

                write_in_progress = false;
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
    const std::shared_ptr<pose_prediction> pp;

    const std::string render_server_addr;
    const int render_server_port;

    boost::asio::thread_pool pool {2};
    boost::asio::io_context io_context;
    tcp::socket socket {io_context};

    boost::asio::io_context::strand read_strand {io_context};
    boost::asio::io_context::strand write_strand {io_context};

    // read_strand accessible
    std::array<char, 12> render_header_buf;
    std::vector<char> render_transfer_buf;

    // write_strand accessible
    std::atomic<bool> write_in_progress {false};
};

PLUGIN_MAIN(offload_rendering_client)
