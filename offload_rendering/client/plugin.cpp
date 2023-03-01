#include "common/threadloop.hpp"
#include "common/data_format.hpp"
#include "common/pose_prediction.hpp"
#include "common/phonebook.hpp"
#include "common/switchboard.hpp"
#include "common/utils/ansi_colors.hpp"
#include "../shared/packets.h"

#include <boost/asio.hpp>
#include "common/extended_window.hpp"
// Both X11 and opencv2 define the Complex type, so we need to undefine it
#undef Complex
#include <opencv2/core/mat.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>

using namespace ILLIXR;
using boost::asio::ip::tcp;

#define PREFIX BMAG << "[Rendering Offload Client] " << CRESET

class offload_rendering_client : public threadloop {
public:

    offload_rendering_client(std::string name_, phonebook* pb_)
        : threadloop{name_, pb_}
        , sb{pb->lookup_impl<switchboard>()}
        , pp{pb->lookup_impl<pose_prediction>()}
        , xwin{new xlib_gl_extended_window{1, 1, pb->lookup_impl<xlib_gl_extended_window>()->glc}}
        , _m_clock{pb->lookup_impl<RelativeClock>()}
        , _m_eyebuffer{sb->get_writer<rendered_frame>("eyebuffer")}
        , render_server_addr{std::getenv("RENDER_SERVER_ADDR")}
        , render_server_port{std::stoi(std::getenv("RENDER_SERVER_PORT"))} {
        // Connect to the render server
        connect();
    }

private:

    void _p_thread_setup() override {
        // Create shared memory for eye textures
        createSharedEyebuffer(&(eyeTextures[0]));
        createSharedEyebuffer(&(eyeTextures[1]));

        // Prevent io_context from exiting when there are no more work to do
        boost::asio::executor_work_guard<boost::asio::io_context::executor_type> work_guard {io_context.get_executor()};
        for (int i = 0; i < 2; i++) {
            boost::asio::post(pool, [this] {
                io_context.run();
            });
        }
        
        read_packet();
    }

    void _p_one_iteration() override {
        if (write_in_progress) {
            std::cout << PREFIX << "OUTBOUND POSE QUEUEING UP" << std::endl;
            return;
        } else {
            write_in_progress = true;
            write_packet();
        }
    }

    skip_option _p_should_skip() override {
        auto timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
        if (timestamp - last_pose_time > (1000.0 / 60.0)) {
            last_pose_time = timestamp;
            return skip_option::run;
        }
        return skip_option::skip_and_yield;
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
        // set socket buffer size
        boost::asio::socket_base::receive_buffer_size option(1024 * 1024 * 32);
        socket.set_option(option);
        
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
            boost::asio::buffer(render_header_buf, sizeof(rendered_frame_header)), 
            boost::asio::transfer_exactly(sizeof(rendered_frame_header)), 
            boost::asio::bind_executor(read_strand, [this](boost::system::error_code ec, std::size_t bytes_transferred) {
                if (ec) {
                    std::cout << PREFIX << "Error reading from socket: " << ec.message() << std::endl;
                    return;
                }

                // Decode struct rendered_frame_header
                rendered_frame_header header;
                memcpy(&header, render_header_buf.data(), sizeof(rendered_frame_header));
                // print size_left and size_right
                std::cout << PREFIX << "Received frame with size_left: " << header.size_left << ", size_right: " << header.size_right << std::endl;

                auto size = header.size_left + header.size_right;
                if (size > render_transfer_buf.size()) {
                    render_transfer_buf.resize(size);
                }

                boost::asio::read(socket, boost::asio::buffer(render_transfer_buf, size), boost::asio::transfer_exactly(size));

                // Process the frame
                process_uncompressed_frame(header);

                read_packet();
            })
        );
    }

    void process_uncompressed_frame(rendered_frame_header& header) {
        [[maybe_unused]] const bool gl_result = static_cast<bool>(glXMakeCurrent(xwin->dpy, xwin->win, xwin->glc));
        // Create cv::Mat from render_transfer_buf
        cv::Mat left {header.rows, header.cols, CV_8UC3, render_transfer_buf.data()};
        cv::Mat right {header.rows, header.cols, CV_8UC3, render_transfer_buf.data() + header.size_left};
        copyToSharedEyebuffer(&(eyeTextures[0]), left);
        copyToSharedEyebuffer(&(eyeTextures[1]), right);

        // imshow
        // cv::imshow("left", left);
        // cv::waitKey(1);

        fast_pose_type fp = fast_pose_type {
                header.pose.pose,
                time_point {std::chrono::duration<long, std::nano>(header.pose.predict_computed_time)},
                time_point {std::chrono::duration<long, std::nano>(header.pose.predict_target_time)},
            };

        _m_eyebuffer.put(_m_eyebuffer.allocate<rendered_frame>(rendered_frame{
            // Somehow, C++ won't let me construct this object if I remove the `rendered_frame{` and `}`.
            // `allocate<rendered_frame>(...)` _should_ forward the arguments to rendered_frame's constructor, but I guess
            // not.
            std::array<GLuint, 2>{eyeTextures[0], eyeTextures[1]}, std::array<GLuint, 2>{0, 0}, fp,
            fp.predict_computed_time, _m_clock->now()}));

        std::cout << PREFIX << "Processed frame" << std::endl;

    }

    void write_packet() {
        auto fp = pp->get_fast_pose();
        // std::cout << PREFIX << "Sending pose to server" << std::endl;
        pose_transfer_packet = pose_transfer {
            .pose = fp.pose,
            .predict_computed_time = fp.predict_computed_time.time_since_epoch().count(),
            .predict_target_time = fp.predict_target_time.time_since_epoch().count(),
        };

        // Print pose
        // std::cout << PREFIX << "Position: " << fast_pose.position[0] << ", " << fast_pose.position[1] << ", " << fast_pose.position[2] << std::endl;

        // Create buffer and send pose to server 
        boost::asio::async_write(
            socket, 
            boost::asio::buffer(&pose_transfer_packet, sizeof(pose_transfer)), 
            boost::asio::transfer_exactly(sizeof(pose_transfer)), 
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

    void createSharedEyebuffer(GLuint* texture_handle) {
        // Create the shared eye texture handle
        glGenTextures(1, texture_handle);
        glBindTexture(GL_TEXTURE_2D, *texture_handle);

        // Set the texture parameters for the texture that the FBO will be mapped into
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAX_LEVEL, 0);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_BORDER);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_BORDER);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB8, display_params::width_pixels, display_params::height_pixels, 0, GL_RGB,
                     GL_UNSIGNED_BYTE, 0);

        // Unbind texture
        glBindTexture(GL_TEXTURE_2D, 0);
    }

    void copyToSharedEyebuffer(GLuint* texture_handle, cv::Mat& image) {
        // Copy the image to the shared texture
        glBindTexture(GL_TEXTURE_2D, *texture_handle);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB8, display_params::width_pixels, display_params::height_pixels, 0, GL_RGB,
                     GL_UNSIGNED_BYTE, image.data);
        glBindTexture(GL_TEXTURE_2D, 0);
    }

private:
    const std::shared_ptr<switchboard> sb;
    const std::shared_ptr<pose_prediction> pp;
    const std::shared_ptr<xlib_gl_extended_window> xwin;
    const std::shared_ptr<const RelativeClock> _m_clock;
    switchboard::writer<rendered_frame> _m_eyebuffer;

    GLuint eyeTextures[2];

    const std::string render_server_addr;
    const int render_server_port;

    boost::asio::thread_pool pool {2};
    boost::asio::io_context io_context;
    tcp::socket socket {io_context};

    boost::asio::io_context::strand read_strand {io_context};
    boost::asio::io_context::strand write_strand {io_context};

    int64_t last_pose_time = 0;

    // read_strand accessible
    std::array<char, sizeof(rendered_frame_header)> render_header_buf;
    std::vector<char> render_transfer_buf;

    // write_strand accessible
    pose_transfer pose_transfer_packet;
    std::atomic<bool> write_in_progress {false};
};

PLUGIN_MAIN(offload_rendering_client)
