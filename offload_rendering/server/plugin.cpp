#include "common/plugin.hpp"

#include "common/data_format.hpp"
#include "common/extended_window.hpp"
#include "common/phonebook.hpp"
#include "common/switchboard.hpp"
#include "common/utils/ansi_colors.hpp"
// Both X11 and opencv2 define the Complex type, so we need to undefine it
#undef Complex
#include "../shared/packets.h"
#include "video_encoder.h"

#include <boost/asio.hpp>
#include <boost/lockfree/queue.hpp>
#include <boost/thread.hpp>
#include <filesystem>
#include <opencv2/core/mat.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <vector>

using namespace ILLIXR;
using boost::asio::ip::tcp;

#define PREFIX BMAG << "[Rendering Offload Server] " << CRESET

class offload_rendering_server : public plugin {
public:
    offload_rendering_server(std::string name_, phonebook* pb_)
        : plugin{name_, pb_}
        , sb{pb->lookup_impl<switchboard>()}
        , xwin{pb->lookup_impl<xlib_gl_extended_window>()}
        , _m_fast_pose{sb->get_writer<fast_pose_type>("fast_pose")}
        , _m_rendered_frame{sb->get_writer<rendered_frame>("rendered_frame")}
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

        // set socket buffer size
        boost::asio::socket_base::send_buffer_size option(1024 * 1024 * 32);
        socket.set_option(option);

        // Print out client ip and port
        std::cout << PREFIX << "Client connected from " << socket.remote_endpoint().address().to_string() << ":"
                  << socket.remote_endpoint().port() << "." << std::endl;

        char packet_id = read<char, 1>();
        assert(packet_id == PING);
        std::cout << PREFIX << "Received ping from client." << std::endl;

        // Create buffer and send pong to client
        std::array<char, 1> pong_buf;
        pong_buf[0] = PONG;
        boost::asio::write(socket, boost::asio::buffer(pong_buf, 1));
    }

    std::shared_ptr<cv::Mat> gl_tex_to_cv_mat(GLuint ogl_texture_id) {
        glBindTexture(GL_TEXTURE_2D, ogl_texture_id);
        GLenum gl_texture_width, gl_texture_height;

        glGetTexLevelParameteriv(GL_TEXTURE_2D, 0, GL_TEXTURE_WIDTH, (GLint*) &gl_texture_width);
        glGetTexLevelParameteriv(GL_TEXTURE_2D, 0, GL_TEXTURE_HEIGHT, (GLint*) &gl_texture_height);

        unsigned char* gl_texture_bytes =
            (unsigned char*) malloc(sizeof(unsigned char) * gl_texture_width * gl_texture_height * 3);
        glGetTexImage(GL_TEXTURE_2D, 0 /* mipmap level */, GL_BGR, GL_UNSIGNED_BYTE, gl_texture_bytes);

        // create unique pointer for cv::Mat
        return std::make_shared<cv::Mat>(gl_texture_height, gl_texture_width, CV_8UC3, gl_texture_bytes);
    }

    void start() override {
        plugin::start();

        encoder = std::make_unique<video_encoder>(
            [this](const GstMapInfo& img0, const GstMapInfo& img1) {
                // we want to copy here since we don't wish to block the gst pipeline
                std::vector<char>               data0 = std::vector<char>(img0.data, img0.data + img0.size);
                std::vector<char>               data1 = std::vector<char>(img1.data, img1.data + img1.size);
                boost::lock_guard<boost::mutex> lock(frame_queue_mutex);
                // pull the frame from the queue
                auto frame = frame_queue.front();
                frame_queue.pop();

                boost::asio::post(write_strand,
                                  [this, data0 = std::move(data0), data1 = std::move(data1), frame = std::move(frame)] {
                                      write_frame(std::move(data0), std::move(data1), std::move(frame));
                                  });
            },
            xwin->dpy);
        encoder->init();

        // Start io_context worker threads
        boost::asio::executor_work_guard<boost::asio::io_context::executor_type> work_guard{io_context.get_executor()};
        for (int i = 0; i < 2; i++) {
            boost::asio::post(pool, [this] {
                io_context.run();
            });
        }
        read_packet();

        sb->schedule<rendered_frame>(id, "eyebuffer", [this](switchboard::ptr<const rendered_frame> datum, std::size_t) {
            // TODO: writing is originally blocked after receiving a frame from the gldemo. we're now blocking the pose from
            // being fed into gldemo if we're still writing the previous frame. Ideally we would need a swapchain

            // if (write_in_progress) {
            //     std::cout << PREFIX << "Rendered frame queueing up!" << std::endl;
            // }
            // while (write_in_progress) {
            //     std::this_thread::sleep_for(std::chrono::milliseconds(1));
            // }
            // write_in_progress = true;
            process_rendered_frame(std::move(datum));

            frame_count++;
            auto timestamp_ms =
                std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch())
                    .count();
            if (timestamp_ms - last_frame_count_time > 1000) {
                std::cout << PREFIX << frame_count << " fps" << std::endl;
                frame_count           = 0;
                last_frame_count_time = timestamp_ms;
            }
        });
    }

    void process_rendered_frame(switchboard::ptr<const rendered_frame> datum) {
        [[maybe_unused]] const bool gl_result = static_cast<bool>(glXMakeCurrent(xwin->dpy, xwin->win, xwin->glc));

        auto left_ptr  = datum->texture_handles[0];
        auto right_ptr = datum->texture_handles[1];

        rendered_frame_header header;
        header.type        = UNCOMPRESSED;
        header.pose        = pose_transfer::serialize(datum->render_pose.pose,
                                                      datum->render_pose.predict_computed_time.time_since_epoch().count(),
                                                      datum->render_pose.predict_target_time.time_since_epoch().count());
        header.render_time = datum->render_time.time_since_epoch().count();
        header.sample_time = datum->sample_time.time_since_epoch().count();

        std::shared_ptr<cv::Mat> left  = gl_tex_to_cv_mat(left_ptr);
        std::shared_ptr<cv::Mat> right = gl_tex_to_cv_mat(right_ptr);
    }

    void write_frame(const std::vector<char>&& left, const std::vector<char>&& right, const rendered_frame_header&& _header) {
        auto header = _header;
        header.size_left   = left.size();
        header.size_right  = right.size();

        std::shared_ptr<rendered_frame_header> header_ptr = std::make_shared<rendered_frame_header>(header);

        std::vector<boost::asio::const_buffer> buffers;
        buffers.push_back(boost::asio::buffer(header_ptr.get(), sizeof(rendered_frame_header)));
        buffers.push_back(boost::asio::buffer(left.data(), left.size()));
        buffers.push_back(boost::asio::buffer(right.data(), right.size()));

        write_begin =
            std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();

        // boost::asio::write(socket, buffers, boost::asio::transfer_exactly(sizeof(rendered_frame_header) + header.size_left +
        // header.size_right)); auto timestamp_ms =
        // std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
        // auto diff = timestamp_ms - write_begin;
        // std::cout << PREFIX << "Sent uncompressed frame to client in " << diff << " ms" << std::endl;
        // write_in_progress = false;

        // C++ standard guarantees that explicit captures won't be optimized out, so header_ptr will be valid
        boost::asio::async_write(
            socket, buffers,
            boost::asio::bind_executor(write_strand, [this, header_ptr](boost::system::error_code ec, std::size_t bytes_transferred) {
                if (ec) {
                    std::cout << PREFIX << "Error writing to socket: " << ec.message() << std::endl;
                    return;
                }

                // print time taken to send frame using write_begin
                auto timestamp_ms =
                    std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch())
                        .count();
                auto diff = timestamp_ms - write_begin;
                std::cout << PREFIX << "Sent uncompressed frame to client with size: " << bytes_transferred << " in " << diff
                          << " ms" << std::endl;

                // std::cout << PREFIX << "Sent uncompressed frame to client with size: " << bytes_transferred << std::endl;
                write_in_progress = false;
            }));
    }

    void read_packet() {
        boost::asio::async_read(
            socket, boost::asio::buffer(pose_buf, sizeof(pose_transfer)), boost::asio::transfer_exactly(sizeof(pose_transfer)),
            boost::asio::bind_executor(read_strand, [this](boost::system::error_code ec, std::size_t bytes_transferred) {
                if (ec) {
                    std::cout << PREFIX << "Error reading from socket: " << ec.message() << std::endl;
                    return;
                }

                if (write_in_progress) {
                    std::cout << PREFIX << "Dropping pose due to unfinished last frame" << std::endl;
                    read_packet();
                    return;
                }

                write_in_progress = true;

                // Parse pose
                pose_transfer pose;
                memcpy(&pose, pose_buf.data(), sizeof(pose_transfer));

                // Print pose
                // std::cout << PREFIX << "Received pose: " << pose.position[0] << ", " << pose.position[1] << ", " <<
                // pose.position[2] << std::endl;

                // Write pose to switchboard
                _m_fast_pose.put(_m_fast_pose.allocate<fast_pose_type>(fast_pose_type{
                    pose.deserialize(), time_point{std::chrono::duration<long, std::nano>(pose.predict_computed_time)},
                    time_point{std::chrono::duration<long, std::nano>(pose.predict_target_time)}}));

                read_packet();
            }));
    }

    template<typename T, unsigned int bytes>
    T read() {
        std::array<char, bytes> buf;
        boost::asio::read(socket, boost::asio::buffer(buf, bytes), boost::asio::transfer_exactly(bytes));
        return *(T*) buf.data();
    }

private:
    const std::shared_ptr<switchboard>                   sb;
    const std::shared_ptr<const xlib_gl_extended_window> xwin;
    const std::string                                    render_server_addr;
    const int                                            render_server_port;

    std::unique_ptr<video_encoder> encoder;

    boost::asio::thread_pool pool{2};
    boost::asio::io_context  io_context;
    tcp::socket              socket{io_context};

    boost::asio::io_context::strand read_strand{io_context};
    boost::asio::io_context::strand write_strand{io_context};

    // read strand accessible
    std::array<char, sizeof(pose_transfer)> pose_buf;
    switchboard::writer<fast_pose_type>     _m_fast_pose;
    switchboard::writer<rendered_frame>     _m_rendered_frame;

    // write strand accessible
    std::atomic<bool>          write_in_progress{false};
    int                        frame_count           = 0;
    int64_t                    last_frame_count_time = 0;
    int64_t                    write_begin           = 0;
    std::queue<rendered_frame_header> frame_queue;
    boost::mutex               frame_queue_mutex;
};

PLUGIN_MAIN(offload_rendering_server)
