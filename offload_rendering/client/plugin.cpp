#include <boost/asio.hpp>
#include <boost/thread.hpp>
#include <boost/thread/synchronized_value.hpp>
#include <queue>

#include "common/data_format.hpp"
#include "common/extended_window.hpp"
#include "common/phonebook.hpp"
#include "common/pose_prediction.hpp"
#include "common/switchboard.hpp"
#include "common/threadloop.hpp"
#include "common/utils/ansi_colors.hpp"

// Both X11 and opencv2 define the Complex type, so we need to undefine it
#undef Complex

#include "../shared/packets.h"

#include <opencv2/core/mat.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>

using namespace ILLIXR;
using boost::asio::ip::tcp;

#define PREFIX BMAG << "[Rendering Offload Client] " << CRESET

// Wake up 1 ms after vsync instead of exactly at vsync to account for scheduling uncertainty
static constexpr std::chrono::milliseconds VSYNC_SAFETY_DELAY{1};

class offload_rendering_client : public threadloop {
public:
    offload_rendering_client(std::string name_, phonebook* pb_)
        : threadloop{name_, pb_}
        , sb{pb->lookup_impl<switchboard>()}
        , pp{pb->lookup_impl<pose_prediction>()}
        , xwin{new xlib_gl_extended_window{1, 1, pb->lookup_impl<xlib_gl_extended_window>()->glc}}
        , _m_clock{pb->lookup_impl<RelativeClock>()}
        , _m_eyebuffer{sb->get_writer<rendered_frame>("eyebuffer")}
        , _m_vsync{sb->get_reader<switchboard::event_wrapper<time_point>>("vsync_estimate")}
        , render_server_addr{std::getenv("RENDER_SERVER_ADDR")}
        , render_server_port{std::stoi(std::getenv("RENDER_SERVER_PORT"))} {
        // Connect to the render server
        connect();
    }

private:
    // Essentially, a crude equivalent of XRWaitFrame.
    void wait_vsync() {
        switchboard::ptr<const switchboard::event_wrapper<time_point>> next_vsync = _m_vsync.get_ro_nullable();
        time_point                                                     now        = _m_clock->now();
        time_point                                                     wait_time;

        if (next_vsync == nullptr) {
            // If no vsync data available, just sleep for roughly a vsync period.
            // We'll get synced back up later.
            std::cout << PREFIX << "No Vsync Data Available" << std::endl;
            std::this_thread::sleep_for(display_params::period);
            return;
        }

        bool hasRenderedThisInterval = (now - lastTime) < display_params::period;

        // If less than one frame interval has passed since we last rendered...
        // if (hasRenderedThisInterval) {
            // We'll wait until the next vsync, plus a small delay time.
            // Delay time helps with some inaccuracies in scheduling.
            wait_time = **next_vsync + VSYNC_SAFETY_DELAY;

            // If our sleep target is in the past, bump it forward
            // by a vsync period, so it's always in the future.
            while (wait_time < now) {
                wait_time += display_params::period;
            }
            // Perform the sleep.
            // TODO: Consider using Monado-style sleeping, where we nanosleep for
            // most of the wait, and then spin-wait for the rest?
            std::this_thread::sleep_for(wait_time - now);
        // }
    }

    void _p_thread_setup() override {
        lastTime = _m_clock->now();

        [[maybe_unused]] const bool gl_result = static_cast<bool>(glXMakeCurrent(xwin->dpy, xwin->win, xwin->glc));

        // Create shared memory for eye textures
        createSharedEyebuffer(&(eyeTextures[0]));
        createSharedEyebuffer(&(eyeTextures[1]));

        // Prevent io_context from exiting when there are no more work to do
        boost::asio::executor_work_guard<boost::asio::io_context::executor_type> work_guard{io_context.get_executor()};
        for (int i = 0; i < 2; i++) {
            boost::asio::post(pool, [this] {
                io_context.run();
            });
        }

        // post read to read_strand
        boost::asio::post(read_strand, [this] {
            read_packet();
        });

        // post write to write_strand
        boost::asio::post(write_strand, [this] {
            write_packet();
        });
    }

    void _p_one_iteration() override {
        wait_vsync();

        // current time
        long now = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();

        // fps counter
        if (now - last_fps_time > 1000) {
            std::cout << PREFIX << "FPS: " << fps_counter << std::endl;
            fps_counter = 0;
            last_fps_time = now;
        } else {
            fps_counter++;
        }

        {
            boost::lock_guard<boost::mutex> lock{render_mutex};

            // skip if no frame to render
            if (!render_header_queue.empty()) {
                auto header = render_header_queue.front();
                render_header_queue.pop();
                auto left_data = render_frame_left_queue.front();
                render_frame_left_queue.pop();
                auto right_data = render_frame_right_queue.front();
                render_frame_right_queue.pop();

                // cv::Mat left{header.rows, header.cols, CV_8UC3, left_data.data()};
                // cv::Mat right{header.rows, header.cols, CV_8UC3, right_data.data()};

                process_uncompressed_frame(header, left_data, right_data);
            }
        }
    }

    // skip_option _p_should_skip() override {
    //     auto timestamp =
    //     std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
    //     if (timestamp - last_pose_time > (1000.0 / 60.0)) {
    //         last_pose_time = timestamp;
    //         return skip_option::run;
    //     }
    //     return skip_option::skip_and_yield;
    // }

    void stop() override {
        io_context.stop();
        threadloop::stop();
    }

    void connect() {
        // All operations are blocking and will be executed on the main thread

        std::cout << "\n" << PREFIX << "Connecting to server " << render_server_addr << ":" << render_server_port << std::endl;

        // Res
        tcp::resolver               resolver(io_context);
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
        auto ping_time =
            std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();

        std::cout << PREFIX << "Sent ping to server" << std::endl;

        // Receive pong from server
        auto response = read<char, 1>();
        assert(response == PONG);
        auto pong_time =
            std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();

        // Print ping-pong time
        std::cout << PREFIX << "RTT: " << pong_time - ping_time << " ms" << std::endl;
    }

    void read_packet() {
        boost::asio::async_read(
            socket, boost::asio::buffer(render_header_buf, sizeof(rendered_frame_header)),
            boost::asio::transfer_exactly(sizeof(rendered_frame_header)),
            boost::asio::bind_executor(read_strand, [this](boost::system::error_code ec, std::size_t bytes_transferred) {
                if (ec) {
                    std::cout << PREFIX << "Error reading from socket: " << ec.message() << std::endl;
                    return;
                }

                rendered_frame_header header;
                memcpy(&header, render_header_buf.data(), sizeof(rendered_frame_header));

                // print size_left and size_right
                // std::cout << PREFIX << "Received frame with size_left : " << header.size_left
                //           << ", size_right: " << header.size_right << std::endl;

                auto size = header.size_left + header.size_right;
                if (size > render_transfer_buf.size()) {
                    render_transfer_buf.resize(size);
                }

                boost::asio::read(socket, boost::asio::buffer(render_transfer_buf, size), boost::asio::transfer_exactly(size));

                std::vector<char> left_data(render_transfer_buf.begin(), render_transfer_buf.begin() + header.size_left);
                std::vector<char> right_data(render_transfer_buf.begin() + header.size_left,
                                             render_transfer_buf.begin() + header.size_left + header.size_right);

                {
                    boost::lock_guard<boost::mutex> lock{render_mutex};
                    this->render_header_queue.push(header);
                    this->render_frame_left_queue.push(std::move(left_data));
                    this->render_frame_right_queue.push(std::move(right_data));
                }

                read_packet();
            }));
    }

    void process_uncompressed_frame(rendered_frame_header& header, std::vector<char> left, std::vector<char> right) {
        [[maybe_unused]] const bool gl_result = static_cast<bool>(glXMakeCurrent(xwin->dpy, xwin->win, xwin->glc));
        // Create cv::Mat from render_transfer_buf
        copyToSharedEyebuffer(&(eyeTextures[0]), left);
        copyToSharedEyebuffer(&(eyeTextures[1]), right);
        glFinish();

        // imshow
        // cv::imshow("left", left);
        // cv::waitKey(1);

        // sleep 2ms
        // std::this_thread::sleep_for(std::chrono::milliseconds(2));

        fast_pose_type fp = fast_pose_type{
            header.pose.pose,
            time_point{std::chrono::duration<long, std::nano>(header.pose.predict_computed_time)},
            time_point{std::chrono::duration<long, std::nano>(header.pose.predict_target_time)},
        };

        lastTime = _m_clock->now();

        _m_eyebuffer.put(_m_eyebuffer.allocate<rendered_frame>(rendered_frame{
            // Somehow, C++ won't let me construct this object if I remove the `rendered_frame{` and `}`.
            // `allocate<rendered_frame>(...)` _should_ forward the arguments to rendered_frame's constructor, but I guess
            // not.
            std::array<GLuint, 2>{eyeTextures[0], eyeTextures[1]}, std::array<GLuint, 2>{0, 0}, fp, fp.predict_computed_time,
            lastTime}));
    }

    void write_packet() {
        auto fp = pp->get_fast_pose();
        // std::cout << PREFIX << "Sending pose to server" << std::endl;
        pose_transfer_packet = pose_transfer{
            .pose                  = fp.pose,
            .predict_computed_time = fp.predict_computed_time.time_since_epoch().count(),
            .predict_target_time   = fp.predict_target_time.time_since_epoch().count(),
        };

        // Print pose
        // std::cout << PREFIX << "Position: " << fast_pose.position[0] << ", " << fast_pose.position[1] << ", " <<
        // fast_pose.position[2] << std::endl;

        // Create buffer and send pose to server
        boost::asio::async_write(
            socket, boost::asio::buffer(&pose_transfer_packet, sizeof(pose_transfer)),
            boost::asio::transfer_exactly(sizeof(pose_transfer)),
            boost::asio::bind_executor(write_strand, [this](boost::system::error_code ec, std::size_t bytes_transferred) {
                if (ec) {
                    std::cout << PREFIX << "Error writing to socket: " << ec.message() << std::endl;
                    return;
                }

            auto timestamp =
            std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
            while (timestamp - last_pose_time < (1000.0 / 120.0)) {
                timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(
                                std::chrono::system_clock::now().time_since_epoch())
                                .count();
            }
            last_pose_time = timestamp;
            write_packet();

            // write_in_progress = false;
            }));
    }

    template<typename T, unsigned int bytes>
    T read() {
        std::array<char, bytes> buf;
        boost::asio::read(socket, boost::asio::buffer(buf, bytes), boost::asio::transfer_exactly(bytes));
        return *(T*) buf.data();
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

    void copyToSharedEyebuffer(GLuint* texture_handle, std::vector<char>& image) {
        // Copy the image to the shared texture
        glBindTexture(GL_TEXTURE_2D, *texture_handle);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB8, display_params::width_pixels, display_params::height_pixels, 0, GL_BGR,
                     GL_UNSIGNED_BYTE, image.data());
        glBindTexture(GL_TEXTURE_2D, 0);
    }

private:
    const std::shared_ptr<switchboard>                                sb;
    const std::shared_ptr<pose_prediction>                            pp;
    const std::shared_ptr<xlib_gl_extended_window>                    xwin;
    const std::shared_ptr<const RelativeClock>                        _m_clock;
    const switchboard::reader<switchboard::event_wrapper<time_point>> _m_vsync;
    switchboard::writer<rendered_frame>                               _m_eyebuffer;
    time_point                                                        lastTime;

    GLuint eyeTextures[2];

    const std::string render_server_addr;
    const int         render_server_port;

    boost::asio::thread_pool pool{2};
    boost::asio::io_context  io_context;
    tcp::socket              socket{io_context};

    boost::asio::io_context::strand read_strand{io_context};
    boost::asio::io_context::strand write_strand{io_context};

    int64_t last_pose_time = 0;

    // read_strand accessible
    std::array<char, sizeof(rendered_frame_header)> render_header_buf;
    std::vector<char>                               render_transfer_buf;

    // write_strand accessible
    pose_transfer     pose_transfer_packet;
    // std::atomic<bool> write_in_progress{false};

    // shared by asio and main thread
    boost::mutex                      render_mutex;
    std::queue<rendered_frame_header> render_header_queue;
    std::queue<std::vector<char>>               render_frame_left_queue;
    std::queue<std::vector<char>>               render_frame_right_queue;

    // main thread only
    int fps_counter = 0;
    long last_fps_time = 0;
};

PLUGIN_MAIN(offload_rendering_client)
