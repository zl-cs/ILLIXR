#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include <opencv2/opencv.hpp>   // Include OpenCV API
#include <mutex>

// ILLIXR includes
#include "common/threadloop.hpp"
#include "common/switchboard.hpp"
#include "common/data_format.hpp"

using namespace ILLIXR;

static constexpr int IMAGE_WIDTH = 848;
static constexpr int IMAGE_HEIGHT = 800;

typedef struct {
	cv::Mat* img0;
	cv::Mat* img1;
	int iteration;
} cam_type;

typedef struct {
	rs2_vector* accel_data;
	int iteration;
} accel_type;

class realsense : public plugin {
public:
	realsense(std::string name_, phonebook *pb_)
        : plugin{name_, pb_}
        , sb{pb->lookup_impl<switchboard>()}
        , _m_imu_cam{sb->publish<imu_cam_type>("imu_cam")}
        , _m_imu_integrator{sb->publish<imu_integrator_seq>("imu_integrator_seq")}
        {
            cfg.disable_all_streams();
            cfg.enable_stream(RS2_STREAM_ACCEL);
            cfg.enable_stream(RS2_STREAM_GYRO);
            cfg.enable_stream(RS2_STREAM_FISHEYE, 1, RS2_FORMAT_Y8);
            cfg.enable_stream(RS2_STREAM_FISHEYE, 2, RS2_FORMAT_Y8);
            profiles = pipe.start(cfg, [&](const rs2::frame& frame) { this->callback(frame); });
        }

	void callback(const rs2::frame& frame)
        {
            std::lock_guard<std::mutex> lock(mutex);
            // This lock guarantees that concurrent invocations of `callback` are serialized.
            // Even if the API does not invoke `callback` in parallel, this is still important for the memory-model.
            // Without this lock, prior invocations of `callback` are not necessarily "happens-before" ordered, so accessing persistent variables constitutes a data-race, which is undefined behavior in the C++ memory model.

            if (auto fs = frame.as<rs2::frameset>()) {
                rs2::video_frame frame_left = fs.get_fisheye_frame(1);
                rs2::video_frame frame_right = fs.get_fisheye_frame(2);
                cv::Mat left = cv::Mat(cv::Size(IMAGE_WIDTH, IMAGE_HEIGHT), CV_8UC1, (void*)frame_left.get_data());
                cv::Mat right = cv::Mat(cv::Size(IMAGE_WIDTH, IMAGE_HEIGHT), CV_8UC1, (void *)frame_right.get_data());
                cam_type_ = cam_type{
                    new cv::Mat{left},
                    new cv::Mat{right},
                    iteration_cam,
                };
                iteration_cam++;
            }

            if (auto mf = frame.as<rs2::motion_frame>()) {
                std::string s = mf.get_profile().stream_name();

                if (s == "Accel")
                {
                    rs2::motion_frame accel = mf;
                    accel_data = accel.get_motion_data();
                    accel_type_.accel_data = &accel_data;
                    accel_type_.iteration = iteration_accel;
                    iteration_accel++;
                }

                if (s == "Gyro")
                {
                    if (last_iteration_accel == accel_type_.iteration) { return; }

                    last_iteration_accel = accel_type_.iteration;
                    rs2_vector accel = *accel_type_.accel_data;
                    rs2::motion_frame gyro = mf;
                    double ts = gyro.get_timestamp();
                    gyro_data = gyro.get_motion_data();

                    // IMU data
                    Eigen::Vector3f la = {accel.x, accel.y, accel.z};
                    Eigen::Vector3f av = {gyro_data.x, gyro_data.y, gyro_data.z};

                    // Eigen::Vector3f la = {accel.z, accel.x, -accel.y};
                    // Eigen::Vector3f av = {-gyro_data.y, gyro_data.z, -gyro_data.x};

                    // Time as ullong (nanoseconds)
                    ullong imu_time = static_cast<ullong>(ts * 1000000);

                    // Time as time_point
                    using time_point = std::chrono::system_clock::time_point;
                    time_type imu_time_point{std::chrono::duration_cast<time_point::duration>(std::chrono::nanoseconds(imu_time))};

                    // Images
                    std::optional<cv::Mat *> img0 = std::nullopt;
                    std::optional<cv::Mat *> img1 = std::nullopt;
                    if (last_iteration_cam != cam_type_.iteration)
                    {
                        last_iteration_cam = cam_type_.iteration;
                        img0 = cam_type_.img0;
                        img1 = cam_type_.img1;
                    }

                    // Submit to switchboard
                    _m_imu_cam->put(new imu_cam_type{
                            imu_time_point,
                            av,
                            la,
                            img0,
                            img1,
                            imu_time,
                        });

                    auto imu_integrator_params = new imu_integrator_seq{
                        .seq = static_cast<int>(++_imu_integrator_seq),
                    };
                    _m_imu_integrator->put(imu_integrator_params);
                }
            }
			
        };

	virtual ~realsense() override { pipe.stop(); }

private:
	const std::shared_ptr<switchboard> sb;
	std::unique_ptr<writer<imu_cam_type>> _m_imu_cam;
    std::unique_ptr<writer<imu_integrator_seq>> _m_imu_integrator;

	std::mutex mutex;
	rs2::pipeline_profile profiles;
	rs2::pipeline pipe;
	rs2::config cfg;
	rs2_vector gyro_data;
	rs2_vector accel_data;

	cam_type cam_type_;
	accel_type accel_type_;
	int iteration_cam = 0;
	int iteration_accel = 0;
	int last_iteration_cam;
	int last_iteration_accel;

    long long _imu_integrator_seq{0};
};

PLUGIN_MAIN(realsense);
