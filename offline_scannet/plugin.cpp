#include "common/data_format.hpp"
#include "common/global_module_defs.hpp"
#include "common/relative_clock.hpp"
#include "common/switchboard.hpp"
#include "common/threadloop.hpp"
#include "data_loading.hpp"

#include <cassert>
#include <ratio>

using namespace ILLIXR;

const record_header scannet_record{
    "scannet",
    {
        {"iteration_no", typeid(std::size_t)},
    },
};

class offline_scannet : public ILLIXR::threadloop {
public:
    offline_scannet(std::string name_, phonebook* pb_)
        : threadloop{name_, pb_}
        , _m_sensor_data{load_data()}
        , _m_sensor_data_it{_m_sensor_data.cbegin()}
        , _m_sb{pb->lookup_impl<switchboard>()}
        , _m_clock{pb->lookup_impl<RelativeClock>()} 
        //this is for RGBA
        , _m_scannet{_m_sb->get_writer<scene_recon_type>("ScanNet_Data")}
        , _m_scannet_rgb{_m_sb->get_writer<scene_recon_type>("ScanNet_RGB_Data")}
        , imu_cam_log{record_logger_}
        , camera_cvtfmt_log{record_logger_} { }

protected:
    virtual skip_option _p_should_skip() override {
        if (_m_sensor_data_it != _m_sensor_data.end()) {
            //pyh assuming its a perfect 30Hz 
            std::this_thread::sleep_for(std::chrono::duration<float, std::milli>(33.33));
            return skip_option::run;
        } else {
            return skip_option::stop;
        }
        //    return skip_option::run;
    }

    virtual void _p_one_iteration() override {
        assert(_m_sensor_data_it != _m_sensor_data.end());
        
        const sensor_types& sensor_datum = _m_sensor_data_it->second;
        ++_m_sensor_data_it;

        cv::Mat cam_depth = *(sensor_datum.depth_cam.load().release());

        if(cam_depth.empty())
        {
            printf("depth not loaded\n");
        }
        cv::Mat cam_color_rgb = *(sensor_datum.color_cam.load().release());
        //cv::Mat cam_color = *(sensor_datum.color_cam.color_load().release());
        
        cv::Mat cam_color(cam_color_rgb.size(), CV_8UC4, cv::Scalar(0,0,0,255));
        cv::cvtColor(cam_color_rgb, cam_color, cv::COLOR_BGR2RGBA,0);
        if(cam_color.empty())
        {
            printf("color not loaded\n");
        }
        
        //printf("received pose x: %f y: %f z: %f, x: %f y: %f, z: %f w:%f\n", sensor_datum.pose.position.x(), sensor_datum.pose.position.y(), sensor_datum.pose.position.z(), 
        //        sensor_datum.pose.orientation.x(), sensor_datum.pose.orientation.y(), sensor_datum.pose.orientation.z(), sensor_datum.pose.orientation.w() );
        pose_type pose = {time_point{}, sensor_datum.pose.position, sensor_datum.pose.orientation};
        //_m_scannet.put(_m_scannet.allocate<groundtruth_sceneRecon_type>(groundtruth_sceneRecon_type{time_point{}, {time_point{},sensor_datum.pose.position,sensor_datum.pose.orientation}, cam1, cam0}));
        _m_scannet.put(_m_scannet.allocate<scene_recon_type>(scene_recon_type{time_point{}, pose, cam_depth, cam_color,sensor_datum.last_frame}));
        _m_scannet_rgb.put(_m_scannet.allocate<scene_recon_type>(scene_recon_type{time_point{}, pose, cam_depth, cam_color_rgb,sensor_datum.last_frame}));
    }

private:
    const std::map<ullong, sensor_types>           _m_sensor_data;
    std::map<ullong, sensor_types>::const_iterator _m_sensor_data_it;
    const std::shared_ptr<switchboard>             _m_sb;
    std::shared_ptr<const RelativeClock>           _m_clock;
    switchboard::writer<scene_recon_type>              _m_scannet;
    switchboard::writer<scene_recon_type>              _m_scannet_rgb;


    record_coalescer imu_cam_log;
    record_coalescer camera_cvtfmt_log;
};

PLUGIN_MAIN(offline_scannet)
