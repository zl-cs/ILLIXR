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
        , _m_scannet{_m_sb->get_writer<scene_recon_type>("ScanNet_Data")}
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
        cv::Mat cam_color = *(sensor_datum.color_cam.color_load().release());
        if(cam_color.empty())
        {
            printf("color not loaded\n");
        }
        
        pose_type pose = {time_point{}, sensor_datum.pose.position, sensor_datum.pose.orientation};
        //_m_scannet.put(_m_scannet.allocate<groundtruth_sceneRecon_type>(groundtruth_sceneRecon_type{time_point{}, {time_point{},sensor_datum.pose.position,sensor_datum.pose.orientation}, cam1, cam0}));
        _m_scannet.put(_m_scannet.allocate<scene_recon_type>(scene_recon_type{time_point{}, pose, cam_depth, cam_color,sensor_datum.last_frame}));
    }

private:
    const std::map<ullong, sensor_types>           _m_sensor_data;
    std::map<ullong, sensor_types>::const_iterator _m_sensor_data_it;
    const std::shared_ptr<switchboard>             _m_sb;
    std::shared_ptr<const RelativeClock>           _m_clock;
    switchboard::writer<scene_recon_type>              _m_scannet;


    record_coalescer imu_cam_log;
    record_coalescer camera_cvtfmt_log;
};

PLUGIN_MAIN(offline_scannet)
