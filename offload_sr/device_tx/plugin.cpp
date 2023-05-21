#include "common/plugin.hpp"

#include "common/data_format.hpp"
#include "common/phonebook.hpp"
#include "common/switchboard.hpp"
#include "sr_input.pb.h"

#include <ecal/ecal.h>
#include <ecal/msg/protobuf/publisher.h>
#include <opencv/cv.hpp>

using namespace ILLIXR;

class offload_writer : public plugin {
public:
    offload_writer(std::string name_, phonebook* pb_)
        : plugin{name_, pb_}
        , sb{pb->lookup_impl<switchboard>()}
        //why buffered_read?
        , _m_scannet_datum{sb->get_reader<scene_recon_type>("ScanNet_Data")} {
        eCAL::Initialize(0, NULL, "Scene Reconstruction Device Transmitter");
        publisher = eCAL::protobuf::CPublisher<sr_input_proto::SRData>("sr_input");
        publisher.SetLayerMode(eCAL::TLayer::tlayer_udp_mc, eCAL::TLayer::smode_off);
        publisher.SetLayerMode(eCAL::TLayer::tlayer_tcp, eCAL::TLayer::smode_auto);
        frame_count=0;
    }

    virtual void start() override {
        plugin::start();

        sb->schedule<scene_recon_type>(id,"ScanNet_Data",[&](switchboard::ptr<const scene_recon_type> datum, std::size_t){
            this->send_sr_data(datum);
        });
    }

    void send_sr_data(switchboard::ptr<const scene_recon_type> datum) {
        printf("send data with frame %u\n", frame_count);
        outgoing_payload = new sr_input_proto::SRData();

        sr_input_proto::Pose* pose = outgoing_payload->mutable_input_pose();
        //set pose
        pose->set_p_x(datum->pose.position.x());
        pose->set_p_y(datum->pose.position.y());
        pose->set_p_z(datum->pose.position.z());

        pose->set_o_x(datum->pose.orientation.x());
        pose->set_o_y(datum->pose.orientation.y());
        pose->set_o_z(datum->pose.orientation.z());
        pose->set_o_w(datum->pose.orientation.w());

        sr_input_proto::ImgData* depth_img = outgoing_payload->mutable_depth_img_data(); 
        cv::Mat cur_depth{datum->depth};
        depth_img->set_rows(cur_depth.rows);
        depth_img->set_columns(cur_depth.cols);
        depth_img->set_img_data((void*) cur_depth.data, cur_depth.rows * cur_depth.cols);

        sr_input_proto::ImgData* rgb_img = outgoing_payload->mutable_rgb_img_data();
        cv::Mat cur_rgb{datum->rgb};
        rgb_img->set_rows(cur_rgb.rows);
        rgb_img->set_columns(cur_rgb.cols);
        rgb_img->set_img_data((void*) cur_rgb.data, cur_rgb.rows * cur_rgb.cols);

        ////this is the data being sned
        publisher.Send(*outgoing_payload);
        delete outgoing_payload;
        frame_count++;
    }

private:
    long                        previous_timestamp = 0;
    sr_input_proto::SRData* outgoing_payload;

    const std::shared_ptr<switchboard>                     sb;
    eCAL::protobuf::CPublisher<sr_input_proto::SRData> publisher;

    switchboard::reader<scene_recon_type> _m_scannet_datum;
    unsigned frame_count;
};

PLUGIN_MAIN(offload_writer)
