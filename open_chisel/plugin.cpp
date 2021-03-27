#include <functional>

#include "common/threadloop.hpp"
#include "common/plugin.hpp"
#include "common/switchboard.hpp"
#include "common/data_format.hpp"
#include "common/phonebook.hpp"
#include "common/pose_prediction.hpp"
//added library
#include <open_chisel/Chisel.h>
#include <open_chisel/Server.h>
#include <open_chisel/ProjectionIntegrator.h>

#include <opencv2/opencv.hpp>


//3/18 use RBGD dataset
#include "data_loading.hpp"

using namespace ILLIXR;
//yihan: using threadloop
class open_chisel : public threadloop {
public:
	open_chisel(std::string name_, phonebook* pb_)
		: threadloop{name_, pb_}
		//subscribe to pose
//		, pp{pb->lookup_impl<pose_prediction>()}
		//3/18 use dataset instead
		, _m_sensor_data{load_data()}
		, _m_sensor_data_it{_m_sensor_data.cbegin()}
		, dataset_first_time{_m_sensor_data.cbegin()->first}
//		, _m_rgb_depth{sb->subscribe_latest<rgb_depth_type>("rgb_depth")}
		, sb{pb->lookup_impl<switchboard>()}
    {
	    //initialize at thread launch time
		int chunkSizeX,chunkSizeY,chunkSizeZ;
		double voxelResolution;
		double truncationDistQuad;
		double truncationDistLinear;
		double truncationDistConst;
		double truncationDistScale;
		int weight;
		bool useColor;
		bool useCarving;
		double carvingDist;
/*
        double carvingDist
        std::string depthImageTopic;
        std::string depthImageInfoTopic;
        std::string depthImageTransform;
        std::string colorImageTopic;
        std::string colorImageInfoTopic;
        std::string colorImageTransform;
        std::string baseTransform;
        std::string baseTransform;
        std::string meshTopic;
        std::string chunkBoxTopic;
*/
		double nearPlaneDist;
		double farPlaneDist;
/*
        chisel_ros::ChiselServer::FusionMode mode;
        std::string modeString;
        std::string pointCloudTopic;
*/

    
		fileToSave = "/home/yihan/openchisel_output/chisel.py";


        //initialize
		chunkSizeX = chunkSizeY = chunkSizeZ = 32;
		truncationDistConst = 0.001504;
		truncationDistQuad = 0.0019;
		truncationDistLinear = 0.00152;
		truncationDistScale = 8.0;

		weight = 1;

		voxelResolution = 0.03;

		nearPlaneDist = 0.05;
		carvingDist = 0.05;
		farPlaneDist = 5.0;

		useCarving =  true;
		useColor = false;


        //Setting up openchisel
        printf("input dataset frame # %lu\n", _m_sensor_data.size());
        //in include/open_chisel/truncation/quadratictruncator.h
		chisel::Vec4 truncation(truncationDistQuad, truncationDistLinear, truncationDistConst, truncationDistScale);

		//line 104 this is essentially a pointer to the newly created chiselserver

	    server = chisel_server::ChiselServerPtr(new chisel_server::ChiselServer(chunkSizeX, chunkSizeY, chunkSizeZ, voxelResolution, useColor));
		server->SetupProjectionIntegrator(	truncation,
							static_cast<uint16_t>(weight),
							useCarving,
							carvingDist);

		server->SetNearPlaneDist(nearPlaneDist);
		server->SetFarPlaneDist(farPlaneDist);

		//synchonrously read camera and look up  depth image this is essentially equals to
		//server->SubscribeDepthImage(depthImageTopic, depthImageInfoTopic, depthImageTransform);
		//this needs SetDepthCameraInfo which calls roscameratochiselcamera
		//fx fy calibrated value from zed , averaged VGA parameter of left eye and right eye
        //3/18 update using tum rgbd dataset 

		chisel::Intrinsics cam_intrinsics;

		cam_intrinsics.SetFx(525.0);
		cam_intrinsics.SetFy(525.0);
		cam_intrinsics.SetCx(319.5);
		cam_intrinsics.SetCy(239.5);

		server->cameraModel.SetIntrinsics(cam_intrinsics);
		server->cameraModel.SetWidth(640);
		server->cameraModel.SetHeight(480);
		server->cameraModel.SetNearPlane(nearPlaneDist);
		server->cameraModel.SetFarPlane(farPlaneDist);

        cur_timestamp = 0;

        _m_start_of_time = std::chrono::high_resolution_clock::now();
		printf("================================OpenChisel: setup finished==========================\n");

	}

    virtual skip_option _p_should_skip() override{
        if(_m_sensor_data_it != _m_sensor_data.end())
        {
            //for dataset no need to yield, simply process the all dataset
            if(cur_timestamp < _m_sensor_data_it->first){
                cur_timestamp = _m_sensor_data_it->first;
                return skip_option::run;
            }
            else{
                ++_m_sensor_data_it;
                return skip_option::skip_and_yield;
            }
        }
        else 
        {
    		server->GetChiselMap()->SaveAllMeshesToPLY(fileToSave);
            return skip_option::skip_and_yield;
        }
        
    /*
        if(_m_sensor_data_it != _m_sensor_data.end())
        {
            std::this_thread::sleep_for(
                                        std::chrono::nanoseconds{_m_sensor_data_it->first - dataset_first_time}
                                        + _m_start_of_time 
                                        - std::chrono::high_resolution_clock::now());
            if(cur_timestamp < _m_sensor_data_it->first)
            {
                cur_timestamp = _m_sensor_data_it->first;
                return skip_option::run;
            }
            else
            {
                ++_m_sensor_data_it;
                return skip_option::skip_and_yield;
            }
        }
        else
        {
            return skip_option::stop;
        }
    */
    }

    void _p_one_iteration() override 
	{
		printf("================================OpenChisel: depth info received==========================\n");
		int depth_column = 640;
		int depth_row = 480;

        //DepthData is float 
        chisel::DepthImage<chisel_server::DepthData> *depth_data = new chisel::DepthImage<chisel_server::DepthData>(depth_column, depth_row);

        //use opencv to read depth info
        //essentially performing ROSImgToDepthImg function
        const char *illixr_data_c_str = std::getenv("ILLIXR_DATA");
        std::string illixr_data = std::string{illixr_data_c_str};
        std::string depth_path = illixr_data;
        depth_path += "/";
        depth_path += _m_sensor_data_it->second.depth_img;
//        printf("depth path %s\n", depth_path.c_str());
        std::string temp = "/home/yihan/ILLIXR/"+illixr_data + "/"+_m_sensor_data_it->second.depth_img;
//        printf("merged path %s\n", temp.c_str());
        cv::Mat depth_info = cv::imread(temp, cv::IMREAD_UNCHANGED);
        //cv::imshow("test display", depth_info);
        //cv::waitKey(0);
        printf("cols: %d, rows: %d, dims: %d\n", depth_info.cols,depth_info.rows, depth_info.dims);


        //This essentially performs ROSImgToDepthImg called by SetDepthImage
        for(int i=0; i < depth_row; i++){
            for(int j=0; j<depth_column; j++){
                if(depth_info.at<ushort>(i,j) != 0)
                {
                    depth_data->SetDataAt(i,j,depth_info.at<ushort>(i,j)/5000.0f);
//                    printf("data entered: %d, %d: %f\n", i,j,depth_data->DepthAt(i,j));
                }
                else
                {
                    depth_data->SetDataAt(i,j,0.0f);
                }
//                printf("data received: at %d, %d: %f\n", i,j,depth_info.at<float>(i,j)/5.f);
            }
        }
        //use pose predicition with current time stamp of depth image
//        const fast_pose_type latest_pose = pp->get_fast_pose();
//        printf("compute time %llu, target time %llu \n", latest_pose.predict_computed_time, latest_pose.predict_target_time);
//        printf("past set data\n");

//      3/18 get latest pose from loaded dataset data
        auto latest_pose = _m_sensor_data.find(cur_timestamp);
        if(latest_pose == _m_sensor_data.end())
        {
            printf("pose for this timestamp is not found\n");
        }
        //use chisel format
        //this is essentially RosTfToChiselTf called by DepthImageCallback
        chisel::Transform pose_data (chisel::Transform::Identity());
        pose_data.translation()(0) = latest_pose->second.position.x();
        pose_data.translation()(1) = latest_pose->second.position.y();
        pose_data.translation()(2) = latest_pose->second.position.z();
        printf("orig x: %f, y:  %f, z: %f \n", latest_pose->second.position.x(), latest_pose->second.position.y(),
                                                latest_pose->second.position.z());


        chisel::Quaternion quat(chisel::Quaternion::Identity());
        quat.x() = latest_pose->second.orientation.x();
        quat.y() = latest_pose->second.orientation.y();
        quat.z() = latest_pose->second.orientation.z();
        quat.w() = latest_pose->second.orientation.w();
        pose_data.linear() = quat.toRotationMatrix();
//        chisel::Transform inv_pose_data = pose_data.inverse();

        
//        printf("chiselmap #4: %p\n",(void*)&(server->chiselMap));

		server->IntegrateLastDepthImage(depth_data, &pose_data);
//		server->IntegrateLastDepthImage(depth_data, &inv_pose_data);
//		server->chiselMap->UpdateMeshes();
//        printf("reached here\n");
	    const chisel::MeshMap& meshMap = server->chiselMap->GetChunkManager().GetAllMeshes();
	    printf("mesh map size %lu\n", meshMap.size());
	    printf("chunk map size %lu\n", server->chiselMap->GetChunkManager().GetChunks().size());

	}

	virtual ~open_chisel() override {
	    printf("chisel exit\n");
	    const chisel::MeshMap& meshMap = server->chiselMap->GetChunkManager().GetAllMeshes();
	    printf("mesh map size %lu\n", meshMap.size());
	    printf("chunk map size %lu\n", server->chiselMap->GetChunkManager().GetChunks().size());
		server->GetChiselMap()->SaveAllMeshesToPLY(fileToSave);
	}

private:
	const std::shared_ptr<switchboard> sb;
//	const std::shared_ptr<pose_prediction> pp;
//	std::unique_ptr<reader_latest<rgb_depth_type>> _m_rgb_depth;
	//need chiselMap in order to have ptr same setup as ChiselServer.h line 168
	//since chisel server only calls chisel thus this is essentially a chisel ptr
	//copied from  include/chisel_ros/ChiselServer.h:173
    chisel_server::ChiselServerPtr server;
	chisel::ProjectionIntegrator projectionIntegrator;
	std::string fileToSave;

    //3/18 use dataset
    const std::map<ullong, sensor_types> _m_sensor_data;
    ullong dataset_first_time;
    time_type _m_start_of_time;
    std::unique_ptr<reader_latest<time_type>> _m_vsync_estimate;
    std::map<ullong, sensor_types>::const_iterator _m_sensor_data_it;
    //timestamp to track progress
    ullong cur_timestamp;
};

PLUGIN_MAIN(open_chisel)
