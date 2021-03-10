#include <functional>

#include "common/threadloop.hpp"
#include "common/plugin.hpp"
#include "common/data_format.hpp"
#include "common/phonebook.hpp"
#include "common/pose_prediction.hpp"

//added library
#include <open_chisel/Chisel.h>
#include <open_chisel/Server.h>
#include <open_chisel/ProjectionIntegrator.h>

#include <opencv2/opencv.hpp>


using namespace ILLIXR;
//yihan: using threadloop
class open_chisel_impl : public threadloop {
public:
	open_chisel_impl(std::string name_, phonebook* pb_)
		: threadloop{name_, pb_}
		//subscribe to pose
		, sb{pb->lookup_impl<switchboard>()}
		, pp{pb->lookup_impl<pose_prediction>()}
		, _m_rgb_depth{sb->subscribe_latest<rgb_depth_type>("rgb_depth")}
    {
	    //initialize at thread launch time
		int chunkSizeX,chunkSizeY,chunkSizeZ;

		unsigned weight;

		double voxelResolution;
		double truncationDistQuad;
		double truncationDistLinear;
		double truncationDistConst;
		double truncationDistScale;
		double carvingDist;
		double nearPlaneDist;
		double farPlaneDist;

		bool useColor;
		bool useCarving;

//		fileToSave = "/home/yihan/openchisel_output/chisel.py";


        //initialize
		chunkSizeX = chunkSizeY = chunkSizeZ = 32;

		weight = 1;

		voxelResolution = 0.03;
		truncationDistQuad = 0.0019;
		truncationDistLinear = 0.00152;
		truncationDistConst = 0.001504;
		truncationDistScale = 8.0;
		nearPlaneDist = 0.05;
		farPlaneDist = 5.0;
		carvingDist = 0.05;

		useCarving =  false;
		useColor = false;

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
		chisel::Intrinsics cam_intrinsics;

		cam_intrinsics.SetFx(353.515);
		cam_intrinsics.SetFy(353.515);
		cam_intrinsics.SetCx(349.5); //depth reso 672x376
		cam_intrinsics.SetCy(193.555);
		server->cameraModel.SetIntrinsics(cam_intrinsics);
		server->cameraModel.SetWidth(672);
		server->cameraModel.SetHeight(376);
		server->cameraModel.SetNearPlane(nearPlaneDist);
		server->cameraModel.SetFarPlane(farPlaneDist);

        prev_timestamp = 0;

    	printf("fx: %f, fy: %f, cx: %f, cy: %f \n", server->cameraModel.GetIntrinsics().GetFx(),
                                            	    server->cameraModel.GetIntrinsics().GetFy(),
                                            	    server->cameraModel.GetIntrinsics().GetCx(),
                                            	    server->cameraModel.GetIntrinsics().GetCy());
		printf("================================OpenChisel: setup finished==========================\n");

	}

    virtual skip_option _p_should_skip() override{
        //check depth, if new depth incoming perform openchisel otherwise yield
        //printf("reached here\n");
        std::cout<<"reached here"<<std::endl;
/*
        auto input_depth = _m_rgb_depth->get_latest_ro();
        if (!input_depth || input_depth->timestamp == prev_timestamp)
        {
           std::this_thread::sleep_for(std::chrono::milliseconds{1});
           return skip_option::skip_and_yield; 
            printf("skip \n");
        }
        else
        {
            //might need to add stats tracker later
            printf("run \n");
            return skip_option::run;
        }
*/
        return skip_option::skip_and_yield;
    }

    void _p_one_iteration() override 
	{
		printf("================================OpenChisel: depth info received==========================\n");

/*
        //need to perform something like original setDeptImage and DepthImageCallback all together
        cv::Mat* depth_info = _m_rgb_depth->get_latest_ro()->depth.value();
        printf("depth row %d\n", depth_info->rows);
        printf("depth col %d\n", depth_info->cols);
        printf("depth dim %d\n", depth_info->dims);

        //DepthData is float 
        chisel::DepthImage<chisel_server::DepthData> *depth_data = new chisel::DepthImage<chisel_server::DepthData>(depth_info->cols, depth_info->rows);
        //This essentially performs ROSImgToDepthImg
        for(int i=0; i < depth_info->rows; i++){
            for(int j=0; j<depth_info->cols; j++){
                depth_data->SetDataAt(i,j,depth_info->at<float>(i,j)/1000.0);
//                printf("data entered: at %d, %d: %f\n", i,j,depth_info->at<float>(i,j));
            }
        }
        //use pose predicition with current time stamp of depth image
        const fast_pose_type latest_pose = pp->get_fast_pose(std::chrono::system_clock::now());
        //use chisel format
        //this is essentially RosTfToChiselTf
        chisel::Transform pose_data (chisel::Transform::Identity());
        pose_data.translation()(0) = latest_pose.pose.position.x();
        pose_data.translation()(1) = latest_pose.pose.position.y();
        pose_data.translation()(2) = latest_pose.pose.position.z();
        printf("orig x: %f, y:  %f, z: %f \n", latest_pose.pose.position.x(), latest_pose.pose.position.y(), latest_pose.pose.position.z());


        chisel::Quaternion quat(chisel::Quaternion::Identity());
        quat.x() = latest_pose.pose.orientation.x();
        quat.y() = latest_pose.pose.orientation.y();
        quat.z() = latest_pose.pose.orientation.z();
        quat.w() = latest_pose.pose.orientation.w();
        pose_data.linear() = quat.toRotationMatrix();
        chisel::Transform inv_pose_data = pose_data.inverse();

        
        printf("chiselmap #4: %p\n",(void*)&(server->chiselMap));

		server->IntegrateLastDepthImage(depth_data, &inv_pose_data);
        printf("reached here\n");
*/
	}

	virtual ~open_chisel_impl() override {
//		server->GetChiselMap()->SaveAllMeshesToPLY(fileToSave);
	}

private:
	const std::shared_ptr<switchboard> sb;
	const std::shared_ptr<pose_prediction> pp;

	
	std::unique_ptr<reader_latest<rgb_depth_type>> _m_rgb_depth;
	//need chiselMap in order to have ptr same setup as ChiselServer.h line 168
	//since chisel server only calls chisel thus this is essentially a chisel ptr
	//copied from  include/chisel_ros/ChiselServer.h:173
    chisel_server::ChiselServerPtr server;
	chisel::ProjectionIntegrator projectionIntegrator;
	std::string fileToSave;

    //timestamp to track progress
    ullong prev_timestamp;

};

PLUGIN_MAIN(open_chisel_impl)
