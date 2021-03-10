// The MIT License (MIT)
// Copyright (c) 2014 Matthew Klingensmith and Ivan Dryanovski
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include <open_chisel/Server.h>
//#include <visualization_msgs/Marker.h>
#include <open_chisel/truncation/QuadraticTruncator.h>
#include <open_chisel/weighting/ConstantWeighter.h>

namespace chisel_server
{
    ChiselServer::ChiselServer(int chunkSizeX, int chunkSizeY, int chunkSizeZ, float resolution, bool color) :
            useColor(color), hasNewData(false), farPlaneDist(0), nearPlaneDist(0)
    {
        chiselMap.reset(new chisel::Chisel(Eigen::Vector3i(chunkSizeX, chunkSizeY, chunkSizeZ), resolution, color));
//        printf("chiselmap #1: %p\n",(void*)&chiselMap);
    }


    ChiselServer::~ChiselServer()
    {

    }
    //need to figure out a way to publish the data
    void ChiselServer::SetupMeshPublisher(const std::string& topic)
    {
       // meshTopic = topic;
       // meshPublisher = nh.advertise<visualization_msgs::Marker>(meshTopic, 1);
    }

    void ChiselServer::PublishMeshes()
    {
	/*
        visualization_msgs::Marker marker;
        FillMarkerTopicWithMeshes(&marker);

        if(!marker.points.empty())
            meshPublisher.publish(marker);
	*/
    }


    void ChiselServer::SetupChunkBoxPublisher(const std::string& boxTopic)
    {
	/*
        chunkBoxTopic = boxTopic;
        chunkBoxPublisher = nh.advertise<visualization_msgs::Marker>(chunkBoxTopic, 1);
        latestChunkPublisher = nh.advertise<visualization_msgs::Marker>(chunkBoxTopic + "/latest", 1);
	*/
    }

    void ChiselServer::SetupDepthPosePublisher(const std::string& depthPoseTopic)
    {
        //depthCamera.lastPosePublisher = nh.advertise<geometry_msgs::PoseStamped>(depthPoseTopic, 1);
    }

    void ChiselServer::SetupDepthFrustumPublisher(const std::string& frustumTopic)
    {
        //depthCamera.frustumPublisher = nh.advertise<visualization_msgs::Marker>(frustumTopic, 1);
    }

    void ChiselServer::PublishDepthFrustum()
    {
	/*
        chisel::Frustum frustum;
        depthCamera.cameraModel.SetupFrustum(depthCamera.lastPose, &frustum);
        visualization_msgs::Marker marker = CreateFrustumMarker(frustum);
        depthCamera.frustumPublisher.publish(marker);
	*/
    }

/*
    visualization_msgs::Marker ChiselServer::CreateFrustumMarker(const chisel::Frustum& frustum)
    {
        visualization_msgs::Marker marker;
        marker.id = 0;
        marker.header.frame_id = baseTransform;
        marker.color.r = 1.;
        marker.color.g = 1.;
        marker.color.b = 1.;
        marker.color.a = 1.;
        marker.pose.position.x = 0;
        marker.pose.position.y = 0;
        marker.pose.position.z = 0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.01;
        marker.scale.y = 0.01;
        marker.scale.z = 0.01;

        marker.type = visualization_msgs::Marker::LINE_LIST;
        const chisel::Vec3* lines = frustum.GetLines();
        for (int i = 0; i < 24; i++)
        {
            const chisel::Vec3& linePoint = lines[i];
            geometry_msgs::Point pt;
            pt.x = linePoint.x();
            pt.y = linePoint.y();
            pt.z = linePoint.z();
            marker.points.push_back(pt);
        }

        return marker;
    }
*/
    void ChiselServer::PublishDepthPose()
    {
        printf("ILLIXR_CHISEL: PublishDepthPose\n");
//	need to change poseformat depthcamera value intake
/*
        chisel::Transform lastPose = depthCamera.lastPose;

        geometry_msgs::PoseStamped pose;
        pose.header.frame_id = baseTransform;
        pose.header.stamp = depthCamera.lastImageTimestamp;
        pose.pose.position.x = lastPose.translation()(0);
        pose.pose.position.y = lastPose.translation()(1);
        pose.pose.position.z = lastPose.translation()(2);

        chisel::Quaternion quat(lastPose.rotation());
        pose.pose.orientation.x = quat.x();
        pose.pose.orientation.y = quat.y();
        pose.pose.orientation.z = quat.z();
        pose.pose.orientation.w = quat.w();

        depthCamera.lastPosePublisher.publish(pose);
*/
	
    }

/*
    void ChiselServer::SetDepthImage(const sensor_msgs::ImageConstPtr& img)
    {
        if (!lastDepthImage.get())
        {
          lastDepthImage.reset(new chisel::DepthImage<DepthData>(img->width, img->height));
        }

        ROSImgToDepthImg(img, lastDepthImage.get());
        depthCamera.lastImageTimestamp = img->header.stamp;
        depthCamera.gotImage = true;
    }

    void ChiselServer::DepthImageCallback(sensor_msgs::ImageConstPtr depthImage)
    {
        if (IsPaused()) return;
        SetDepthImage(depthImage);

        bool gotTransform = false;
        tf::StampedTransform tf;

        int tries = 0;
        int maxTries = 10;

        while(!gotTransform && tries < maxTries)
        {
            tries++;
            try
            {
                transformListener.waitForTransform(depthCamera.transform, baseTransform, depthImage->header.stamp, ros::Duration(0.5));
                transformListener.lookupTransform(depthCamera.transform, baseTransform, depthImage->header.stamp, tf);
                depthCamera.gotPose = true;
                gotTransform = true;
            }
            catch (std::exception& e)
            {
                ros::Rate lookupRate(0.5f);
                ROS_WARN("%s\n", e.what());
            }
        }

        depthCamera.lastPose = RosTfToChiselTf(tf);

        hasNewData = true;
    }
*/

    void ChiselServer::SetupProjectionIntegrator(const chisel::Vec4& truncation, uint16_t weight, bool useCarving, float carvingDist)
    {
        projectionIntegrator.SetCentroids(GetChiselMap()->GetChunkManager().GetCentroids());
        projectionIntegrator.SetTruncator(chisel::TruncatorPtr(new chisel::QuadraticTruncator(truncation(0), truncation(1), truncation(2), truncation(3))));
        projectionIntegrator.SetWeighter(chisel::WeighterPtr(new chisel::ConstantWeighter(weight)));
        projectionIntegrator.SetCarvingDist(carvingDist);
        projectionIntegrator.SetCarvingEnabled(useCarving);
    }


    //original IntegrateLastDepthImage()
    void ChiselServer::IntegrateLastDepthImage()
    {
           printf("CHISEL: Integrating depth scan\n");
         //  chiselMap->IntegrateDepthScan<DepthData>(projectionIntegrator, lastDepthImage, depthCamera.lastPose, depthCamera.cameraModel);
            printf("CHISEL: Done with scan\n");
            PublishLatestChunkBoxes();
            PublishDepthFrustum();

           // chiselMap->UpdateMeshes();
            //hasNewData = false;
    }

//modified IntegrateLastDepthImage()
    void ChiselServer::IntegrateLastDepthImage(chisel::DepthImage<chisel_server::DepthData>* input_depth, chisel::Transform* input_pose)
//    void IntegrateLastDepthImage(chisel::DepthImage<chisel_server::DepthData>* input_depth, chisel::Transform* input_pose)
    {
           printf("CHISEL: Integrating last depth scan\n");

/*
	       printf("x: %d, y:  %d, z: %d \n", input_pose.translation()(0), input_pose.translation()(1), input_pose.translation()(2));
	       for(int i=0; i<input_depth->GetWidth(); i++)
	       {
	        for(int j=0; j<input_depth->GetHeight(); j++)
	        {
	           printf("%f\n",input_depth->DepthAt(i,j)); 
	        }
	       }
*/

//           chiselMap->IntegrateDepthScan<DepthData>(projectionIntegrator, input_depth, input_pose, depthCamera.cameraModel);
           printf("orig x: %f, y:  %f, z: %f \n", input_pose->translation()(0), input_pose->translation()(1), input_pose->translation()(2));
           printf("chiselmap #2: %p\n",(void*)&chiselMap);
//           chiselMap->temp_IntegrateDepthScan(projectionIntegrator, input_depth, input_pose, cameraModel);
           printf("#1 fx: %f, fy: %f, cx: %f, cy: %f \n", cameraModel.GetIntrinsics().GetFx(),
                                                        cameraModel.GetIntrinsics().GetFy(),
                                                        cameraModel.GetIntrinsics().GetCx(),
                                                        cameraModel.GetIntrinsics().GetCy());

           chiselMap->IntegrateDepthScan<DepthData>(projectionIntegrator, input_depth, input_pose, cameraModel);

           printf("CHISEL: Done with scan\n");
	       //TODO how to publish this
           /*
           PublishLatestChunkBoxes();
           printf("failed 1\n");
           PublishDepthFrustum();
           printf("failed 2\n");
           chiselMap->UpdateMeshes();
           printf("failed 3\n");
           */

    }
    void ChiselServer::PublishLatestChunkBoxes()
    {
/*
        if (!latestChunkPublisher) return;
        const chisel::ChunkManager& chunkManager = chiselMap->GetChunkManager();
        visualization_msgs::Marker marker;
        marker.header.stamp = ros::Time::now();
        marker.header.frame_id = baseTransform;
        marker.ns = "chunk_box";
        marker.type = visualization_msgs::Marker::CUBE_LIST;
        marker.scale.x = chunkManager.GetChunkSize()(0) * chunkManager.GetResolution();
        marker.scale.y = chunkManager.GetChunkSize()(1) * chunkManager.GetResolution();
        marker.scale.z = chunkManager.GetChunkSize()(2) * chunkManager.GetResolution();
        marker.pose.position.x = 0;
        marker.pose.position.y = 0;
        marker.pose.position.z = 0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.color.r = 0.3f;
        marker.color.g = 0.95f;
        marker.color.b = 0.3f;
        marker.color.a = 0.6f;
        const chisel::ChunkSet& latest = chiselMap->GetMeshesToUpdate();
        for (const std::pair<chisel::ChunkID, bool>& id : latest)
        {
            if(chunkManager.HasChunk(id.first))
            {
                chisel::AABB aabb = chunkManager.GetChunk(id.first)->ComputeBoundingBox();
                chisel::Vec3 center = aabb.GetCenter();
                geometry_msgs::Point pt;
                pt.x = center.x();
                pt.y = center.y();
                pt.z = center.z();
                marker.points.push_back(pt);
            }
        }

        latestChunkPublisher.publish(marker);
*/
    }

    void ChiselServer::PublishChunkBoxes()
    {
/*
        const chisel::ChunkManager& chunkManager = chiselMap->GetChunkManager();
        visualization_msgs::Marker marker;
        marker.header.stamp = ros::Time::now();
        marker.header.frame_id = baseTransform;
        marker.ns = "chunk_box";
        marker.type = visualization_msgs::Marker::CUBE_LIST;
        marker.scale.x = chunkManager.GetChunkSize()(0) * chunkManager.GetResolution();
        marker.scale.y = chunkManager.GetChunkSize()(1) * chunkManager.GetResolution();
        marker.scale.z = chunkManager.GetChunkSize()(2) * chunkManager.GetResolution();
        marker.pose.position.x = 0;
        marker.pose.position.y = 0;
        marker.pose.position.z = 0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.color.r = 0.95f;
        marker.color.g = 0.3f;
        marker.color.b = 0.3f;
        marker.color.a = 0.6f;
        for (const std::pair<chisel::ChunkID, chisel::ChunkPtr>& pair : chunkManager.GetChunks())
        {
            chisel::AABB aabb = pair.second->ComputeBoundingBox();
            chisel::Vec3 center = aabb.GetCenter();
            geometry_msgs::Point pt;
            pt.x = center.x();
            pt.y = center.y();
            pt.z = center.z();
            marker.points.push_back(pt);
        }

        chunkBoxPublisher.publish(marker);
*/
    }

    chisel::Vec3 LAMBERT(const chisel::Vec3& n, const chisel::Vec3& light)
    {
        return fmax(n.dot(light), 0.0f) * chisel::Vec3(0.5, 0.5, 0.5);
    }
/*
    void ChiselServer::FillMarkerTopicWithMeshes(visualization_msgs::Marker* marker)
    {
        assert(marker != nullptr);
        marker->header.stamp = ros::Time::now();
        marker->header.frame_id = baseTransform;
        marker->scale.x = 1;
        marker->scale.y = 1;
        marker->scale.z = 1;
        marker->pose.orientation.x = 0;
        marker->pose.orientation.y = 0;
        marker->pose.orientation.z = 0;
        marker->pose.orientation.w = 1;
        marker->type = visualization_msgs::Marker::TRIANGLE_LIST;
        const chisel::MeshMap& meshMap = chiselMap->GetChunkManager().GetAllMeshes();

        if(meshMap.size() == 0)
        {
            return;
        }

        chisel::Vec3 lightDir(0.8f, -0.2f, 0.7f);
        lightDir.normalize();
        chisel::Vec3 lightDir1(-0.5f, 0.2f, 0.2f);
        lightDir.normalize();
        const chisel::Vec3 ambient(0.2f, 0.2f, 0.2f);
        //int idx = 0;
        for (const std::pair<chisel::ChunkID, chisel::MeshPtr>& meshes : meshMap)
        {
            const chisel::MeshPtr& mesh = meshes.second;
            for (size_t i = 0; i < mesh->vertices.size(); i++)
            {
                const chisel::Vec3& vec = mesh->vertices[i];
                geometry_msgs::Point pt;
                pt.x = vec[0];
                pt.y = vec[1];
                pt.z = vec[2];
                marker->points.push_back(pt);

                if(mesh->HasColors())
                {
                    const chisel::Vec3& meshCol = mesh->colors[i];
                    std_msgs::ColorRGBA color;
                    color.r = meshCol[0];
                    color.g = meshCol[1];
                    color.b = meshCol[2];
                    color.a = 1.0;
                    marker->colors.push_back(color);
                }
                else
                {
                  if(mesh->HasNormals())
                  {
                      const chisel::Vec3 normal = mesh->normals[i];
                      std_msgs::ColorRGBA color;
                      chisel::Vec3 lambert = LAMBERT(normal, lightDir) + LAMBERT(normal, lightDir1) + ambient;
                      color.r = fmin(lambert[0], 1.0);
                      color.g = fmin(lambert[1], 1.0);
                      color.b = fmin(lambert[2], 1.0);
                      color.a = 1.0;
                      marker->colors.push_back(color);
                  }
                  else
                  {
                    std_msgs::ColorRGBA color;
                    color.r = vec[0] * 0.25 + 0.5;
                    color.g = vec[1] * 0.25 + 0.5;
                    color.b = vec[2] * 0.25 + 0.5;
                    color.a = 1.0;
                    marker->colors.push_back(color);
                  }
                }
                //marker->indicies.push_back(idx);
                //idx++;
            }
        }
    }
*/

} // namespace chisel 
