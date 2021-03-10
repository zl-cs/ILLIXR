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

//yihan: basically rewrite the server without chisel
#ifndef _SERVER_H_
#define _SERVER_H_

#include <memory>
#include <open_chisel/Chisel.h>
#include <open_chisel/ProjectionIntegrator.h>
#include <open_chisel/camera/PinholeCamera.h>
#include <open_chisel/camera/DepthImage.h>
#include <open_chisel/camera/ColorImage.h>
#include <open_chisel/pointcloud/PointCloud.h>
namespace chisel_server
{

    typedef float DepthData;
//    yihan: not supporting color right now
//    typedef uint8_t ColorData;

    class ChiselServer
    {
        public:


            ChiselServer();
            ChiselServer(int chunkSizeX, int chunkSizeY, int chunkSizeZ, float resolution, bool color);
            virtual ~ChiselServer();

            inline chisel::ChiselPtr GetChiselMap() { return chiselMap; }
            inline void SetChiselMap(const chisel::ChiselPtr value) { chiselMap = value; }

            inline const std::string& GetBaseTransform() const { return baseTransform; }
            inline const std::string& GetMeshTopic() const { return meshTopic; }

            void SetupProjectionIntegrator(const chisel::Vec4& truncation, uint16_t weight, bool useCarving, float carvingDist);

	    //have no good solution to deal with now
            void SetupMeshPublisher(const std::string& meshTopic);
            void SetupChunkBoxPublisher(const std::string& boxTopic);
            void SetupDepthPosePublisher(const std::string& depthPoseTopic);
            void SetupDepthFrustumPublisher(const std::string& frustumTopic);

            void PublishMeshes();
            void PublishChunkBoxes();
            void PublishLatestChunkBoxes();
            void PublishDepthPose();
            void PublishDepthFrustum();

            void IntegrateLastDepthImage();
            //needed to import depth and pose from plugin
            void IntegrateLastDepthImage(chisel::DepthImage<chisel_server::DepthData> *input_depth, chisel::Transform*  input_pose);
//            void FillMarkerTopicWithMeshes(visualization_msgs::Marker* marker);
            inline void SetBaseTransform(const std::string& frameName) { baseTransform = frameName; }

            inline bool HasNewData() { return hasNewData; }

            inline float GetNearPlaneDist() const { return nearPlaneDist; }
            inline float GetFarPlaneDist() const { return farPlaneDist; }
            inline void SetNearPlaneDist(float dist) { nearPlaneDist = dist; }
            inline void SetFarPlaneDist(float dist) { farPlaneDist = dist; }




         //   void SetDepthImage(const sensor_msgs::ImageConstPtr& img);
         //   void SetDepthPose(const Eigen::Affine3f& tf);
         //   void SetDepthCameraInfo(const sensor_msgs::CameraInfoConstPtr& info);

            //moved to public due to access need from plugin
            std::shared_ptr<chisel::DepthImage<DepthData> > lastDepthImage;
            //variable from roscameratopic that still needs from openchisel
            chisel::PinholeCamera cameraModel;
            chisel::Transform lastPose;
            std::chrono::time_point<std::chrono::system_clock> lastImageTimestamp;
            chisel::ChiselPtr chiselMap;

        protected:
            //visualization_msgs::Marker CreateFrustumMarker(const chisel::Frustum& frustum);

            chisel::ProjectionIntegrator projectionIntegrator;
            std::string baseTransform;
            std::string meshTopic;
            std::string chunkBoxTopic;
//            ros::Publisher meshPublisher;
//            ros::Publisher chunkBoxPublisher;
//            ros::Publisher latestChunkPublisher;
            bool useColor;
            bool hasNewData;
            float nearPlaneDist;
            float farPlaneDist;
    };
    typedef std::shared_ptr<ChiselServer> ChiselServerPtr;
    typedef std::shared_ptr<const ChiselServer> ChiselServerConstPtr;

} // namespace chisel_server

#endif // CHISELSERVER_H_ 
