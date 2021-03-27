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

#ifndef CHISEL_H_
#define CHISEL_H_

#include <open_chisel/threading/Threading.h>
#include <open_chisel/ChunkManager.h>
#include <open_chisel/ProjectionIntegrator.h>
#include <open_chisel/geometry/Geometry.h>
#include <open_chisel/camera/PinholeCamera.h>
#include <open_chisel/camera/DepthImage.h>
#include <open_chisel/geometry/Frustum.h>
#include <open_chisel/pointcloud/PointCloud.h>

namespace chisel
{

    class Chisel
    {
        public:
            Chisel();
            Chisel(const Eigen::Vector3i& chunkSize, float voxelResolution, bool useColor);
            virtual ~Chisel();

            inline const ChunkManager& GetChunkManager() const { return chunkManager; }
            inline ChunkManager& GetMutableChunkManager() { return chunkManager; }
            inline void SetChunkManager(const ChunkManager& manager) { chunkManager = manager; }

            void IntegratePointCloud(const ProjectionIntegrator& integrator,
                                     const PointCloud& cloud,
                                     const Transform& extrinsic,
                                     float maxDist);
            template <class DataType> void IntegrateDepthScan(const ProjectionIntegrator& integrator,
                                                              const DepthImage<DataType>* depthImage,
//                                                              const std::shared_ptr<const DepthImage<DataType> >& depthImage,
                                                              const Transform* extrinsic,
                                                              const PinholeCamera& camera)
            {
                    printf("CHISEL: Integrating a scan\n");

                    DataType minimum, maximum, mean; 
                    depthImage->GetStats(minimum, maximum, mean); 
                    printf("depth img stats: minimum : %f maximum: %f mean: %f \n", minimum, maximum, mean);

                    Frustum frustum;
/*                    printf("#2 fx: %f, fy: %f, cx: %f, cy: %f \n", camera.GetIntrinsics().GetFx(),
                                                                    camera.GetIntrinsics().GetFy(),
                                                                    camera.GetIntrinsics().GetCx(),
                                                                    camera.GetIntrinsics().GetCy());
*/
                    PinholeCamera cameraCopy = camera;
                    cameraCopy.SetNearPlane(static_cast<float>(minimum));
                    cameraCopy.SetFarPlane(static_cast<float>(maximum));
/*
                    printf("#3 fx: %f, fy: %f, cx: %f, cy: %f \n", cameraCopy.GetIntrinsics().GetFx(),
                                                                    cameraCopy.GetIntrinsics().GetFy(),
                                                                    cameraCopy.GetIntrinsics().GetCx(),
                                                                    cameraCopy.GetIntrinsics().GetCy());

                    for(int i=0; i< 8; i++)
                    {
                        printf("old x: %f , old y: %f, old z: %f \n", frustum.GetCorners()[i].x(),
                                                                      frustum.GetCorners()[i].y(),
                                                                      frustum.GetCorners()[i].z());
                    }
*/
                    cameraCopy.SetupFrustum(*extrinsic, &frustum);
                    for(int i=0; i< 8; i++)
                    {
                        printf("new x: %f , new y: %f, new z: %f \n", frustum.GetCorners()[i].x(),
                                                                      frustum.GetCorners()[i].y(),
                                                                      frustum.GetCorners()[i].z());                                                                      
                    }

//                    printf("#2 orig x: %f, y:  %f, z: %f \n", extrinsic->translation()(0), extrinsic->translation()(1), extrinsic->translation()(2));

                    ChunkIDList chunksIntersecting;
                    //323 stop
                    chunkManager.GetChunkIDsIntersecting(frustum, &chunksIntersecting);
                    //printf("ILLIXR checkpoint 0.5\n");

                    std::mutex mutex;
                    ChunkIDList garbageChunks;
                    for(const ChunkID& chunkID : chunksIntersecting)
                    //parallel_for(chunksIntersecting.begin(), chunksIntersecting.end(), [&](const ChunkID& chunkID)
                    {
                        bool chunkNew = false;
//                        printf("ILLIXR checkpoint 1\n");
                        mutex.lock();
                        if (!chunkManager.HasChunk(chunkID))
                        {
                           chunkNew = true;
                           //this is where chunkmap getting inserted
                           chunkManager.CreateChunk(chunkID);
                        }

                        ChunkPtr chunk = chunkManager.GetChunk(chunkID);
                        mutex.unlock();
                 //       printf("ILLIXR checkpoint 2\n");

                        bool needsUpdate = integrator.Integrate(depthImage, camera, extrinsic, chunk.get());
                        //bool needsUpdate = true;
                        mutex.lock();
                        if (needsUpdate)
                        {
                            int count=0;
                            printf("needs update\n");
                            for (int dx = -1; dx <= 1; dx++)
                            {
                                for (int dy = -1; dy <= 1; dy++)
                                {
                                    for (int dz = -1; dz <= 1; dz++)
                                    {
                                        count++;
                                        meshesToUpdate[chunkID + ChunkID(dx, dy, dz)] = true;
                                    }
                                }
                            }
                            printf("ILLIXR: %d meshes need update\n", count);
                        }
                        else if(chunkNew)
                        {
                            garbageChunks.push_back(chunkID);
                        }
                        mutex.unlock();
                     //   printf("ILLIXR checkpoint 3\n");

                    }
                    //);
                    printf("CHISEL: Integrate Done with scan\n");
                    GarbageCollect(garbageChunks);
                    chunkManager.PrintMemoryStatistics();
            }

            template <class DataType, class ColorType> void IntegrateDepthScanColor(const ProjectionIntegrator& integrator, const std::shared_ptr<const DepthImage<DataType> >& depthImage,  const Transform& depthExtrinsic, const PinholeCamera& depthCamera, const std::shared_ptr<const ColorImage<ColorType> >& colorImage, const Transform& colorExtrinsic, const PinholeCamera& colorCamera)
            {
                    Frustum frustum;
                    depthCamera.SetupFrustum(depthExtrinsic, &frustum);

                    ChunkIDList chunksIntersecting;
                    chunkManager.GetChunkIDsIntersecting(frustum, &chunksIntersecting);

                    std::mutex mutex;
                    ChunkIDList garbageChunks;
                    //for ( const ChunkID& chunkID : chunksIntersecting)
                    parallel_for(chunksIntersecting.begin(), chunksIntersecting.end(), [&](const ChunkID& chunkID)
                    {

                        mutex.lock();
                        bool chunkNew = false;
                        if (!chunkManager.HasChunk(chunkID))
                        {
                           chunkNew = true;
                           chunkManager.CreateChunk(chunkID);
                        }

                        ChunkPtr chunk = chunkManager.GetChunk(chunkID);
                        mutex.unlock();


                        bool needsUpdate = integrator.IntegrateColor(depthImage, depthCamera, depthExtrinsic, colorImage, colorCamera, colorExtrinsic, chunk.get());

                        mutex.lock();
                        if (needsUpdate)
                        {
                            for (int dx = -1; dx <= 1; dx++)
                            {
                                for (int dy = -1; dy <= 1; dy++)
                                {
                                    for (int dz = -1; dz <= 1; dz++)
                                    {
                                        meshesToUpdate[chunkID + ChunkID(dx, dy, dz)] = true;
                                    }
                                }
                            }
                        }
                        else if(chunkNew)
                        {
                            garbageChunks.push_back(chunkID);
                        }
                        mutex.unlock();
                    }
                    );

                    GarbageCollect(garbageChunks);
                    //chunkManager.PrintMemoryStatistics();
            }

            void GarbageCollect(const ChunkIDList& chunks);
            void UpdateMeshes();

            bool SaveAllMeshesToPLY(const std::string& filename);
            void Reset();

            const ChunkSet& GetMeshesToUpdate() const { return meshesToUpdate; }

        protected:
            ChunkManager chunkManager;
            ChunkSet meshesToUpdate;

    };
    typedef std::shared_ptr<Chisel> ChiselPtr;
    typedef std::shared_ptr<const Chisel> ChiselConstPtr;

} // namespace chisel 

#endif // CHISEL_H_ 
