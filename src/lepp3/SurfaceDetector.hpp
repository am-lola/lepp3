#ifndef BASE_SURFACE_DETECTOR_H_
#define BASE_SURFACE_DETECTOR_H_

#include "lepp3/Typedefs.hpp"
#include "lepp3/SurfaceFinder.hpp"
#include "lepp3/SurfaceClusterer.hpp"
#include "lepp3/FrameData.hpp"
#include "lepp3/SurfaceData.hpp"

#include <vector>
#include <thread>
#include <mutex>

namespace lepp {

template<class PointT>
class SurfaceDetector : public FrameDataObserver, public FrameDataSubject, 
                        public SurfaceDataObserver, public SurfaceDataSubject
{
 public:
    SurfaceDetector(bool surfaceDetectorActive)
      : surfaceDetectorActive(surfaceDetectorActive),
        finder_(new SurfaceFinder<PointT>(surfaceDetectorActive)),
        frameNum(0), surfaceFrameNum(0), surfaceReferenceFrameNum(0), 
        exchangeCloud(PointCloudPtr(new PointCloudT())), newInputCloud(false)
    {
      // start surface pipeline thread
      if (surfaceDetectorActive)
      {
        // Constructs the new thread and runs it. Does not block execution.
        std::thread sufaceDetectionThread(&SurfaceDetector::clusterTask, this);

        // allow execution to continue independently
        sufaceDetectionThread.detach();
      }

      // start thread for RANSAC surface coefficient finder
      std::thread ransacThread(&SurfaceDetector::ransacTask, this);
      ransacThread.detach();
    }

    /**
     * FrameDataObserver interface method implementation.
     */
    virtual void updateFrame(FrameDataPtr frameData);

    /**
    * SurfaceDataObserver interface method implementation.
    */
    virtual void updateSurfaces(SurfaceDataPtr surfaceData);

  private:
    boost::shared_ptr<SurfaceFinder<PointT> > finder_;

    // boolean indicating whether the surface detector was enabled in config files
    bool surfaceDetectorActive;

    // mutex variables that limits access to exchangePlanes and 
    // exchangePlaneCoefficients variable
    std::mutex planeMutex;
    // mutex variable that limits access to exchangeSurfaces variable
    std::mutex surfaceMutex;
    // mutex to fill exchangeCloud
    std::mutex exchangeCloudMutex;

    // variables that are needed for communication between the main thread and 
    // the surface detection thread
    std::vector<PointCloudPtr> exchangePlanes;
    std::vector<pcl::ModelCoefficients> exchangePlaneCoefficients;
    long frameNum;
    long surfaceFrameNum;
    long surfaceReferenceFrameNum;
    std::vector<SurfaceModelPtr> exchangeSurfaces;
    PointCloudPtr exchangeCloud;
    bool newInputCloud;

    /**
    * Method that is called by surfaceDetectionThread. It invokes the clustering and
    * approximation of surfaces with convex hulls.
    */
    void clusterTask();

    /**
    * Method that is called by ransac thread. It invokes the detection of surface
    * coefficients and the corresponding planes using RANSAC
    */
    void ransacTask();
};


template<class PointT>
void SurfaceDetector<PointT>::clusterTask()
{
  while (true)
  {
    // copy planes from exchange variables to local variables
    planeMutex.lock();
    SurfaceDataPtr surfaceData(new SurfaceData(frameNum));
    surfaceData->planes = exchangePlanes;
    surfaceData->planeCoefficients = exchangePlaneCoefficients;
    planeMutex.unlock();

    // invoke surface pipeline if there are any planes
    if (surfaceData->planes.size() != 0)
      SurfaceDataSubject::notifyObservers(surfaceData);
  }
}

template<class PointT>
void SurfaceDetector<PointT>::ransacTask()
{
  while(true)
  {
    // copy cloud pointer to local variable
    exchangeCloudMutex.lock();
    PointCloudPtr cloud = exchangeCloud;
    bool computeNewCoefficients = newInputCloud;
    newInputCloud = false;
    exchangeCloudMutex.unlock();

    // find plane coefficients with ransac
    if (computeNewCoefficients)
    {
      std::vector<PointCloudPtr> planes;
      std::vector<pcl::ModelCoefficients> planeCoefficients;
      if (cloud->size() > 0)
        finder_->findSurfaces(cloud, planes, planeCoefficients);

      // copy detected planes and plane coefficients over in exchange variables
      planeMutex.lock();
      exchangePlanes = planes;
      exchangePlaneCoefficients = planeCoefficients;
      planeMutex.unlock();
    }
  }
}


template<class PointT>
void SurfaceDetector<PointT>::updateSurfaces(SurfaceDataPtr surfaceData)
{
  // copy detected surfaces over in exchangeSurfaces variable
  surfaceMutex.lock();
  // deep copy - enable in case of race conditions
  /*
  exchangeSurfaces.clear();
  for (size_t i = 0; i < surfaceData->surfaces.size(); i++)
  {
    int id = surfaceData->surfaces[i]->id();
    PointCloudPtr cloud(new PointCloudT(*surfaceData->surfaces[i]->get_cloud()));
    pcl::ModelCoefficients planeCoefficients(surfaceData->surfaces[i]->get_planeCoefficients());
    PointCloudPtr hull(new PointCloudT(*surfaceData->surfaces[i]->get_hull()));
    SurfaceModelPtr copyModel(new SurfaceModel(cloud, planeCoefficients));
    copyModel->set_id(id);
    copyModel->set_hull(hull);
    exchangeSurfaces.push_back(copyModel);
  }
  */
  exchangeSurfaces = surfaceData->surfaces;
  surfaceReferenceFrameNum = surfaceData->frameNum;
  surfaceFrameNum++;
  surfaceMutex.unlock();
}


template<class PointT>
void SurfaceDetector<PointT>::updateFrame(FrameDataPtr frameData) 
{
  // copy current point cloud into exchange variable
  exchangeCloudMutex.lock();
  exchangeCloud = PointCloudPtr(new PointCloudT(*frameData->cloud));
  newInputCloud = true;
  exchangeCloudMutex.unlock();

  // copy latest coefficients into frameData
  planeMutex.lock();
  frameData->planeCoefficients = exchangePlaneCoefficients; 
  frameNum = frameData->frameNum;   
  planeMutex.unlock();

  if (surfaceDetectorActive)
  {
    // copy latest detected surfaces into frameData
    surfaceMutex.lock();
    frameData->surfaces = exchangeSurfaces;
    frameData->surfaceReferenceFrameNum = surfaceReferenceFrameNum;
    frameData->surfaceFrameNum = surfaceFrameNum;
    surfaceMutex.unlock();
  }
  FrameDataSubject::notifyObservers(frameData);
}

}
#endif