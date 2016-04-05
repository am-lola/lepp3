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
        frameNum(0), surfaceFrameNum(0), surfaceReferenceFrameNum(0)
    {
      if (surfaceDetectorActive)
      {
        // Constructs the new thread and runs it. Does not block execution.
        std::thread sufaceDetectionThread(&SurfaceDetector::clusterTask, this);

        // allow execution to continue independently
        sufaceDetectionThread.detach();
      }
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

    // variables that are needed for communication between the main thread and 
    // the surface detection thread
    std::vector<PointCloudPtr> exchangePlanes;
    std::vector<pcl::ModelCoefficients> exchangePlaneCoefficients;
    long frameNum;
    long surfaceFrameNum;
    long surfaceReferenceFrameNum;
    std::vector<SurfaceModelPtr> exchangeSurfaces;

    /**
    * Method that is called by surfaceDetectionThread. It invokes the clustering and
    * approximation of surfaces with convex hulls.
    */
    void clusterTask();
};


template<class PointT>
void  SurfaceDetector<PointT>::clusterTask()
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
void SurfaceDetector<PointT>::updateSurfaces(SurfaceDataPtr surfaceData)
{
  // copy detected surfaces over in exchangeSurfaces variable
  surfaceMutex.lock();
  exchangeSurfaces = surfaceData->surfaces;
  surfaceReferenceFrameNum = surfaceData->frameNum;
  surfaceFrameNum++;
  surfaceMutex.unlock();
}


template<class PointT>
void SurfaceDetector<PointT>::updateFrame(FrameDataPtr frameData) 
{
  std::vector<PointCloudPtr> planes;
  std::vector<pcl::ModelCoefficients> planeCoefficients;
  // detect planes
  finder_->findSurfaces(frameData, planes, planeCoefficients);

  if (surfaceDetectorActive)
  {
    // copy detected planes and plane coefficients over in exchange variables
    planeMutex.lock();
    exchangePlanes = planes;
    exchangePlaneCoefficients = planeCoefficients;
    frameNum = frameData->frameNum;
    planeMutex.unlock();

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