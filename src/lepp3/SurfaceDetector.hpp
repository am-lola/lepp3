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
                        public SurfaceDataObserver, public SurfaceDataSubject {
public:
  SurfaceDetector(bool surfaceDetectorActive, typename SurfaceFinder<PointT>::Parameters const& surfFinderParameters)
      : surfaceDetectorActive(surfaceDetectorActive),
        finder_(new SurfaceFinder<PointT>(surfaceDetectorActive, surfFinderParameters)),
        frameNum(1),
        surfaceDetectionIteration(0),
        surfaceReferenceFrameNum(0),
        planeCoeffsIteration(0),
        planeCoeffsReferenceFrameNum(0),
        exchangeCloud(PointCloudPtr(new PointCloudT())),
        newInputCloud(false),
        exit_threads_(false) {
    // start surface pipeline thread
    if (surfaceDetectorActive) {
      threads_.emplace_back(&SurfaceDetector::clusterTask, this);
    }

    // start thread for RANSAC surface coefficient finder
    threads_.emplace_back(&SurfaceDetector::ransacTask, this);
  }

  virtual ~SurfaceDetector() {
    exit_threads_ = true;
    for (auto& t : threads_) {
      t.join();
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
  // mutex to fill exchangeCloud
  std::mutex exchangeCloudMutex;

  // the exchange planes and exchange coefficients are needed for communication of the
  // ransac task and the surface detection/main thread. The ransac task fills both vectors
  // with newly detected planes and their coefficients. The surface detection and main thread
  // use these data in their computations.
  std::vector<PointCloudPtr> exchangePlanes;
  std::vector<pcl::ModelCoefficients> exchangePlaneCoefficients;


  /**
   * Container for all threads
   * Necessary to cleanly exit them
   */
  std::vector<std::thread> threads_;
  /**
   * Flag to signal threads to exit
   */
  bool exit_threads_;

  // holds the last frame number
  long frameNum;

  // counts the iterations the surface detection thread
  long surfaceDetectionIteration;

  // holds the frame number of the cloud to which the current exchange surfaces belong
  long surfaceReferenceFrameNum;

  // counts the number of iterations of the SurfaceFinder aka ransac task
  long planeCoeffsIteration;

  // holds the frame number of the cloud to which the newest plane coefficients belong
  long planeCoeffsReferenceFrameNum;

  // this vector is the output from the surface detection pipeline (sufaceDetectionThread).
  std::vector<SurfaceModelPtr> exchangeSurfaces;

  // when there is a new frame, the point cloud from the frame is copied into this
  // exchange cloud. The exchange cloud is used by the SurfaceFinder to find new model
  // coefficients. The point cloud in frame data cannot be used for this since the
  // SurfaceFinder ransac algorithm runs in its own thread independently.
  PointCloudPtr exchangeCloud;

  // this boolean indicates whether there is a new input cloud and the model coefficients
  // should be computed again
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
void SurfaceDetector<PointT>::clusterTask() {
  while (!exit_threads_) {
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
void SurfaceDetector<PointT>::ransacTask() {
  while (!exit_threads_) {
    // copy cloud pointer to local variable
    PointCloudPtr cloud;
    bool computeNewCoefficients = false;

    {
      std::lock_guard<std::mutex> lock(exchangeCloudMutex);
      cloud = exchangeCloud;
      computeNewCoefficients = newInputCloud;
      newInputCloud = false;
    }
    assert(cloud);

    // find plane coefficients with ransac if there is a new cloud set
    if (computeNewCoefficients) {
      // store current frame num before calling ransac
      planeMutex.lock();
      long tmpFrameNum = frameNum;
      planeMutex.unlock();

      // find planes and plane coefficients in current cloud
      std::vector<PointCloudPtr> planes;
      std::vector<pcl::ModelCoefficients> planeCoefficients;
      finder_->findSurfaces(cloud, planes, planeCoefficients);

      // copy detected planes and plane coefficients over in exchange variables
      planeMutex.lock();
      // copy back planes and plane coefficients into exchange variables
      exchangePlanes = planes;
      exchangePlaneCoefficients = planeCoefficients;
      // increase the number of ransac iterations 
      // (iterations to find planes and plane coefficients of current cloud)
      planeCoeffsIteration++;
      // store back frame num to which the computed coefficients belong
      planeCoeffsReferenceFrameNum = tmpFrameNum;
      planeMutex.unlock();
    }
  }
}


template<class PointT>
void SurfaceDetector<PointT>::updateSurfaces(SurfaceDataPtr surfaceData) {
  // copy detected surfaces over in exchangeSurfaces variable
  surfaceMutex.lock();
  exchangeSurfaces = surfaceData->surfaces;
  surfaceReferenceFrameNum = surfaceData->frameNum;
  surfaceDetectionIteration++;
  surfaceMutex.unlock();
}


template<class PointT>
void SurfaceDetector<PointT>::updateFrame(FrameDataPtr frameData) {
  // copy latest coefficients into frameData
  planeMutex.lock();
  frameData->planeCoefficients = exchangePlaneCoefficients;
  // store the planeCoeffsIteration cound and planeCoeffsReferenceFrameNum in frameData
  frameData->planeCoeffsIteration = planeCoeffsIteration;
  frameData->planeCoeffsReferenceFrameNum = planeCoeffsReferenceFrameNum;
  // store frame number of current frame
  frameNum = frameData->frameNum;
  planeMutex.unlock();

  // copy current point cloud into exchange variable
  {
    std::lock_guard<std::mutex> lock(exchangeCloudMutex);
    exchangeCloud = PointCloudPtr(new PointCloudT(*frameData->cloud));
    newInputCloud = true;
  }

  if (surfaceDetectorActive) {
    // copy latest detected surfaces into frameData
    surfaceMutex.lock();
    frameData->surfaces = exchangeSurfaces;
    frameData->surfaceReferenceFrameNum = surfaceReferenceFrameNum;
    frameData->surfaceDetectionIteration = surfaceDetectionIteration;
    surfaceMutex.unlock();
  }
  FrameDataSubject::notifyObservers(frameData);
}

}
#endif
