#ifndef BASE_SURFACE_DETECTOR_H_
#define BASE_SURFACE_DETECTOR_H_

#include "lepp3/Typedefs.hpp"
#include "lepp3/SurfaceFinder.hpp"
#include "lepp3/SurfaceClusterer.hpp"
#include "lepp3/FrameData.hpp"

#include <vector>

namespace lepp {

template<class PointT>
class SurfaceDetector : public FrameDataObserver, public FrameDataSubject 
{
 public:
    SurfaceDetector(bool surfaceDetectorActive)
      : surfaceDetectorActive(surfaceDetectorActive),
        finder_(new SurfaceFinder<PointT>(surfaceDetectorActive)),
        clusterer_(new SurfaceClusterer<PointT>()) {}

    /**
     * FrameDataObserver interface method implementation.
     */
    virtual void updateFrame(FrameDataPtr frameData);

  private:
    boost::shared_ptr<SurfaceFinder<PointT> > finder_;
    boost::shared_ptr<SurfaceClusterer<PointT> > clusterer_;
    bool surfaceDetectorActive;

};


template<class PointT>
void SurfaceDetector<PointT>::updateFrame(FrameDataPtr frameData) 
{
  std::vector<PointCloudPtr> planes;
  std::vector<pcl::ModelCoefficients> planeCoefficients;
  // detect planes
  finder_->findSurfaces(frameData, planes, planeCoefficients);

  // cluster planes and create surface models
  clusterer_->clusterSurfaces(planes, planeCoefficients, frameData->surfaces);

  notifyObservers(frameData);
}

}
#endif