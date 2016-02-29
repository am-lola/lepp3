#ifndef BASE_SURFACE_DETECTOR_H_
#define BASE_SURFACE_DETECTOR_H_

#include <vector>
#include <algorithm>

#include <pcl/visualization/cloud_viewer.h>

#include "lepp3/Typedefs.hpp"
#include "lepp3/VideoObserver.hpp"
#include "lepp3/BaseSegmenter.hpp"
#include "lepp3/NoopSegmenter.hpp"
#include "lepp3/SurfaceSegmenter.hpp"
#include "lepp3/ObjectApproximator.hpp"
#include "lepp3/MomentOfInertiaApproximator.hpp"
#include "lepp3/SplitApproximator.hpp"
#include "lepp3/SurfaceApproximator.hpp"
#include "lepp3/SurfaceFeatureEstimator.hpp"
#include "lepp3/FrameDataObserver.hpp"

using namespace lepp;

#include "lepp3/debug/timer.hpp"

int FRAME_COUNT = 0;


template<class PointT>
class SurfaceDetector : public FrameDataObserver {

 public:
    SurfaceDetector();
    virtual ~SurfaceDetector() {}

    /**
     * VideoObserver interface method implementation.
     */
    virtual void updateFrame(boost::shared_ptr<FrameData> frameData);
    /**
       * Attaches a new SurfaceAggregator, which will be notified of newly detected
       * surfaces by this detector.
       */
    void attachObserver(boost::shared_ptr<FrameDataObserver> observer);

  protected:
    /**
     * Notifies any observers about newly detected surfaces.
     */
    void notifyObservers(boost::shared_ptr<FrameData> frameData);

  private:
    PointCloudConstPtr cloud_;

    /**
     * Tracks all attached SurfaceAggregators that wish to be notified of newly
     * detected surfaces.
     */
    std::vector<boost::shared_ptr<FrameDataObserver> > observers;

    boost::shared_ptr<BaseSegmenter<PointT> > segmenter_;
    boost::shared_ptr<SurfaceApproximator<PointT> > approximator_;

};

template<class PointT>
SurfaceDetector<PointT>::SurfaceDetector()
    : approximator_(
      boost::shared_ptr<SurfaceApproximator<PointT> >(
        new SurfaceFeatureEstimator<PointT>)),
      segmenter_(new SurfaceSegmenter<PointT>()) {
}


template<class PointT>
void SurfaceDetector<PointT>::updateFrame(boost::shared_ptr<FrameData> frameData) {

  cout << "frame " << ++FRAME_COUNT << endl;

  frameData->cloudMinusSurfaces = PointCloudPtr(new PointCloudT());
  std::vector<PointCloudConstPtr> surfaces;
  segmenter_->segment(frameData->cloud, surfaces, frameData->cloudMinusSurfaces, frameData->surfaceCoefficients);

  // create surface models of out surfaces and their coefficients
  for(size_t i = 0; i < surfaces.size(); i++)
    frameData->surfaces.push_back(approximator_->approximate(surfaces[i],frameData->surfaceCoefficients.at(i)));

  notifyObservers(frameData);
}

template<class PointT>
void SurfaceDetector<PointT>::attachObserver(
    boost::shared_ptr<FrameDataObserver> observer) {
  observers.push_back(observer);
}

template<class PointT>
void SurfaceDetector<PointT>::notifyObservers(boost::shared_ptr<FrameData> frameData) {
  size_t sz = observers.size();
  for (size_t i = 0; i < sz; ++i) {
    observers[i]->updateFrame(frameData);
  }
}

#endif
