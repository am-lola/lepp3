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
#include "lepp3/SurfaceAggregator.hpp"
#include "lepp3/ObjectApproximator.hpp"
#include "lepp3/MomentOfInertiaApproximator.hpp"
#include "lepp3/SplitApproximator.hpp"
#include "lepp3/SurfaceApproximator.hpp"
#include "lepp3/SurfaceFeatureEstimator.hpp"

using namespace lepp;

#include "lepp3/debug/timer.hpp"

int FRAME_COUNT = 0;


template<class PointT>
class SurfaceDetector : public lepp::VideoObserver<PointT> {

 public:
    SurfaceDetector();
    virtual ~SurfaceDetector() {}

    /**
     * VideoObserver interface method implementation.
     */
    virtual void notifyNewFrame(
        int idx,
        const PointCloudConstPtr& point_cloud);
    /**
       * Attaches a new SurfaceAggregator, which will be notified of newly detected
       * surfaces by this detector.
       */
    void attachSurfaceAggregator(boost::shared_ptr<SurfaceAggregator<PointT> > aggregator);

  protected:
    /**
     * Notifies any observers about newly detected surfaces.
     */
    void notifySurfaces(std::vector<SurfaceModelPtr> const& surfaces,
      PointCloudPtr &cloudMinusSurfaces, std::vector<pcl::ModelCoefficients> &surfaceCoefficients);

  private:
    PointCloudConstPtr cloud_;

    /**
     * Tracks all attached SurfaceAggregators that wish to be notified of newly
     * detected surfaces.
     */
    std::vector<boost::shared_ptr<SurfaceAggregator<PointT> > > aggregators;

    boost::shared_ptr<BaseSegmenter<PointT> > segmenter_;
    boost::shared_ptr<SurfaceApproximator<PointT> > approximator_;

    /**
     * Performs a new update of the surface approximations.
     * Triggered when the detector is notified of a new frame (i.e. point cloud).
     */
    void update();
};

template<class PointT>
SurfaceDetector<PointT>::SurfaceDetector()
    : approximator_(
      boost::shared_ptr<SurfaceApproximator<PointT> >(
        new SurfaceFeatureEstimator<PointT>)),
      segmenter_(new SurfaceSegmenter<PointT>()) {
}



template<class PointT>
void SurfaceDetector<PointT>::notifyNewFrame(int id,
    const PointCloudConstPtr& point_cloud) {
  
  FRAME_COUNT++;
  cout << "FRAME " << FRAME_COUNT << endl;
  //if (FRAME_COUNT == 10)
  //  exit(0);
  
  cloud_ = point_cloud;
  try 
  {
    update();
  } catch (...) 
  {
   // std::cerr << "SurfaceDetector: Surface detection failed ..." << std::endl;
  }
}

template<class PointT>
void SurfaceDetector<PointT>::update() {

  PointCloudPtr cloudMinusSurfaces(new PointCloudT());
  std::vector<PointCloudConstPtr> surfaces;
  std::vector<pcl::ModelCoefficients> surfaceCoefficients;
  segmenter_->segment(cloud_, surfaces, cloudMinusSurfaces, surfaceCoefficients);

  // create surface models of out surfaces and their coefficients
  std::vector<SurfaceModelPtr> surfaceModels;
  for(size_t i = 0; i < surfaces.size(); i++)
    surfaceModels.push_back(approximator_->approximate(surfaces[i],surfaceCoefficients.at(i)));

  notifySurfaces(surfaceModels, cloudMinusSurfaces, surfaceCoefficients);
}

template<class PointT>
void SurfaceDetector<PointT>::attachSurfaceAggregator(
    boost::shared_ptr<SurfaceAggregator<PointT> > aggregator) {
  aggregators.push_back(aggregator);
}

template<class PointT>
void SurfaceDetector<PointT>::notifySurfaces(std::vector<SurfaceModelPtr> const& surfaces,
  PointCloudPtr &cloudMinusSurfaces, std::vector<pcl::ModelCoefficients> &surfaceCoefficients) {
  size_t sz = aggregators.size();
  for (size_t i = 0; i < sz; ++i) {
    aggregators[i]->updateSurfaces(surfaces, cloudMinusSurfaces, surfaceCoefficients);
  }
}

#endif
