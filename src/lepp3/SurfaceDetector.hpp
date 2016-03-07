#ifndef BASE_SURFACE_DETECTOR_H_
#define BASE_SURFACE_DETECTOR_H_

#include "lepp3/Typedefs.hpp"
#include "lepp3/BaseSegmenter.hpp"
#include "lepp3/SurfaceSegmenter.hpp"
#include "lepp3/FrameData.hpp"

namespace lepp {

template<class PointT>
class SurfaceDetector : public FrameDataObserver, public FrameDataSubject 
{
 public:
    SurfaceDetector();
    virtual ~SurfaceDetector() {}

    /**
     * VideoObserver interface method implementation.
     */
    virtual void updateFrame(FrameDataPtr frameData);

  private:
    boost::shared_ptr<BaseSegmenter<PointT> > segmenter_;
};

template<class PointT>
SurfaceDetector<PointT>::SurfaceDetector()
    : segmenter_(new SurfaceSegmenter<PointT>()) {}


template<class PointT>
void SurfaceDetector<PointT>::updateFrame(FrameDataPtr frameData) 
{
  segmenter_->segment(frameData);
  notifyObservers(frameData);
}

}
#endif