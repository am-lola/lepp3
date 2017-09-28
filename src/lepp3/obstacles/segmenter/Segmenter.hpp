#ifndef LEPP_OBSTACLES_SEGMENTER_SEGMENTER_H
#define LEPP_OBSTACLES_SEGMENTER_SEGMENTER_H

#include "lepp3/FrameData.hpp"

namespace lepp {

class ObstacleSegmenter : public FrameDataObserver, public FrameDataSubject {
public:
  virtual void updateFrame(FrameDataPtr frameData) override;

private:
  virtual std::vector<ObjectModelParams> extractObstacleParams(PointCloudConstPtr cloud) = 0;
};

}

#endif
