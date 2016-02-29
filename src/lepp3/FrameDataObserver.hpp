#ifndef FRAME_DATA_OBSERVER_H_
#define FRAME_DATA_OBSERVER_H_

#include "lepp3/FrameData.hpp"

namespace lepp {

class FrameDataObserver {
public:
  virtual void updateFrame(boost::shared_ptr<FrameData> frameData) = 0;
};

}  // namespace lepp

#endif