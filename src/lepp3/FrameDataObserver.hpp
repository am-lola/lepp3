#ifndef FRAME_DATA_OBSERVER_H_
#define FRAME_DATA_OBSERVER_H_

#include "lepp3/FrameData.hpp"

namespace lepp {

class FrameDataObserver {
public:
  virtual void updateFrame(FrameDataPtr frameData) = 0;
};

}  // namespace lepp

#endif