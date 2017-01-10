#ifndef LEPP3_VISUALIZATION_IMAGE_VISUALIZER_H__
#define LEPP3_VISUALIZATION_IMAGE_VISUALIZER_H__

#include <sstream>
#include "lepp3/FrameData.hpp"
#include "lepp3/Typedefs.hpp"
#include "lepp3/visualization/BaseVisualizer.hpp"

#include <iostream>

namespace lepp {

  class ImageVisualizer : public BaseVisualizer {

  public:
    ImageVisualizer()
        {
          a =0;
    }

    /**
 * Destructor
 */
    ~ImageVisualizer() { }

    /**
 * FrameDataObserver interface implementation: processes the current point cloud.
 */
    void updateFrame(FrameDataPtr frameData);

private:
  int a;

  };

  void ImageVisualizer::updateFrame(FrameDataPtr frameData) {
    return;
  };

}// namespace lepp

#endif // LEPP3_VISUALIZATION_CALIBRATOR_VISUALIZER_H__
