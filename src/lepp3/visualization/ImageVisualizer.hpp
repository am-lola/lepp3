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
      camera_matrix[0][0] = 5.2921508098293293e+02;    camera_matrix[0][1] = 0.0;                       camera_matrix[0][2] = 3.2894272028759258e+02;
      camera_matrix[1][0] = 0.0;                       camera_matrix[1][1] = 5.2556393630057437e+02;    camera_matrix[1][2] = 2.6748068171871557e+02;
      camera_matrix[2][0] = 0.0;                       camera_matrix[2][1] = 0.0;                       camera_matrix[2][2] = 1.0;
      viz.Start("Lola Listener");
      viz.SetCameraIntrinsics(camera_matrix);
    }

    /**
 * Destructor
 */
    ~ImageVisualizer()
    {
      viz.Stop();
    }

    /**
 * FrameDataObserver interface implementation: processes the current point cloud.
 */
    void updateFrame(FrameDataPtr frameData);

private:

    ar::ARVisualizer viz;
    double camera_matrix[3][3];



  };

  void ImageVisualizer::updateFrame(FrameDataPtr frameData) {
    return;
//    viz.NotifyNewVideoFrame(width, height, pixels);
  };

}// namespace lepp

#endif // LEPP3_VISUALIZATION_CALIBRATOR_VISUALIZER_H__
