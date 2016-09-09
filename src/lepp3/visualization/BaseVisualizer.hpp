#ifndef LEPP3_VISUALIZATION_BASE_VISUALIZER_H__
#define LEPP3_VISUALIZATION_BASE_VISUALIZER_H__

#include <am2b-arvis/ARVisualizer.hpp>

#include "lepp3/FrameData.hpp"

namespace lepp {

/**
 * A base class for visualizers
 */
class BaseVisualizer : public FrameDataObserver {

public:
  /**
   * The ctor only takes care of the basic initialization of `arvis_`. The rest
   * is done in any inherited class.
   */
  BaseVisualizer(
    char const* name = "lepp3", int const& width = 1024, int const& height = 768)
      : arvis_(new ar::ARVisualizer()) {

      arvis_->Start(name, width, height);
  }

  /**
   * There's no need to include the pure virtual function from the
   * `FrameDataObserver` class, as `BaseVisualizer` is also an abstract class
   * and it will not be implemented here.
   */

  ~BaseVisualizer() {
    arvis_->Stop();
  }

protected:
  /**
   * Instance holding an ARVisualizer object
   */
  std::shared_ptr<ar::ARVisualizer> arvis_;
};

}

#endif // LEPP3_VISUALIZATION_BASE_VISUALIZER_H__
