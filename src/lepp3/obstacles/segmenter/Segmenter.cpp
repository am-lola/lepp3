#include "Segmenter.hpp"

#ifdef LEPP3_ENABLE_TRACING
#include "lepp3/util/lepp3_tracepoint_provider.hpp"
#endif

void lepp::ObstacleSegmenter::updateFrame(FrameDataPtr frameData) {
  if (0 < frameData->cloudMinusSurfaces->size()) {
#ifdef LEPP3_ENABLE_TRACING
    tracepoint(lepp3_trace_provider, obstacle_segmenter_start);
#endif

    frameData->obstacleParams = extractObstacleParams(frameData->cloudMinusSurfaces);

#ifdef LEPP3_ENABLE_TRACING
    tracepoint(lepp3_trace_provider, obstacle_segmenter_end);
#endif
  }
  notifyObservers(frameData);
}
