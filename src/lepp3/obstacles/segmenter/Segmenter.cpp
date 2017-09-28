#include "Segmenter.hpp"

void lepp::ObstacleSegmenter::updateFrame(FrameDataPtr frameData) {
  if (0 < frameData->cloudMinusSurfaces->size()) {
    frameData->obstacleParams = extractObstacleParams(frameData->cloudMinusSurfaces);
  }
  notifyObservers(frameData);
}
