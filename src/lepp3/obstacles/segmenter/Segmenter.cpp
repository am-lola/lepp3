#include "Segmenter.hpp"

void lepp::ObstacleSegmenter::updateFrame(FrameDataPtr frameData) {
  if (0 < frameData->cloudMinusSurfaces->size()) {
    frameData->obstacleClouds = extractObstacleClouds(frameData->cloudMinusSurfaces);
  }
  notifyObservers(frameData);
}
