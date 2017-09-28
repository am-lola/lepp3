#ifndef LEPP3_FILTER_CLOUD_POST_PT1_FILTER_H__
#define LEPP3_FILTER_CLOUD_POST_PT1_FILTER_H__

#include "lepp3/filter/cloud/post/CloudPostFilter.hpp"
#include <boost/unordered_set.hpp>
#include <boost/unordered_map.hpp>

namespace lepp {

/**
 * An implementation of a `CloudFilter` where points are included only
 * if they have been seen in a certain percentage of frames of the last N
 * frames.
 */
template<class PointT>
class Pt1Filter : public CloudPostFilter<PointT> {
  using MapPoint = lepp::cloud_post_filter::MapPoint;

public:
  virtual void newFrame() override;
  virtual void newPoint(PointT& p, PointCloudT& filtered) override;
  virtual void getFiltered(PointCloudT& filtered) override;

private:
  boost::unordered_set<MapPoint> this_frame_;
  boost::unordered_map<MapPoint, float> all_points_;
};
}

template<class PointT>
void lepp::Pt1Filter<PointT>::newFrame() {
  this_frame_.clear();
}

template<class PointT>
void lepp::Pt1Filter<PointT>::newPoint(PointT& p, PointCloudT& filtered) {
  MapPoint map_point = MapPoint(p.x * 100, p.y * 100, p.z * 100);
  float& ref = all_points_[map_point];
  // TODO Factor out the parameter value to a member constant.
  ref = 0.9*ref + 0.1*10;
  this_frame_.insert(map_point);
}

template<class PointT>
void lepp::Pt1Filter<PointT>::getFiltered(PointCloudT& filtered) {
  boost::unordered_map<MapPoint, float>::iterator it =
      all_points_.begin();
  while (it != all_points_.end()) {
    if (this_frame_.find(it->first) == this_frame_.end()) {
      it->second = 0.9*it->second + 0.1*0.;
    }
    if (it->second >= 4.) {
      PointT pt;
      pt.x = it->first.x / 100.;
      pt.y = it->first.y / 100.;
      pt.z = it->first.z / 100.;
      filtered.push_back(pt);
    }
    ++it;
  }
}

#endif
