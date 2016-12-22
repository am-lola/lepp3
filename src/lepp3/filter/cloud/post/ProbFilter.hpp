#ifndef LEPP3_FILTER_CLOUD_POST_PROB_FILTER_H__
#define LEPP3_FILTER_CLOUD_POST_PROB_FILTER_H__

#include "lepp3/filter/cloud/post/CloudPostFilter.hpp"
#include <boost/unordered_set.hpp>
#include <boost/unordered_map.hpp>

namespace lepp {

/**
 * An implementation of a `CloudPostFilter` where points are included only
 * if they have been seen in a certain percentage of frames of the last N
 * frames.
 */
template<class PointT>
class ProbFilter : public CloudPostFilter<PointT> {
  using MapPoint = lepp::cloud_post_filter::MapPoint;

public:
  ProbFilter(bool larger_voxelization = false)
    : larger_voxelization_(larger_voxelization) {}

  virtual void newFrame() override;
  virtual void newPoint(PointT& p, PointCloudT& filtered) override;
  virtual void getFiltered(PointCloudT& filtered) override;

private:
  boost::unordered_set<MapPoint> this_frame_;
  boost::unordered_map<MapPoint, uint32_t> all_points_;
  MapPoint min_pt;
  MapPoint max_pt;

  bool const larger_voxelization_;
};
}

template<class PointT>
void lepp::ProbFilter<PointT>::newFrame() {
  this_frame_.clear();
  min_pt.x = std::numeric_limits<int>::max();
  min_pt.y = std::numeric_limits<int>::max();
  min_pt.z = std::numeric_limits<int>::max();
  max_pt.x = std::numeric_limits<int>::min();
  max_pt.y = std::numeric_limits<int>::min();
  max_pt.z = std::numeric_limits<int>::min();
}

template<class PointT>
void lepp::ProbFilter<PointT>::newPoint(PointT& p, PointCloudT& filtered) {
  MapPoint map_point = MapPoint(p.x * 100, p.y * 100, p.z * 100);

  if (larger_voxelization_) {
    int const mask = 0xFFFFFFFE;
    map_point.x &= mask;
    map_point.y &= mask;
    map_point.z &= mask;
  }

  uint32_t& ref = all_points_[map_point];
  ref <<= 1;
  ref |= 1;
  this_frame_.insert(map_point);
  // Keep track of the min/max points so that we know the bounding box of
  // the current cloud.
  min_pt.x = std::min(min_pt.x, map_point.x);
  min_pt.y = std::min(min_pt.y, map_point.y);
  min_pt.z = std::min(min_pt.z, map_point.z);
  max_pt.x = std::max(max_pt.x, map_point.x);
  max_pt.y = std::max(max_pt.y, map_point.y);
  max_pt.z = std::max(max_pt.z, map_point.z);
}

template<class PointT>
void lepp::ProbFilter<PointT>::getFiltered(PointCloudT& filtered) {
  boost::unordered_map<MapPoint, uint32_t>::iterator it =
      all_points_.begin();
  while (it != all_points_.end()) {
    if (this_frame_.find(it->first) == this_frame_.end()) {
      it->second <<= 1;
    }
    int count = __builtin_popcount(it->second);
    if (count >= 10) {
      PointT pt;
      pt.x = it->first.x / 100.;
      pt.y = it->first.y / 100.;
      pt.z = it->first.z / 100.;
      filtered.push_back(pt);
    }

    // Allow for a bit of leeway with removing points at the very boundary of
    // the bounding box. To do this, the bounding box is increased by 10 [cm]
    // in each direction. Only points within *this* bounding box are kept in
    // the map -- all others removed.
    // TODO See if tweaking these values up/down yields any benefits.
    min_pt.x -= 10;
    min_pt.y -= 10;
    min_pt.z -= 10;
    max_pt.x += 10;
    max_pt.y += 10;
    max_pt.z += 10;
    MapPoint const& pt = it->first;
    if (pt.x < min_pt.x || pt.x > max_pt.x ||
        pt.y < min_pt.y || pt.y > max_pt.y ||
        pt.z < min_pt.z || pt.z > max_pt.z) {
      all_points_.erase(it++);
    } else {
      ++it;
    }
  }
}

#endif
