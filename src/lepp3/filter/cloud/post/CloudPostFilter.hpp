#ifndef LEPP3_FILTER_CLOUD_POST_CLOUD_POST_FILTER_H__
#define LEPP3_FILTER_CLOUD_POST_CLOUD_POST_FILTER_H__

namespace lepp {

namespace cloud_post_filter {
/**
 * A struct that is used to describe a single point in space that can be used
 * to index sets and maps of such points.
 */
struct MapPoint {
  int x;
  int y;
  int z;
  MapPoint(int x, int y, int z) : x(x), y(y), z(z) {}
  MapPoint() {}
};

/**
 * Structs that are to be placed in associative containers must be comparable.
 */
bool operator==(MapPoint const& lhs, MapPoint const& rhs) {
  return lhs.x == rhs.x && lhs.y == rhs.y && lhs.z == rhs.z;
}

/**
 * Provides a hash value for a `MapPoint` in order to allow for it to be
 * placed in boost's map and set.
 */
size_t hash_value(MapPoint const& pt) {
  std::size_t seed = 0;
  boost::hash_combine(seed, pt.x);
  boost::hash_combine(seed, pt.y);
  boost::hash_combine(seed, pt.z);
  return seed;
}
}

template<class PointT>
class CloudPostFilter {
public:
  virtual void newFrame() = 0;
  virtual void newPoint(PointT& p, PointCloudT& filtered) = 0;
  virtual void getFiltered(PointCloudT& filtered) = 0;
};

}

#endif
