#ifndef LEPP3_FILTER_POINT_FILTER_H__
#define LEPP3_FILTER_POINT_FILTER_H__

#include <string>
#include <vector>

namespace lepp {

template<class PointT>
class PointFilter {
public:
  virtual bool apply(PointT& pt) = 0;
  virtual void prepareNext() = 0;

  /**
   * @brief Defines an order value for a filter
   *
   * Lower orders will be applied first
   */
  virtual int order() const { return 0; }
  /**
   * @brief Unique identifier for this filter
   *
   * This is required tio check for dependencies
   */
  virtual const char* name() const = 0;

  /**
   * @brief This returns a list of dependencies to check for before adding this filter
   *
   * Make sure dependencies always have a lower order than the filter itself
   */
  virtual std::vector<std::string> dependencies() const { return {}; }
};

}  // namespace lepp
#endif
