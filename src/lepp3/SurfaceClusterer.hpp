#ifndef lepp3_SURFACE_CLUSTERER_HPP__
#define lepp3_SURFACE_CLUSTERER_HPP__

#include <unordered_map>

#include "lepp3/Typedefs.hpp"
#include "lepp3/SurfaceData.hpp"
#include "lepp3/util/Projection.h"
#include "lepp3/util/VoxelGrid.h"

#include <pcl/filters/project_inliers.h>

#include <omp.h>

#ifdef LEPP3_ENABLE_TRACING
#include "lepp3/util/lepp3_tracepoint_provider.hpp"
#endif

namespace lepp {

template<class PointT>
class SurfaceClusterer : public SurfaceDataObserver, public SurfaceDataSubject {
public:
  struct Parameters {
    // constant variables for clustering
    double CLUSTER_TOLERANCE;
    int MIN_CLUSTER_SIZE;
  };

  SurfaceClusterer(Parameters const& params)
      : CLUSTER_TOLERANCE(params.CLUSTER_TOLERANCE),
        MIN_CLUSTER_SIZE(params.MIN_CLUSTER_SIZE) {}

  /**
  * Cluster the given surfaces into planes. Store the found surfaces and surface model
  * coefficients in 'surfaces' and 'surfaceCoefficients'. Subtract the found surfaces
  * from the input cloud and store the remaining cloud in 'cloudMinusSurfaces'.
  */
  virtual void updateSurfaces(SurfaceDataPtr surfaceData);

private:
  /**
  * A plane might contain several non-connected planes that do not correspond
  * to the same surface. Thus, the planes are clustered into seperate surfaces
  * in this function if necessary.
  **/
  void cluster(
      PointCloudPtr plane,
      pcl::ModelCoefficients& planeCoefficients,
      std::vector<SurfaceModelPtr>& surfaces);

  /**
  * Project given surfaces on plane specified by the corresponding plane coefficients.
  */
  void projectOnPlane(SurfaceModelPtr surface);

  /**
   * Projects a 3d plane into 2D for faster processing
   * Note: The function returns a 3D vector with z set to 1
   */
  std::vector<Eigen::Vector3f> project_to_2d_plane(const PointCloudT& cloud, const lepp::util::Projection& proj) const;

  // constant variables for clustering
  const double CLUSTER_TOLERANCE;
  const int MIN_CLUSTER_SIZE;
};

template<class PointT>
void SurfaceClusterer<PointT>::projectOnPlane(SurfaceModelPtr surface) {
  // project surface on corresponding plane
  pcl::ProjectInliers<PointT> proj;
  proj.setModelType(pcl::SACMODEL_PLANE);
  proj.setInputCloud(surface->get_cloud());
  pcl::ModelCoefficients::Ptr coeffPtr = boost::shared_ptr<pcl::ModelCoefficients>(
      new pcl::ModelCoefficients(surface->get_planeCoefficients()));
  proj.setModelCoefficients(coeffPtr);
  PointCloudPtr tmp(new PointCloudT());
  proj.filter(*tmp);
  surface->set_cloud(tmp);
}

template<class PointT>
std::vector<Eigen::Vector3f>
SurfaceClusterer<PointT>::project_to_2d_plane(const PointCloudT& cloud, const lepp::util::Projection& proj) const {
  std::vector<Eigen::Vector3f> result;
  result.reserve(cloud.points.size());

  for (const auto& pt : cloud.points) {
    auto pt_2d = proj(pt.x, pt.y, pt.z);
    result.emplace_back(pt_2d[0], pt_2d[1], 1.0f);
  }

  return result;
}

template<class PointT>
void SurfaceClusterer<PointT>::cluster(
    PointCloudPtr plane,
    pcl::ModelCoefficients& planeCoefficients,
    std::vector<SurfaceModelPtr>& surfaces) {

#ifdef LEPP3_ENABLE_TRACING
  tracepoint(lepp3_trace_provider, surface_cluster_start);
#endif

  lepp::util::Projection proj(planeCoefficients.values);
  auto plane_2d = project_to_2d_plane(*plane, proj);

  lepp::util::VoxelGrid<2> voxelGrid(CLUSTER_TOLERANCE);
  voxelGrid.build(plane_2d);

  std::unordered_map<size_t, PointCloudT> clusters;

  for (size_t i = 0; i < plane_2d.size(); ++i) {
    size_t cluster = voxelGrid.clusterForPoint(plane_2d[i]);
    clusters[cluster].points.emplace_back(plane->points[i]);
  }

  // cluster the current plane into seperate surfaces
  std::vector<SurfaceModelPtr> clusteredSurfaces;

  for (auto& entry : clusters) {
    auto& cloud = entry.second;

    if (cloud.points.size() < MIN_CLUSTER_SIZE)
      continue;

    cloud.is_dense = true;
    cloud.width = cloud.points.size();
    cloud.height = 1;

    clusteredSurfaces.push_back(
        SurfaceModelPtr(new SurfaceModel(boost::make_shared<PointCloudT>(cloud), planeCoefficients)));

    // project the found surfaces on corresponding plane
    projectOnPlane(clusteredSurfaces.back());
  }

  //add clusetered surfaces to shared frameData variable
#pragma omp critical
  surfaces.insert(std::end(surfaces), std::begin(clusteredSurfaces), std::end(clusteredSurfaces));

#ifdef LEPP3_ENABLE_TRACING
  tracepoint(lepp3_trace_provider, surface_cluster_end);
#endif
}

template<class PointT>
void SurfaceClusterer<PointT>::updateSurfaces(SurfaceDataPtr surfaceData) {
#pragma omp parallel for schedule(dynamic, 1)
  for (size_t i = 0; i < surfaceData->planes.size(); i++) {
    // cluster planes into seperate surfaces and create SurfaceModels
    cluster(surfaceData->planes[i], surfaceData->planeCoefficients[i], surfaceData->surfaces);
  }
  notifyObservers(surfaceData);
}

} // namespace lepp

#endif
