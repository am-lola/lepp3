#ifndef lepp3_SURFACE_CLUSTERER_HPP__
#define lepp3_SURFACE_CLUSTERER_HPP__

#include <chrono>
#include <unordered_map>

#include "lepp3/Typedefs.hpp"
#include "lepp3/SurfaceData.hpp"
#include "lepp3/util/Projection.h"

#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/project_inliers.h>

#include <omp.h>

namespace lepp {

template<class PointT>
class SurfaceClusterer : public SurfaceDataObserver, public SurfaceDataSubject {
public:
  struct Parameters {
    // constant variables for clustering
    double CLUSTER_TOLERANCE;
    int MIN_CLUSTER_SIZE;

    // side length of a voxel when applying voxelgridfilter
    double COARSE_VOXEL_SIZE_X;
    double COARSE_VOXEL_SIZE_Y;
    double COARSE_VOXEL_SIZE_Z;
    double FINE_VOXEL_SIZE_X;
    double FINE_VOXEL_SIZE_Y;
    double FINE_VOXEL_SIZE_Z;
    int FINE_COARSE_LIMIT;
  };

  SurfaceClusterer(Parameters const& params)
      : CLUSTER_TOLERANCE(params.CLUSTER_TOLERANCE),
        MIN_CLUSTER_SIZE(params.MIN_CLUSTER_SIZE),
        COARSE_VOXEL_SIZE_X(params.COARSE_VOXEL_SIZE_X),
        COARSE_VOXEL_SIZE_Y(params.COARSE_VOXEL_SIZE_Y),
        COARSE_VOXEL_SIZE_Z(params.COARSE_VOXEL_SIZE_Z),
        FINE_VOXEL_SIZE_X(params.FINE_VOXEL_SIZE_X),
        FINE_VOXEL_SIZE_Y(params.FINE_VOXEL_SIZE_Y),
        FINE_VOXEL_SIZE_Z(params.FINE_VOXEL_SIZE_Z),
        FINE_COARSE_LIMIT(params.FINE_COARSE_LIMIT) {}

  /**
  * Cluster the given surfaces into planes. Store the found surfaces and surface model
  * coefficients in 'surfaces' and 'surfaceCoefficients'. Subtract the found surfaces
  * from the input cloud and store the remaining cloud in 'cloudMinusSurfaces'.
  */
  virtual void updateSurfaces(SurfaceDataPtr surfaceData);

private:
  virtual void updateSurfacesNew(SurfaceDataPtr surfaceData);

  virtual void updateSurfacesOld(SurfaceDataPtr surfaceData);

  /**
  * Extracts the Euclidean clusters from the given point cloud.
  * Returns a vector where each element represents the pcl::PointIndices
  * instance representing the corresponding cluster.
  */
  void getSurfaceClusters(PointCloudPtr const& cloud,
                          std::vector<pcl::PointIndices>& cluster_indices);

  /**
  * A plane might contain several non-connected planes that do not correspond
  * to the same surface. Thus, the planes are clustered into seperate surfaces
  * in this function if necessary.
  **/
  void cluster(
      PointCloudPtr plane,
      pcl::ModelCoefficients& planeCoefficients,
      std::vector<SurfaceModelPtr>& surfaces);
  void clusterNew(
      PointCloudPtr plane,
      pcl::ModelCoefficients& planeCoefficients,
      std::vector<SurfaceModelPtr>& surfaces);

  /**
    * Downsample point cloud. Reduce the size of the given point cloud.
    * This function is called after the segmentation/classifying step but
    * before clustering.
    * This step is NOT done before segmentation/classify because cloudMinusSurfaces
    * should have a resolution as high as possible since this cloud is used to
    * detect obstacles.
    * Clustering surfaces is found to be quite inefficient though but since we
    * obtain a convex hull for each surface later on anyway, downsampling the point
    * size does not influence the quality of the surface clustering.
    */
  void downSample(PointCloudPtr& cloud, const double voxelSizeX,
                  const double voxelSizeY, const double voxelSizeZ);

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

  // side length of a voxel when applying voxelgridfilter
  const double COARSE_VOXEL_SIZE_X;
  const double COARSE_VOXEL_SIZE_Y;
  const double COARSE_VOXEL_SIZE_Z;
  const double FINE_VOXEL_SIZE_X;
  const double FINE_VOXEL_SIZE_Y;
  const double FINE_VOXEL_SIZE_Z;
  const int FINE_COARSE_LIMIT;
};


template<class PointT>
void SurfaceClusterer<PointT>::downSample(PointCloudPtr& cloud,
                                          const double voxelSizeX, const double voxelSizeY, const double voxelSizeZ) {
  PointCloudPtr cloud_filtered(new PointCloudT());
  pcl::VoxelGrid<PointT> sor;
  sor.setInputCloud(cloud);
  sor.setLeafSize(voxelSizeX, voxelSizeY, voxelSizeZ);
  sor.filter(*cloud_filtered);
  cloud = cloud_filtered;
}


template<class PointT>
void SurfaceClusterer<PointT>::getSurfaceClusters(
    PointCloudPtr const& cloud, std::vector<pcl::PointIndices>& cluster_indices) {
  // Extract the clusters from such a filtered cloud.
  int max_size = cloud->points.size();
  pcl::EuclideanClusterExtraction<PointT> clusterizer;
  clusterizer.setClusterTolerance(CLUSTER_TOLERANCE);
  clusterizer.setMinClusterSize(MIN_CLUSTER_SIZE);
  clusterizer.setMaxClusterSize(max_size);
  pcl::search::KdTree<pcl::PointXYZ>::Ptr kd_tree_(
      new pcl::search::KdTree<pcl::PointXYZ>);
  kd_tree_->setInputCloud(cloud);
  clusterizer.setSearchMethod(kd_tree_);
  clusterizer.setInputCloud(cloud);
  clusterizer.extract(cluster_indices);
}

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
void SurfaceClusterer<PointT>::cluster(
    PointCloudPtr plane,
    pcl::ModelCoefficients& planeCoefficients,
    std::vector<SurfaceModelPtr>& surfaces) {
  // A classified surface may consist of different clusters.
  // Get point indices of points belonging to the same cluster.
  std::vector<pcl::PointIndices> cluster_indices;
  getSurfaceClusters(plane, cluster_indices);

  // cluster the current plane into seperate surfaces
  std::vector<SurfaceModelPtr> clusteredSurfaces;
  for (int j = 0; j < cluster_indices.size(); j++) {
    // extract all points from the classified surface that
    // belong to the same clauster
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(plane);
    // TODO: Optimization. ClusterIndices are copied beacuse
    // extract expects a PointIndices::Ptr when calling setIndices.
    pcl::PointIndices::Ptr currentClusterIndices(
        new pcl::PointIndices(cluster_indices[j]));
    extract.setIndices(currentClusterIndices);
    extract.setNegative(false);
    PointCloudPtr current(new PointCloudT());
    extract.filter(*current);

    // create new surface model for current plane
    clusteredSurfaces.push_back(SurfaceModelPtr(new SurfaceModel(current, planeCoefficients)));

    // project the found surfaces on corresponding plane
    projectOnPlane(clusteredSurfaces[j]);
  }

  //add clusetered surfaces to shared frameData variable
#pragma omp critical
  surfaces.insert(std::end(surfaces), std::begin(clusteredSurfaces), std::end(clusteredSurfaces));
}

template<class PointT>
std::vector<Eigen::Vector3f> SurfaceClusterer<PointT>::project_to_2d_plane(const PointCloudT& cloud, const lepp::util::Projection& proj) const
{
  std::vector<Eigen::Vector3f> result;
  result.reserve(cloud.points.size());

  for (const auto& pt : cloud.points)
  {
    auto pt_2d = proj(pt.x, pt.y, pt.z);
    result.emplace_back(pt_2d[0], pt_2d[1], 1.0f);
  }

  return result;
}

template<class PointT>
void SurfaceClusterer<PointT>::clusterNew(
    PointCloudPtr plane,
    pcl::ModelCoefficients& planeCoefficients,
    std::vector<SurfaceModelPtr>& surfaces) {
  lepp::util::Projection proj(planeCoefficients.values);
  auto plane_2d = project_to_2d_plane(*plane, proj);

  lepp::util::VoxelGrid<2> voxelGrid(CLUSTER_TOLERANCE);
  voxelGrid.build(plane_2d);

  std::unordered_map<size_t, PointCloudT> clusters;

  for (size_t i = 0; i < plane_2d.size(); ++i)
  {
    size_t cluster = voxelGrid.clusterForPoint(plane_2d[i]);

    if (cluster >= lepp::util::VoxelGrid<2>::CLUSTERED_CELL_START)
    {
      clusters[cluster].points.emplace_back(plane->points[i]);
    }
  }

  // cluster the current plane into seperate surfaces
  std::vector<SurfaceModelPtr> clusteredSurfaces;

  for (auto& entry : clusters)
  {
    auto& cloud = entry.second;

    if (cloud.points.size() < MIN_CLUSTER_SIZE)
      continue;

    cloud.is_dense = true;
    cloud.width = cloud.points.size();
    cloud.height = 1;

    clusteredSurfaces.push_back(SurfaceModelPtr(new SurfaceModel(boost::make_shared<PointCloudT>(cloud), planeCoefficients)));

    // project the found surfaces on corresponding plane
    projectOnPlane(clusteredSurfaces.back());
  }

  //add clusetered surfaces to shared frameData variable
#pragma omp critical
  surfaces.insert(std::end(surfaces), std::begin(clusteredSurfaces), std::end(clusteredSurfaces));
}

template<class PointT>
void SurfaceClusterer<PointT>::updateSurfacesOld(SurfaceDataPtr surfaceData) {
  #pragma omp parallel for schedule(dynamic, 1)
  for (size_t i = 0; i < surfaceData->planes.size(); i++) {
    if (surfaceData->planes[i]->size() > FINE_COARSE_LIMIT)
      downSample(surfaceData->planes[i], COARSE_VOXEL_SIZE_X, COARSE_VOXEL_SIZE_Y, COARSE_VOXEL_SIZE_Z);
    else
      downSample(surfaceData->planes[i], FINE_VOXEL_SIZE_X, FINE_VOXEL_SIZE_Y, FINE_VOXEL_SIZE_Z);

    // cluster planes into seperate surfaces and create SurfaceModels
    cluster(surfaceData->planes[i], surfaceData->planeCoefficients[i], surfaceData->surfaces);
  }
}

template<class PointT>
void SurfaceClusterer<PointT>::updateSurfacesNew(SurfaceDataPtr surfaceData) {
  #pragma omp parallel for schedule(dynamic, 1)
  for (size_t i = 0; i < surfaceData->planes.size(); i++) {
    // cluster planes into seperate surfaces and create SurfaceModels
    clusterNew(surfaceData->planes[i], surfaceData->planeCoefficients[i], surfaceData->surfaces);
  }
}

template<class PointT>
void SurfaceClusterer<PointT>::updateSurfaces(SurfaceDataPtr surfaceData) {
  // reduce number of points of each plane
  using namespace std::chrono;
  auto start_time = high_resolution_clock::now();
#if 0
  updateSurfacesOld(surfaceData);
#else
  updateSurfacesNew(surfaceData);
#endif
  auto end_time = high_resolution_clock::now();
  duration<double, std::milli> duration = end_time - start_time;
  std::cout << "Duration: " << duration.count() << " ms\n" << std::endl;

  notifyObservers(surfaceData);
}

} // namespace lepp

#endif
