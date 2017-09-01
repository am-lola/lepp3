#ifndef lepp3_SURFACE_FINDER_HPP__
#define lepp3_SURFACE_FINDER_HPP__

#include "lepp3/Typedefs.hpp"
#include "lepp3/util/Timer.hpp"

#include <pcl/filters/model_outlier_removal.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

#include <cmath>
#include <limits>

namespace lepp {

template<class PointT>
class SurfaceFinder {
public:
  struct Parameters {
    int MAX_ITERATIONS;

    //How close a point must be to the model in order to be considered an inlier
    double DISTANCE_THRESHOLD;

    //How small the left (extracted) pointcloud should be for termination of the plane segmentation
    double MIN_FILTER_PERCENTAGE;

    // The function to classify segmented planes according to deviation in their normals
    double DEVIATION_ANGLE;

  };

  SurfaceFinder(bool surfaceDetectorActive, Parameters const& surfFinderParameters);

  /**
  * Segment the given cloud into surfaces. Store the found surfaces and surface model
  * coefficients in 'surfaces' and 'surfaceCoefficients'. Subtract the found surfaces
  * from the input cloud and store the remaining cloud in 'cloudMinusSurfaces'.
  */
  void findSurfaces(
      PointCloudPtr cloud,
      std::vector<PointCloudPtr>& planes,
      std::vector<pcl::ModelCoefficients>& planeCoefficients);

private:
  /**
  * Detect all planes in the given point cloud and store those and their
  * coefficients in the given vectors.
  */
  void findPlanes(PointCloudPtr& cloud_filtered,
                  std::vector<PointCloudPtr>& planes,
                  std::vector<pcl::ModelCoefficients>& planeCoefficients);

  /**
  * If surface detector is disabled, only the ground has to be removed but all other
  * planes have to stay in place. For this only one plane (the ground) and its coefficients
  * are passed on to the PlaneInlierFinder where the ground is removed from the point cloud.
  */
  void removeNonGroundCoefficients(std::vector<PointCloudPtr>& planes,
                                   std::vector<pcl::ModelCoefficients>& planeCoefficients);

  /**
  * Several planes corresponding to the same surface might be detected.
  * Merge planes that have almost the same normal vector and z-intersection.
  **/
  void classify(PointCloudPtr const& cloud_planar_surface,
                const pcl::ModelCoefficients& coeffs,
                std::vector<PointCloudPtr>& planes,
                std::vector<pcl::ModelCoefficients>& planeCoefficients);

  /**
  * Returns the angle between two plane represented by their model coefficients.
  */
  double getAngle(const pcl::ModelCoefficients& coeffs1,
                  const pcl::ModelCoefficients& coeffs2);


  //Previous plane coeffs are only used for tricking ransac, comment out for trial with the rest of the relevant part
  std::vector<pcl::ModelCoefficients> previous_plane_coeffs;

  /**
  * Instance used to extract the planes from the input cloud.
  */
  pcl::SACSegmentation<PointT> segmentation_;

  //max number of RANSAC iterations
  const int MAX_ITERATIONS;

  //How close a point must be to the model in order to be considered an inlier
  const double DISTANCE_THRESHOLD;

  //How small the left (extracted) pointcloud should be for termination of the plane segmentation
  const double MIN_FILTER_PERCENTAGE;

  // The function to classify segmented planes according to deviation in their normals
  const double DEVIATION_ANGLE;

  // boolean indicating whether the surface detector was activated in config file
  bool surfaceDetectorActive;
};

template<class PointT>
SurfaceFinder<PointT>::SurfaceFinder(bool surfaceDetectorActive, Parameters const& surfFinderParameters)
    : surfaceDetectorActive(surfaceDetectorActive), MAX_ITERATIONS(surfFinderParameters.MAX_ITERATIONS),
      DISTANCE_THRESHOLD(surfFinderParameters.DISTANCE_THRESHOLD),
      MIN_FILTER_PERCENTAGE(surfFinderParameters.MIN_FILTER_PERCENTAGE),
      DEVIATION_ANGLE(surfFinderParameters.DEVIATION_ANGLE) { //, cloud_surfaces_(new PointCloudT()) {
  // Parameter initialization of the plane segmentation
  segmentation_.setOptimizeCoefficients(true);
  segmentation_.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
  segmentation_.setMethodType(pcl::SAC_RANSAC);
  segmentation_.setMaxIterations(MAX_ITERATIONS); // value recognized by Irem
  segmentation_.setDistanceThreshold(DISTANCE_THRESHOLD);
  segmentation_.setAxis(Eigen::Vector3f(0.0, 0.0, 1.0));
  segmentation_.setEpsAngle(0.26); // allowed deviation of surface normals from vertical axis: ~15 degrees
}


template<class PointT>
double SurfaceFinder<PointT>::getAngle(
    const pcl::ModelCoefficients& coeffs1, const pcl::ModelCoefficients& coeffs2) {
  //Scalar Product
  float scalar_product =
      (coeffs2.values[0] * coeffs1.values[0])
      + (coeffs2.values[1] * coeffs1.values[1])
      + (coeffs2.values[2] * coeffs1.values[2]);
  double angle = acos(scalar_product) * 180.0 / M_PI;
  return angle;
}


template<class PointT>
void SurfaceFinder<PointT>::classify(
    PointCloudPtr const& cloud_planar_surface,
    const pcl::ModelCoefficients& coeffs,
    std::vector<PointCloudPtr>& planes,
    std::vector<pcl::ModelCoefficients>& planeCoefficients) {

  int size = planeCoefficients.size();
  for (int i = 0; i < size; i++) {
    double angle = getAngle(coeffs, planeCoefficients.at(i));
    // ax + by + cz + d = 0
    // two planes belong to the same surface if the angle of the normal vector
    // to the groud is roughly the same and if their 'height' (intersectionf of plane with z-axis)
    // is roughly the same. Note, that the 'height' is given by ax + by + cz + d = 0 where x=y=0,
    // i.e. by z = -d/c
    if ((angle < DEVIATION_ANGLE || angle > 180 - DEVIATION_ANGLE) &&
        (std::abs(coeffs.values[3] / coeffs.values[2] -
                  planeCoefficients.at(i).values[3] / planeCoefficients.at(i).values[2]) < 0.01)) {
      *planes.at(i) += *cloud_planar_surface;
      return;
    }
  }
  planes.push_back(cloud_planar_surface);
  planeCoefficients.push_back(coeffs);
}


template<class PointT>
void SurfaceFinder<PointT>::findPlanes(
    PointCloudPtr& cloud_filtered,
    std::vector<PointCloudPtr>& planes,
    std::vector<pcl::ModelCoefficients>& planeCoefficients) {

  HiResTimer timer;
  timer.start();
  // Instance that will be used to perform the elimination of unwanted points
  // from the point cloud.
  pcl::ExtractIndices<PointT> extract;

  // Will hold the indices of the next extracted plane within the loop
  pcl::PointIndices::Ptr currentPlaneIndices(new pcl::PointIndices);

  // Remove planes until we reach x % of the original number of points
  const size_t pointThreshold = MIN_FILTER_PERCENTAGE * cloud_filtered->size();

  /************ TRICK RANSAC HERE ************Comment out with previous plane coeff variable above for trial********/
   
  std::cout << "Tricking RANSAC, original size: " << cloud_filtered->points.size() << std::endl;
  std::cout << "Previous Coeffs: " << previous_plane_coeffs.size() << std::endl;
  for (size_t i = 0; i < previous_plane_coeffs.size(); i++)
  {
    PointCloudPtr currentPlane(new PointCloudT());
    // PointCloudPtr plane_points (new pcl::PointCloud<PointT>);
    pcl::PointIndices::Ptr plane_indices(new pcl::PointIndices);
    pcl::ModelOutlierRemoval<PointT> plane_filter(true);
    plane_filter.setModelCoefficients (previous_plane_coeffs[i]);
    plane_filter.setThreshold (0.04);
    plane_filter.setModelType (pcl::SACMODEL_PLANE);
    plane_filter.setInputCloud (cloud_filtered);
    plane_filter.filter (plane_indices->indices);

    if (plane_indices->indices.size() < 1000)
      continue;

    std::cout << "Plane " << i << ": " << plane_indices->indices.size() << " inliers" << std::endl;

    pcl::ExtractIndices<PointT> extract;
    extract.setIndices(plane_indices);
    extract.setInputCloud(cloud_filtered);
    extract.setNegative(false);
    extract.filter(*currentPlane);

    extract.setNegative(true);
    extract.filter(*cloud_filtered);

    classify(currentPlane, previous_plane_coeffs[i], planes, planeCoefficients);
  }
  std::cout << "Tricking RANSAC, filtered size: " << cloud_filtered->points.size() << "/" << pointThreshold << std::endl;
  previous_plane_coeffs.clear(); 
  
  //******************TRICK RANSAC**************************/

  bool first = true;
  while (cloud_filtered->size() > pointThreshold) {
    // Try to obtain the next plane...
    pcl::ModelCoefficients currentPlaneCoefficients;
    segmentation_.setInputCloud(cloud_filtered);
    segmentation_.segment(*currentPlaneIndices, currentPlaneCoefficients);

    // We didn't get any plane in this run. Therefore, there are no more planes
    // to be removed from the cloud.
    if (currentPlaneIndices->indices.size() == 0)
      break;

    // Cloud that holds a plane in each iteration, to be added to the total cloud.
    PointCloudPtr currentPlane(new PointCloudT());

    // Add the planar inliers to the cloud holding the surfaces
    extract.setInputCloud(cloud_filtered);
    extract.setIndices(currentPlaneIndices);
    extract.setNegative(false);
    extract.filter(*currentPlane);

    // ... and remove those inliers from the input cloud
    extract.setNegative(true);
    extract.filter(*cloud_filtered);

    //Classify the Cloud
    classify(currentPlane, currentPlaneCoefficients, planes, planeCoefficients);
    previous_plane_coeffs.push_back(currentPlaneCoefficients);
  }
  timer.stop();
  std::cout << "Finding planes took " << timer.duration() << " ms." << std::endl;
}


template<class PointT>
void SurfaceFinder<PointT>::removeNonGroundCoefficients(
    std::vector<PointCloudPtr>& planes,
    std::vector<pcl::ModelCoefficients>& planeCoefficients) {
  if (0 == planes.size()) {
    return;
  }

  // compute centroid for each plane and find minimum
  double minHeight = std::numeric_limits<double>::max();
  size_t minIndex = 0;
  for (size_t i = 0; i < planes.size(); i++) {
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*planes[i], centroid);
    if (centroid[2] < minHeight) {
      minHeight = centroid[2];
      minIndex = i;
    }
  }

  // replace planes and plane coefficients by new vectors that only include the ground
  std::vector<PointCloudPtr> ground;
  ground.push_back(planes[minIndex]);
  planes = ground;
  std::vector<pcl::ModelCoefficients> groundCoeff;
  groundCoeff.push_back(planeCoefficients[minIndex]);
  planeCoefficients = groundCoeff;
}


template<class PointT>
void SurfaceFinder<PointT>::findSurfaces(
    PointCloudPtr cloud,
    std::vector<PointCloudPtr>& planes,
    std::vector<pcl::ModelCoefficients>& planeCoefficients) {
  // extract those planes that are considered as surfaces and put them in cloud_surfaces_
  findPlanes(cloud, planes, planeCoefficients);

  if (!surfaceDetectorActive)
    // remove all coefficients from planeCoefficients except for the ground coefficients
    removeNonGroundCoefficients(planes, planeCoefficients);
}

} // namespace lepp

#endif
