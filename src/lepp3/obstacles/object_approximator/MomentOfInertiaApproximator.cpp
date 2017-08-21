#include "MomentOfInertiaApproximator.hpp"

#include <pcl/common/pca.h>
#include <pcl/common/common.h>
#include <pcl/kdtree/kdtree_flann.h>

lepp::ObjectModelPtr lepp::MomentOfInertiaObjectApproximator::approximate(const ObjectModelParams& object_params) {
  // Firstly, obtain the principal component descriptors
  float major_value, middle_value, minor_value;
  pcl::PCA<PointT> pca;
  pca.setInputCloud(object_params.obstacleCloud);
  Eigen::Vector3f eigenvalues = pca.getEigenValues();
  major_value = eigenvalues(0);
  middle_value = eigenvalues(1);
  minor_value = eigenvalues(2);
  Eigen::Matrix3f eigenvectors = pca.getEigenVectors();
  std::vector<Eigen::Vector3f> axes;
  for (size_t i = 0; i < 3; ++i) {
    axes.push_back(eigenvectors.col(i));
  }

  // if we have a hint for the object's center use it, if not estimate it from center of mass
  Eigen::Vector3f mass_center;
  if (!std::isnan(object_params.center.x) && !std::isnan(object_params.center.y) && !std::isnan(object_params.center.z))
    mass_center = object_params.center;
  else
    mass_center = estimateMassCenter(object_params.obstacleCloud);

  // Based on these descriptors, decide which object type should be used.
  boost::shared_ptr<ObjectModel> model;
  if ((middle_value / major_value > .6) && (minor_value / major_value > .1)) {
    boost::shared_ptr<SphereModel> sphere(new SphereModel(0, Coordinate()));
    performFitting(sphere, object_params.obstacleCloud, mass_center, axes);
    model = sphere;
  } else if (middle_value / major_value < .25) {
    boost::shared_ptr<CapsuleModel> capsule(new CapsuleModel(0, Coordinate(), Coordinate()));
    performFitting(capsule, object_params.obstacleCloud, mass_center, axes);
    model = capsule;
  } else {
    // The fall-back is a sphere
    boost::shared_ptr<SphereModel> sphere(new SphereModel(0, Coordinate()));
    performFitting(sphere, object_params.obstacleCloud, mass_center, axes);
    model = sphere;
  }

  // Now we pack the model into a "composite" of one element to satisfy the
  // interface!
  boost::shared_ptr<CompositeModel> approx(new CompositeModel);
  model->set_id(object_params.id);
  approx->addModel(model);
  return approx;
}

Eigen::Vector3f lepp::MomentOfInertiaObjectApproximator::estimateMassCenter(const PointCloudConstPtr& point_cloud) {
  PointT min_pt;
  PointT max_pt;
  // TODO Is this really a good heuristic? (It comes from the legacy code)
  pcl::getMinMax3D(*point_cloud, min_pt, max_pt);
  Eigen::Vector3f mass_center;
  mass_center(0) = (max_pt.x + min_pt.x) / 2;
  mass_center(1) = 1.02 * ((max_pt.y + min_pt.y) / 2);
  mass_center(2) = 1.02 * ((max_pt.z + min_pt.z) / 2);

  return mass_center;
}


void lepp::MomentOfInertiaObjectApproximator::performFitting(boost::shared_ptr<SphereModel> sphere,
                                                             const PointCloudConstPtr& point_cloud,
                                                             Eigen::Vector3f mass_center,
                                                             std::vector<Eigen::Vector3f> const& axes) {
  Eigen::VectorXf coeffs;

  Eigen::Vector4f max_point;
  Eigen::Vector4f center;

  Eigen::Vector3f result;

  float radius;
  float dist;
  std::vector<int> nearestPointIndex(1);
  std::vector<float> nearestPointSquaredDistance(1);
  float max_dist;
  Eigen::Vector3f max_distance_vector;
  PointT searchPoint;
  pcl::KdTreeFLANN<PointT> kdtree;

  Eigen::Vector3f radius_vector;

  center(0) = mass_center(0);
  center(1) = mass_center(1);
  center(2) = mass_center(2);
  pcl::getMaxDistance(*point_cloud, center, max_point);

  // calculate radius
  radius_vector(0) = center(0) - max_point(0);
  radius_vector(1) = center(1) - max_point(1);
  radius_vector(2) = center(2) - max_point(2);
  radius = sqrt(
      radius_vector(0) * radius_vector(0) + radius_vector(1) * radius_vector(1) + radius_vector(2) * radius_vector(2));

  // Finally sets the calculated coeffs
  sphere->set_radius(radius);
  sphere->set_center(Coordinate(center(0), center(1), center(2)));
}


void lepp::MomentOfInertiaObjectApproximator::performFitting(boost::shared_ptr<CapsuleModel> capsule,
                                                             const PointCloudConstPtr& point_cloud,
                                                             Eigen::Vector3f mass_center,
                                                             std::vector<Eigen::Vector3f> const& axes) {
  Eigen::VectorXf coeffs;

  Eigen::Vector4f max_point;
  Eigen::Vector4f center;

  Eigen::Vector3f result;

  float radius;
  float dist;
  std::vector<int> nearestPointIndex(1);
  std::vector<float> nearestPointSquaredDistance(1);
  float max_dist;
  Eigen::Vector3f max_distance_vector;
  PointT searchPoint;
  pcl::KdTreeFLANN<PointT> kdtree;

  float dist_y;
  float dist_z;
  PointT center_xyz;
  Eigen::Vector3f point_help;


  // find the point with maximum distance from the center
  center(0) = mass_center(0);
  center(1) = mass_center(1);
  center(2) = mass_center(2);
  pcl::getMaxDistance(*point_cloud, center, max_point);

  // calculate max_distance from center point
  max_distance_vector(0) = center(0) - max_point(0);
  max_distance_vector(1) = center(1) - max_point(1);
  max_distance_vector(2) = center(2) - max_point(2);
  max_dist = sqrt(max_distance_vector(0) * max_distance_vector(0) + max_distance_vector(1) * max_distance_vector(1) +
                  max_distance_vector(2) * max_distance_vector(2));

  // create 1st searchPoint
  searchPoint.x = center(0) + max_dist * (axes.at(0))(0);
  searchPoint.y = center(1) + max_dist * (axes.at(0))(1);
  searchPoint.z = center(2) + max_dist * (axes.at(0))(2);

  // search nearest point in x-direction
  kdtree.setInputCloud(point_cloud);
  kdtree.nearestKSearch(searchPoint, 1, nearestPointIndex, nearestPointSquaredDistance);
  dist = sqrt((((point_cloud->at(nearestPointIndex[0]))).x - center(0)) *
              ((point_cloud->at(nearestPointIndex[0])).x - center(0)) +
              ((point_cloud->at(nearestPointIndex[0])).y - center(1)) *
              ((point_cloud->at(nearestPointIndex[0])).y - center(1)) +
              ((point_cloud->at(nearestPointIndex[0])).z - center(2)) *
              ((point_cloud->at(nearestPointIndex[0])).z - center(2)));

  // point1 in camera system
  // 0.75 to make sure that all points are inliers (consider the two hemispheres at the ends)
  point_help(0) = center(0) + 0.75 * dist * (axes.at(0))(0);
  point_help(1) = center(1) + 0.75 * dist * (axes.at(0))(1);
  point_help(2) = center(2) + 0.75 * dist * (axes.at(0))(2);

  // save results format r, p1_notrans, p2_notrans, p1_trans, p2_trans
  // save p1_notrans
  coeffs.resize(13);
  coeffs[1] = point_help(0);
  coeffs[2] = point_help(1);
  coeffs[3] = point_help(2);

  // Transform to OdoCoordinateSystem
  // (Skip this -- shouldn't be a part of the detection...)
  // result = TransformPointToOdo(point_help);
  // save p1_trans
  coeffs[7] = 0.;
  coeffs[8] = 0.;
  coeffs[9] = 0.;

  // point2 in camera system
  // 0.75 to make sure that all points are inliers (consider the two hemispheres at the ends)
  point_help(0) = center(0) - 0.75 * dist * (axes.at(0))(0);
  point_help(1) = center(1) - 0.75 * dist * (axes.at(0))(1);
  point_help(2) = center(2) - 0.75 * dist * (axes.at(0))(2);

  // save p2_notrans
  coeffs[4] = point_help(0);
  coeffs[5] = point_help(1);
  coeffs[6] = point_help(2);

  // Transform to OdoCoordinateSystem
  // result = TransformPointToOdo(point_help);

  // save p2_trans
  coeffs[10] = 0.;
  coeffs[11] = 0.;
  coeffs[12] = 0.;

  // search nearest point in +/- y-direction and +- z-direction and get min distance
  // + y-direction
  searchPoint.x = center(0) + max_dist / 1.5 * (axes.at(1))(0);
  searchPoint.y = center(1) + max_dist / 1.5 * (axes.at(1))(1);
  searchPoint.z = center(2) + max_dist / 1.5 * (axes.at(1))(2);
  kdtree.nearestKSearch(searchPoint, 1, nearestPointIndex, nearestPointSquaredDistance);
  dist_y = sqrt((((point_cloud->at(nearestPointIndex[0]))).x - center(0)) *
                ((point_cloud->at(nearestPointIndex[0])).x - center(0)) +
                ((point_cloud->at(nearestPointIndex[0])).y - center(1)) *
                ((point_cloud->at(nearestPointIndex[0])).y - center(1)) +
                ((point_cloud->at(nearestPointIndex[0])).z - center(2)) *
                ((point_cloud->at(nearestPointIndex[0])).z - center(2)));

  // - y-direction
  searchPoint.x = center(0) - max_dist / 1.5 * (axes.at(1))(0);
  searchPoint.y = center(1) - max_dist / 1.5 * (axes.at(1))(1);
  searchPoint.z = center(2) - max_dist / 1.5 * (axes.at(1))(2);
  kdtree.nearestKSearch(searchPoint, 1, nearestPointIndex, nearestPointSquaredDistance);
  dist = sqrt((((point_cloud->at(nearestPointIndex[0]))).x - center(0)) *
              ((point_cloud->at(nearestPointIndex[0])).x - center(0)) +
              ((point_cloud->at(nearestPointIndex[0])).y - center(1)) *
              ((point_cloud->at(nearestPointIndex[0])).y - center(1)) +
              ((point_cloud->at(nearestPointIndex[0])).z - center(2)) *
              ((point_cloud->at(nearestPointIndex[0])).z - center(2)));
  if (dist_y > dist) {
    dist_y = dist;
  }

  // + z-direction
  searchPoint.x = center(0) + max_dist / 1.5 * (axes.at(2))(0);
  searchPoint.y = center(1) + max_dist / 1.5 * (axes.at(2))(1);
  searchPoint.z = center(2) + max_dist / 1.5 * (axes.at(2))(2);
  kdtree.nearestKSearch(searchPoint, 1, nearestPointIndex, nearestPointSquaredDistance);
  dist_z = sqrt((((point_cloud->at(nearestPointIndex[0]))).x - center(0)) *
                ((point_cloud->at(nearestPointIndex[0])).x - center(0)) +
                ((point_cloud->at(nearestPointIndex[0])).y - center(1)) *
                ((point_cloud->at(nearestPointIndex[0])).y - center(1)) +
                ((point_cloud->at(nearestPointIndex[0])).z - center(2)) *
                ((point_cloud->at(nearestPointIndex[0])).z - center(2)));

  // - z-direction
  searchPoint.x = center(0) - max_dist / 1.5 * (axes.at(2))(0);
  searchPoint.y = center(1) - max_dist / 1.5 * (axes.at(2))(1);
  searchPoint.z = center(2) - max_dist / 1.5 * (axes.at(2))(2);
  kdtree.nearestKSearch(searchPoint, 1, nearestPointIndex, nearestPointSquaredDistance);
  dist = sqrt((((point_cloud->at(nearestPointIndex[0]))).x - center(0)) *
              ((point_cloud->at(nearestPointIndex[0])).x - center(0)) +
              ((point_cloud->at(nearestPointIndex[0])).y - center(1)) *
              ((point_cloud->at(nearestPointIndex[0])).y - center(1)) +
              ((point_cloud->at(nearestPointIndex[0])).z - center(2)) *
              ((point_cloud->at(nearestPointIndex[0])).z - center(2)));
  if (dist_z > dist) {
    dist_z = dist;
  }


  // calculate radius for a safety solution
  radius = sqrt(dist_y * dist_y + dist_z * dist_z);

  // save radius
  coeffs[0] = 0.9 * radius;

  capsule->set_radius(.9 * radius);
  capsule->set_first(Coordinate(coeffs[1], coeffs[2], coeffs[3]));
  capsule->set_second(Coordinate(coeffs[4], coeffs[5], coeffs[6]));
}
