#ifndef lepp3_SURFACE_FEATURE_ESTIMATOR_H__
#define lepp3_SURFACE_FEATURE_ESTIMATOR_H__

#include "lepp3/SurfaceApproximator.hpp"

#include <pcl/common/pca.h>
#include <pcl/common/common.h>

#include "lepp3/models/SurfaceModel.h"

namespace lepp {

/**
 * An implementation of the SurfaceApproximator abstract base class that performs
 * approximations to the features or attributes of a detected surface. The implementation
 *  is based on approximations in obstacle code.
 */

template<class PointT>
class SurfaceFeatureEstimator: public SurfaceApproximator<PointT> {
public:
	boost::shared_ptr<PlaneModel> approximate(
			PointCloudConstPtr &point_cloud);
private:
	/*	Takes a pointer to a plane model and sets the parameters of the
	 model so that it describes the point cloud with the given features in the
	 best way.
	 This design isn't even *that* bad, but it would be better to use some sort
	 of double-dispatch so that Approximator doesn't need to be recompiled
	 when another model is added.
	 */
	void performFitting(boost::shared_ptr<PlaneModel> plane,
			const typename pcl::PointCloud<PointT>::ConstPtr& point_cloud,
			Eigen::Vector3f mass_center,
			std::vector<Eigen::Vector3f> const& axes);

	/**
	 * Returns a point representing an estimation of the position of the center
	 * of mass for the given point cloud.
	 */
	Eigen::Vector3f estimateMassCenter(
			const typename pcl::PointCloud<PointT>::ConstPtr& point_cloud);

	/*Creates a bounding box around the surface and returns the area of the bounding box
	 * The result is closer to real surface area compared to the functions returning area of
	 * the estimated polygon for the surface.*/

	float getBoundingBoxArea(
			const typename pcl::PointCloud<PointT>::ConstPtr& point_cloud);
};

template<class PointT>
boost::shared_ptr<PlaneModel> SurfaceFeatureEstimator<PointT>::approximate(
		PointCloudConstPtr &point_cloud) {
	// Firstly, obtain the principal component descriptors
	float major_value, middle_value, minor_value;
	pcl::PCA<PointT> pca;
	pca.setInputCloud(point_cloud);
	Eigen::Vector3f eigenvalues = pca.getEigenValues();
	major_value = eigenvalues(0);
	middle_value = eigenvalues(1);
	minor_value = eigenvalues(2);
	Eigen::Matrix3f eigenvectors = pca.getEigenVectors();

	std::vector<Eigen::Vector3f> axes;

	for (size_t i = 0; i < 3; ++i) {
		axes.push_back(eigenvectors.col(i));
	}

	// Guesstimate the center of mass
	Eigen::Vector3f mass_center(estimateMassCenter(point_cloud));

	boost::shared_ptr < PlaneModel > plane(new PlaneModel(0, Coordinate(), 0, point_cloud));
	performFitting(plane, point_cloud, mass_center, axes);

	return plane;
}

template<class PointT>
Eigen::Vector3f SurfaceFeatureEstimator<PointT>::estimateMassCenter(
		const typename pcl::PointCloud<PointT>::ConstPtr& point_cloud) {
	PointT min_pt;
	PointT max_pt;

	pcl::getMinMax3D(*point_cloud, min_pt, max_pt);
	Eigen::Vector3f mass_center;
	mass_center(0) = (max_pt.x + min_pt.x) / 2;
	mass_center(1) = 1.02 * ((max_pt.y + min_pt.y) / 2);
	mass_center(2) = 1.02 * ((max_pt.z + min_pt.z) / 2);

	return mass_center;
}

template<class PointT>
void SurfaceFeatureEstimator<PointT>::performFitting(
		boost::shared_ptr<PlaneModel> plane,
		const typename pcl::PointCloud<PointT>::ConstPtr& point_cloud,
		Eigen::Vector3f mass_center, std::vector<Eigen::Vector3f> const& axes) {

	Eigen::Vector4f center;

	center(0) = mass_center(0);
	center(1) = mass_center(1);
	center(2) = mass_center(2);
	plane->set_area(getBoundingBoxArea(point_cloud));
	plane->set_center(Coordinate(center(0), center(1), center(2)));

	//TODO implement inclenation
	plane->set_inclination(0);
}
template<class PointT>
float SurfaceFeatureEstimator<PointT>::getBoundingBoxArea(
		const typename pcl::PointCloud<PointT>::ConstPtr& point_cloud) {

	PointT min_pt;
	PointT max_pt;

	pcl::getMinMax3D(*point_cloud, min_pt, max_pt);
	double area =(max_pt.x - min_pt.x) * (max_pt.y - min_pt.y);
    return area;
}

} // namespace lepp

#endif
