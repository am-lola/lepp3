#ifndef lepp3_SURFACE_FINDER_HPP__
#define lepp3_SURFACE_FINDER_HPP__

#include "lepp3/Typedefs.hpp"

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

#include <cmath>
#include <limits>

namespace lepp {

template<class PointT>
class SurfaceFinder 
{
public:
    SurfaceFinder(bool surfaceDetectorActive);

    /**
    * Segment the given cloud into surfaces. Store the found surfaces and surface model 
    * coefficients in 'surfaces' and 'surfaceCoefficients'. Subtract the found surfaces 
    * from the input cloud and store the remaining cloud in 'cloudMinusSurfaces'.
    */
	void findSurfaces(
		PointCloudPtr cloud,
		std::vector<PointCloudPtr> &planes, 
		std::vector<pcl::ModelCoefficients> &planeCoefficients);

private:
	/**
	* Detect all planes in the given point cloud and store those and their 
	* coefficients in the given vectors.
	*/
    void findPlanes(PointCloudPtr &cloud_filtered, 
    	std::vector<PointCloudPtr> &planes, 
    	std::vector<pcl::ModelCoefficients> &planeCoefficients);

	/**
	* If surface detector is disabled, only the ground has to be removed but all other
	* planes have to stay in place. For this only one plane (the ground) and its coefficients
	* are passed on to the PlaneInlierFinder where the ground is removed from the point cloud.
	*/
	void removeNonGroundCoefficients(std::vector<PointCloudPtr> &planes, 
		std::vector<pcl::ModelCoefficients>  &planeCoefficients);

	/**
	* Several planes corresponding to the same surface might be detected.
	* Merge planes that have almost the same normal vector and z-intersection.
	**/
	void classify(PointCloudPtr const& cloud_planar_surface,
			const pcl::ModelCoefficients &coeffs,
			std::vector<PointCloudPtr> &planes,
			std::vector<pcl::ModelCoefficients> &planeCoefficients);

	/**
	* Returns the angle between two plane represented by their model coefficients.
	*/
	double getAngle(const pcl::ModelCoefficients &coeffs1, 
		const pcl::ModelCoefficients &coeffs2);

	/**
	* Instance used to extract the planes from the input cloud.
	*/
	pcl::SACSegmentation<PointT> segmentation_;

	/*Segmentation ratio*/
	const double MIN_FILTER_PERCENTAGE;

	// boolean indicating whether the surface detector was activated in config file
	bool surfaceDetectorActive;
};

template<class PointT>
SurfaceFinder<PointT>::SurfaceFinder(bool surfaceDetectorActive) :
        MIN_FILTER_PERCENTAGE(0.08),
        surfaceDetectorActive(surfaceDetectorActive) 
{ //, cloud_surfaces_(new PointCloudT()) {
	// Parameter initialization of the plane segmentation
	segmentation_.setOptimizeCoefficients(true);
	segmentation_.setModelType(pcl::SACMODEL_PLANE);
	segmentation_.setMethodType(pcl::SAC_RANSAC);
	segmentation_.setMaxIterations(200); // value recognized by Irem
	segmentation_.setDistanceThreshold(0.02);
}



template<class PointT> 
double SurfaceFinder<PointT>::getAngle(
		const pcl::ModelCoefficients &coeffs1, const pcl::ModelCoefficients &coeffs2) {
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
		const pcl::ModelCoefficients & coeffs,
		std::vector<PointCloudPtr> &planes,
		std::vector<pcl::ModelCoefficients> &planeCoefficients) {

	int size = planeCoefficients.size();
	for (int i = 0; i < size; i++) {
		double angle = getAngle(coeffs, planeCoefficients.at(i));
		// ax + by + cz + d = 0
		// two planes belong to the same surface if the angle of the normal vector 
		// to the groud is roughly the same and if their 'height' (intersectionf of plane with z-axis)
		// is roughly the same. Note, that the 'height' is given by ax + by + cz + d = 0 where x=y=0,
		// i.e. by z = -d/c
		if ((angle < 3 || angle > 177) && 
			(std::abs(coeffs.values[3]/coeffs.values[2] - 
				planeCoefficients.at(i).values[3]/planeCoefficients.at(i).values[2]) < 0.01)) {
			*planes.at(i) += *cloud_planar_surface;
			return;
		}
	}
	planes.push_back(cloud_planar_surface);
	planeCoefficients.push_back(coeffs);
}



template<class PointT>
void SurfaceFinder<PointT>::findPlanes(
	PointCloudPtr &cloud_filtered,
	std::vector<PointCloudPtr> &planes,
	std::vector<pcl::ModelCoefficients> &planeCoefficients) {

	// Instance that will be used to perform the elimination of unwanted points
	// from the point cloud.
	pcl::ExtractIndices<PointT> extract;

	// Will hold the indices of the next extracted plane within the loop
	pcl::PointIndices::Ptr currentPlaneIndices(new pcl::PointIndices);

	// Remove planes until we reach x % of the original number of points
	const size_t pointThreshold = MIN_FILTER_PERCENTAGE * cloud_filtered->size();

	bool first=true;
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
	}
}


template<class PointT>
void SurfaceFinder<PointT>::removeNonGroundCoefficients(
	std::vector<PointCloudPtr> &planes, 
	std::vector<pcl::ModelCoefficients>  &planeCoefficients)
{
	// compute centroid for each plane and find minimum
	double minHeight = std::numeric_limits<double>::max();
	size_t minIndex = -1;
	for (size_t i = 0; i < planes.size(); i++)
	{
		Eigen::Vector4f centroid;
		pcl::compute3DCentroid (*planes[i], centroid);
		if (centroid[2] < minHeight)
		{
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
	std::vector<PointCloudPtr> &planes, 
	std::vector<pcl::ModelCoefficients>  &planeCoefficients) 
{
    // extract those planes that are considered as surfaces and put them in cloud_surfaces_
  	findPlanes(cloud, planes, planeCoefficients);

    if (!surfaceDetectorActive)
		// remove all coefficients from planeCoefficients except for the ground coefficients
		removeNonGroundCoefficients(planes, planeCoefficients);
}

} // namespace lepp

#endif