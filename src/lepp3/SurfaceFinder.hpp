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
		FrameDataPtr frameData, 
		std::vector<PointCloudPtr> &planes, 
		std::vector<pcl::ModelCoefficients> &planeCoefficients);

private:
	/**
	* Performs some initial preprocessing and filtering appropriate for the
	* segmentation algorithm.
	* Takes the original cloud as a parameter and returns a pointer to a newly
	* created (and allocated) cloud containing the result of the filtering.
	*/
	PointCloudPtr preprocessCloud(PointCloudConstPtr const& cloud);

	/**
	* Detect all planes in the given point cloud and store those and their 
	* coefficients in the given vectors.
	*/
    void findPlanes(PointCloudPtr &cloud_filtered, 
    	std::vector<PointCloudPtr> &planes, 
    	std::vector<pcl::ModelCoefficients> &planeCoefficients);

	/**
	* If surface detector is disabled, only the ground has to be removed but all other
	* planes have to stay in place. To make sure that the whole ground is removed,
	* planes are detected and classified and all removed from the given point cloud.
	* In a second step all non-ground planes are added back to the point cloud.
	* This is done by removing the lowest plane (which is the ground) and all other planes
	* that are less than a few centimeters apart from the plane.
	*/
	void addNonGroundPlanes(PointCloudPtr &cloud_filtered, 
		std::vector<PointCloudPtr> &planes);


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
PointCloudPtr SurfaceFinder<PointT>::preprocessCloud(
		PointCloudConstPtr const& cloud) {
	// Remove NaN points from the input cloud.
	// The pcl API forces us to pass in a reference to the vector, even if we have
	// no use of it later on ourselves.
	PointCloudPtr cloud_filtered(new PointCloudT());
	std::vector<int> index;
	pcl::removeNaNFromPointCloud<PointT>(*cloud, *cloud_filtered, index);

	return cloud_filtered;
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
void SurfaceFinder<PointT>::addNonGroundPlanes(
	PointCloudPtr &cloud_filtered, 
	std::vector<PointCloudPtr> &planes)
{
	// compute centroid for each plane and find minimum
	double minHeight = std::numeric_limits<double>::lowest();
	double planeHeights[planes.size()];
	for (size_t i = 0; i < planes.size(); i++)
	{
		Eigen::Vector4f centroid;
		pcl::compute3DCentroid (*planes[i], centroid);
		planeHeights[i] = centroid[2];
		// pay attention to the orientation of the z-axis! Axis is inverted!
		if (planeHeights[i] > minHeight)
			minHeight = planeHeights[i];
	}		
	
	// add all planes back that are further than Xcm apart from the ground
	for (size_t i = 0; i < planes.size(); i++)
	{
		if (std::abs(planeHeights[i] - minHeight) > 0.05)
			*cloud_filtered += *planes[i];
	}
}


template<class PointT>
void SurfaceFinder<PointT>::findSurfaces(
	FrameDataPtr frameData, 
	std::vector<PointCloudPtr> &planes, 
	std::vector<pcl::ModelCoefficients> &planeCoefficients) 
{
	frameData->cloudMinusSurfaces = preprocessCloud(frameData->cloud);

    // extract those planes that are considered as surfaces and put them in cloud_surfaces_
    findPlanes(frameData->cloudMinusSurfaces, planes, planeCoefficients);

    if (!surfaceDetectorActive)
    {
    	// add all planes beside the ground back to the point cloud
		addNonGroundPlanes(frameData->cloudMinusSurfaces, planes);
		// do not cluster and process planes if surface detector was disabled
    	return;
    }
}

} // namespace lepp

#endif