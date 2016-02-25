#ifndef lepp3_SURFACE_SEGMENTER_H__
#define lepp3_SURFACE_SEGMENTER_H__

#include "lepp3/Typedefs.hpp"
#include "lepp3/BaseSegmenter.hpp"

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>

#include <cmath>

namespace lepp {

template<class PointT>
class SurfaceSegmenter: public BaseSegmenter<PointT> {
public:
    SurfaceSegmenter();

    /**
    * Segment the given cloud into surfaces. Store the found surfaces and surface model 
    * coefficients in 'surfaces' and 'surfaceCoefficients'. Subtract the found surfaces 
    * from the input cloud and store the remaining cloud in 'cloudMinusSurfaces'.
    */
	virtual void segment(const PointCloudConstPtr& cloud,
			std::vector<PointCloudConstPtr> &surfaces,
			PointCloudPtr &cloudMinusSurfaces,
			std::vector<pcl::ModelCoefficients> &surfaceCoefficients);

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
    void findPlanes(PointCloudPtr const& cloud_filtered, 
    	std::vector<PointCloudPtr> &planes, 
    	std::vector<pcl::ModelCoefficients> &planeCoefficients);

	/**
	* Extracts the Euclidean clusters from the given point cloud.
	* Returns a vector where each element represents the pcl::PointIndices
	* instance representing the corresponding cluster.
	*/
    void getSurfaceClusters(PointCloudPtr const& cloud, 
    	std::vector<pcl::PointIndices> &cluster_indices);

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
	* A plane might contain several non-connected planes that do not correspond
	* to the same surface. Thus, the planes are clustered into seperate surfaces 
	* in this function if necessary.
	**/
	void cluster(std::vector<PointCloudPtr> &planes, 
		std::vector<pcl::ModelCoefficients> &planeCoefficients, 
		std::vector<PointCloudConstPtr> &surfaces,
		std::vector<pcl::ModelCoefficients> &surfaceCoefficients);

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
    void downSample(PointCloudPtr &cloud);

    /**
    * Filter out all the points from the cloud that are close to the surface.
    */
	void filterInvalidObstacles(std::vector<PointCloudConstPtr> &surfaces, 
		PointCloudPtr &cloud);


	/**
	* Return average z-coordinate of all points in given point cloud.
	*/
	double getAverageHeight(const PointCloudT &cloud);

	/**
	* Instance used to extract the planes from the input cloud.
	*/
	pcl::SACSegmentation<PointT> segmentation_;
	/**
	* Instance used to extract the actual clusters from the input cloud.
	*/
	pcl::EuclideanClusterExtraction<PointT> clusterizer_;

	/*Segmentation ratio*/
	const double MIN_FILTER_PERCENTAGE;
};

template<class PointT>
SurfaceSegmenter<PointT>::SurfaceSegmenter() :
        MIN_FILTER_PERCENTAGE(0.1) { //, cloud_surfaces_(new PointCloudT()) {
	// Parameter initialization of the plane segmentation
	segmentation_.setOptimizeCoefficients(true);
	segmentation_.setModelType(pcl::SACMODEL_PLANE);
	segmentation_.setMethodType(pcl::SAC_RANSAC);
	segmentation_.setMaxIterations(200); // value recognized by Irem
	segmentation_.setDistanceThreshold(0.02);
}

template<class PointT>
PointCloudPtr SurfaceSegmenter<PointT>::preprocessCloud(
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
double SurfaceSegmenter<PointT>::getAngle(
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
void SurfaceSegmenter<PointT>::classify(
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
void SurfaceSegmenter<PointT>::findPlanes(
	PointCloudPtr const& cloud_filtered,
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
void SurfaceSegmenter<PointT>::getSurfaceClusters(
		PointCloudPtr const& cloud, std::vector<pcl::PointIndices> &cluster_indices) {
	// Extract the clusters from such a filtered cloud.
	int max_size=cloud->points.size();
	clusterizer_.setClusterTolerance(0.03);
	clusterizer_.setMinClusterSize(1800);
	clusterizer_.setMaxClusterSize(max_size);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr kd_tree_(
			new pcl::search::KdTree<pcl::PointXYZ>);
	kd_tree_->setInputCloud(cloud);
	clusterizer_.setSearchMethod(kd_tree_);
	clusterizer_.setInputCloud(cloud);
	clusterizer_.extract(cluster_indices);
}



template<class PointT>
void SurfaceSegmenter<PointT>::cluster(
	std::vector<PointCloudPtr> &planes,
	std::vector<pcl::ModelCoefficients> &planeCoefficients,
	std::vector<PointCloudConstPtr> &surfaces,
	std::vector<pcl::ModelCoefficients> &surfaceCoefficients) {

	// iterate over all planes found so far
	for (int i = 0; i < planes.size(); i++) 
	{
		// A classified surface may consist of different clusters. 
		// Get point indices of points belonging to the same cluster.
        std::vector<pcl::PointIndices> cluster_indices;
        getSurfaceClusters(planes.at(i), cluster_indices);

        // cluster the current plane into seperate surfaces
        for (int j = 0; j < cluster_indices.size(); j++)
        {
        	// extract all points from the classified surface that 
        	// belong to the same clauster
        	pcl::ExtractIndices<PointT> extract;
        	extract.setInputCloud(planes[i]);
        	// TODO: Optimization. ClusterIndices are copied beacuse
        	// extract expects a PointIndices::Ptr when calling setIndices.
        	pcl::PointIndices::Ptr currentClusterIndices(
        		new pcl::PointIndices(cluster_indices[j]));
			extract.setIndices(currentClusterIndices);
			extract.setNegative(false);
			PointCloudPtr current(new PointCloudT());
			extract.filter(*current);

			// add the extracted cluster to the surface array
			surfaces.push_back(current);
			surfaceCoefficients.push_back(planeCoefficients[i]);
        }
	}
}


template<class PointT>
void SurfaceSegmenter<PointT>::downSample(PointCloudPtr &cloud)
{
  PointCloudPtr cloud_filtered(new PointCloudT());
  pcl::VoxelGrid<PointT> sor;
  sor.setInputCloud (cloud);
  sor.setLeafSize (0.01, 0.01, 0.01);
  sor.filter (*cloud_filtered);
  cloud = cloud_filtered;
}



template<class PointT>
double SurfaceSegmenter<PointT>::getAverageHeight(const PointCloudT &cloud)
{
	double avgHeight = 0;
	for (int i = 0; i < cloud.size(); i++)
		avgHeight += cloud[i].z;

	avgHeight /= cloud.size();
	return avgHeight;
}


template<class PointT>
void SurfaceSegmenter<PointT>::filterInvalidObstacles(
	std::vector<PointCloudConstPtr> &surfaces, 
	PointCloudPtr &cloud)
{
	for (size_t i = 0; i < surfaces.size(); i++)
	{
		double avgHeight = getAverageHeight(*surfaces[i]);

		// surface is not the ground
		if (avgHeight < -0.1) // TODO fine tune parameter
		{
			PointT minPoint;
			PointT maxPoint;
			// get max x,y,z coordinates of points in surfaces[i]
			pcl::getMinMax3D(*surfaces[i], minPoint, maxPoint);

			// enlarge the box in X and Y direction. Use average height for Z-coordinate.
			const double delta = 0.02;
			Eigen::Vector4f minVec(minPoint.x - delta, minPoint.y - delta, avgHeight - 2*delta, 0);
			Eigen::Vector4f maxVec(maxPoint.x + delta, maxPoint.y + delta, avgHeight + 2*delta, 0);

			// filter out all points in the box spanned by minVec and maxVec
			pcl::CropBox<PointT> cropFilter;
			cropFilter.setInputCloud(cloud);
			cropFilter.setMin(minVec);
			cropFilter.setMax(maxVec);
			cropFilter.setNegative(true);
			PointCloudPtr cloudFiltered(new PointCloudT());
			cropFilter.filter(*cloudFiltered);
			cloud = cloudFiltered;
		}
	}
}


template<class PointT>
void SurfaceSegmenter<PointT>::segment(
		const PointCloudConstPtr& cloud,
		std::vector<PointCloudConstPtr> &surfaces,
		PointCloudPtr &cloudMinusSurfaces,
		std::vector<pcl::ModelCoefficients> &surfaceCoefficients) {
	
	cloudMinusSurfaces = preprocessCloud(cloud);
	std::vector<PointCloudPtr> planes;
	std::vector<pcl::ModelCoefficients> planeCoefficients;
    // extract those planes that are considered as surfaces and put them in cloud_surfaces_
    findPlanes(cloudMinusSurfaces, planes, planeCoefficients);

    // reduce number of points of each plane
    for (size_t i = 0; i < planes.size(); i++)
    	downSample(planes[i]);

    // cluster planes into seperate surfaces
    cluster(planes, planeCoefficients, surfaces, surfaceCoefficients);

    // filter out points that are close to stairs
    filterInvalidObstacles(surfaces, cloudMinusSurfaces);
}

} // namespace lepp

#endif