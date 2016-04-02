#ifndef lepp3_SURFACE_SEGMENTER_H__
#define lepp3_SURFACE_SEGMENTER_H__

#include "lepp3/Typedefs.hpp"
#include "lepp3/BaseSegmenter.hpp"

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/project_inliers.h>

#include <cmath>
#include <limits>

namespace lepp {

template<class PointT>
class SurfaceSegmenter: public BaseSegmenter<PointT> {
public:
    SurfaceSegmenter(bool surfaceDetectorActive);

    /**
    * Segment the given cloud into surfaces. Store the found surfaces and surface model 
    * coefficients in 'surfaces' and 'surfaceCoefficients'. Subtract the found surfaces 
    * from the input cloud and store the remaining cloud in 'cloudMinusSurfaces'.
    */
	virtual void segment(FrameDataPtr frameData);

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
	void cluster(
		PointCloudPtr plane,
		pcl::ModelCoefficients &planeCoefficients,
		FrameDataPtr frameData);

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
	* Project given surfaces on plane specified by the corresponding plane coefficients.
	*/
	void projectOnPlane(SurfaceModelPtr surface);


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
	* Instance used to extract the planes from the input cloud.
	*/
	pcl::SACSegmentation<PointT> segmentation_;
	/**
	* Instance used to extract the actual clusters from the input cloud.
	*/
	pcl::EuclideanClusterExtraction<PointT> clusterizer_;

	/*Segmentation ratio*/
	const double MIN_FILTER_PERCENTAGE;

	// boolean indicating whether the surface detector was activated in config file
	bool surfaceDetectorActive;
};

template<class PointT>
SurfaceSegmenter<PointT>::SurfaceSegmenter(bool surfaceDetectorActive) :
        MIN_FILTER_PERCENTAGE(0.1),
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
void SurfaceSegmenter<PointT>::addNonGroundPlanes(
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
void SurfaceSegmenter<PointT>::getSurfaceClusters(
		PointCloudPtr const& cloud, std::vector<pcl::PointIndices> &cluster_indices) {
	// Extract the clusters from such a filtered cloud.
	int max_size=cloud->points.size();
	clusterizer_.setClusterTolerance(0.03);
	clusterizer_.setMinClusterSize(2300);
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
	PointCloudPtr plane,
	pcl::ModelCoefficients &planeCoefficients,
	FrameDataPtr frameData) 
{
	// A classified surface may consist of different clusters. 
	// Get point indices of points belonging to the same cluster.
    std::vector<pcl::PointIndices> cluster_indices;
    getSurfaceClusters(plane, cluster_indices);

    // cluster the current plane into seperate surfaces
    for (int j = 0; j < cluster_indices.size(); j++)
    {
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
		frameData->surfaces.push_back(SurfaceModelPtr(new SurfaceModel(current, planeCoefficients)));
    }
}


template<class PointT>
void SurfaceSegmenter<PointT>::projectOnPlane(SurfaceModelPtr surface)
{
	// project surface on corresponding plane
	pcl::ProjectInliers<PointT> proj;
	proj.setModelType (pcl::SACMODEL_PLANE);
	proj.setInputCloud (surface->get_cloud());
	pcl::ModelCoefficients::Ptr coeffPtr = boost::shared_ptr<pcl::ModelCoefficients>(
		new pcl::ModelCoefficients(surface->get_planeCoefficients()));
	proj.setModelCoefficients (coeffPtr);
	PointCloudPtr tmp(new PointCloudT());
	proj.filter (*tmp);
	surface->set_cloud(tmp);
}


template<class PointT>
void SurfaceSegmenter<PointT>::segment(FrameDataPtr frameData) 
{
	frameData->cloudMinusSurfaces = preprocessCloud(frameData->cloud);
	std::vector<PointCloudPtr> planes;
	std::vector<pcl::ModelCoefficients> planeCoefficients;
    // extract those planes that are considered as surfaces and put them in cloud_surfaces_
    findPlanes(frameData->cloudMinusSurfaces, planes, planeCoefficients);

    if (!surfaceDetectorActive)
    {
    	// add all planes beside the ground back to the point cloud
		addNonGroundPlanes(frameData->cloudMinusSurfaces, planes);
		// do not cluster and process planes if surface detector was disabled
    	return;
    }

    // reduce number of points of each plane
    for (size_t i = 0; i < planes.size(); i++)
    {
    	downSample(planes[i]);
    	
    	// cluster planes into seperate surfaces and create SurfaceModels
    	cluster(planes[i], planeCoefficients[i], frameData);
    }
    	
    for (size_t i = 0; i < frameData->surfaces.size(); i++)
    {
    	// project the found surfaces on corresponding plane
   		projectOnPlane(frameData->surfaces[i]);
   	}
}

} // namespace lepp

#endif