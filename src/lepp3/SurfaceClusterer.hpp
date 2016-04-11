#ifndef lepp3_SURFACE_CLUSTERER_HPP__
#define lepp3_SURFACE_CLUSTERER_HPP__

#include "lepp3/Typedefs.hpp"
#include "lepp3/SurfaceData.hpp"

#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/project_inliers.h>

#include <omp.h>

namespace lepp {

template<class PointT>
class SurfaceClusterer : public SurfaceDataObserver, public SurfaceDataSubject
{
public:
    SurfaceClusterer(std::vector<double> &clusterParameters) : 
    	CLUSTER_TOLERANCE(clusterParameters[0]),
    	MIN_CLUSTER_SIZE(clusterParameters[1]),
    	VOXEL_SIZE_X(clusterParameters[2]),
    	VOXEL_SIZE_Y(clusterParameters[3]),
    	VOXEL_SIZE_Z(clusterParameters[4])
    {}

    /**
    * Cluster the given surfaces into planes. Store the found surfaces and surface model 
    * coefficients in 'surfaces' and 'surfaceCoefficients'. Subtract the found surfaces 
    * from the input cloud and store the remaining cloud in 'cloudMinusSurfaces'.
    */
	virtual void updateSurfaces(SurfaceDataPtr surfaceData);

private:
	/**
	* Extracts the Euclidean clusters from the given point cloud.
	* Returns a vector where each element represents the pcl::PointIndices
	* instance representing the corresponding cluster.
	*/
    void getSurfaceClusters(PointCloudPtr const& cloud, 
    	std::vector<pcl::PointIndices> &cluster_indices);

	/**
	* A plane might contain several non-connected planes that do not correspond
	* to the same surface. Thus, the planes are clustered into seperate surfaces 
	* in this function if necessary.
	**/
	void cluster(
		PointCloudPtr plane,
		pcl::ModelCoefficients &planeCoefficients,
		std::vector<SurfaceModelPtr> &surfaces);

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

	// constant variables for clustering
	const double CLUSTER_TOLERANCE;
	const int MIN_CLUSTER_SIZE;

	// side length of a voxel when applying voxelgridfilter
	const double VOXEL_SIZE_X;
	const double VOXEL_SIZE_Y;
	const double VOXEL_SIZE_Z;
};


template<class PointT>
void SurfaceClusterer<PointT>::downSample(PointCloudPtr &cloud)
{
  PointCloudPtr cloud_filtered(new PointCloudT());
  pcl::VoxelGrid<PointT> sor;
  sor.setInputCloud (cloud);
  sor.setLeafSize (VOXEL_SIZE_X, VOXEL_SIZE_Y, VOXEL_SIZE_Z);
  sor.filter (*cloud_filtered);
  cloud = cloud_filtered;
}


template<class PointT>
void SurfaceClusterer<PointT>::getSurfaceClusters(
		PointCloudPtr const& cloud, std::vector<pcl::PointIndices> &cluster_indices) {
	// Extract the clusters from such a filtered cloud.
	int max_size=cloud->points.size();
	pcl::EuclideanClusterExtraction<PointT> clusterizer;
	clusterizer.setClusterTolerance(0.03);
	clusterizer.setMinClusterSize(2300);
	clusterizer.setMaxClusterSize(max_size);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr kd_tree_(
			new pcl::search::KdTree<pcl::PointXYZ>);
	kd_tree_->setInputCloud(cloud);
	clusterizer.setSearchMethod(kd_tree_);
	clusterizer.setInputCloud(cloud);
	clusterizer.extract(cluster_indices);
}

template<class PointT>
void SurfaceClusterer<PointT>::projectOnPlane(SurfaceModelPtr surface)
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
void SurfaceClusterer<PointT>::cluster(
	PointCloudPtr plane,
	pcl::ModelCoefficients &planeCoefficients,
	std::vector<SurfaceModelPtr> &surfaces) 
{
	// A classified surface may consist of different clusters. 
	// Get point indices of points belonging to the same cluster.
    std::vector<pcl::PointIndices> cluster_indices;
    getSurfaceClusters(plane, cluster_indices);

    // cluster the current plane into seperate surfaces
   	std::vector<SurfaceModelPtr> clusteredSurfaces;
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
		clusteredSurfaces.push_back(SurfaceModelPtr(new SurfaceModel(current, planeCoefficients)));

		// project the found surfaces on corresponding plane
		projectOnPlane(clusteredSurfaces[j]);
    }

    //add clusetered surfaces to shared frameData variable
#pragma omp critical
	surfaces.insert(std::end(surfaces), std::begin(clusteredSurfaces), std::end(clusteredSurfaces));
}

template<class PointT>
void SurfaceClusterer<PointT>::updateSurfaces(SurfaceDataPtr surfaceData)
{
    // reduce number of points of each plane
#pragma omp parallel for schedule(dynamic,1)
    for (size_t i = 0; i < surfaceData->planes.size(); i++)
    {
    	downSample(surfaceData->planes[i]);

    	// cluster planes into seperate surfaces and create SurfaceModels
    	cluster(surfaceData->planes[i], surfaceData->planeCoefficients[i], surfaceData->surfaces);
    }
    notifyObservers(surfaceData);
}

} // namespace lepp

#endif