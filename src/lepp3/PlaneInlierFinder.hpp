#ifndef PLANE_INLIER_FINDER_H_
#define PLANE_INLIER_FINDER_H_

#include "lepp3/Typedefs.hpp"
#include "lepp3/FrameData.hpp"

#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/ModelCoefficients.h>

#include <vector>
#include <omp.h>

namespace lepp {

template<class PointT>
class PlaneInlierFinder : public FrameDataObserver, public FrameDataSubject
{
public:
	PlaneInlierFinder(std::vector<double> planeInlierFinderParameters) : 
		MIN_DIST_TO_PLANE(planeInlierFinderParameters[0]) {}

	/**
	* Update observer with new frame data.
	*/
	virtual void updateFrame(FrameDataPtr frameData);

private:
	const double MIN_DIST_TO_PLANE;

	/**
	* Filter out all points that belong to a plane in the given cloud. 
	* Remove those points from the cloud and save the resulting cloud in cloudMinusSurfaces.
	*/
	void filterInliers(PointCloudConstPtr cloud, std::vector<pcl::ModelCoefficients> 
		&planeCoefficients, PointCloudPtr &cloudMinusSurfaces);

};


template<class PointT>
void PlaneInlierFinder<PointT>::filterInliers(PointCloudConstPtr cloud, 
	std::vector<pcl::ModelCoefficients> &planeCoefficients, PointCloudPtr &cloudMinusSurfaces)
{
	/*
	std::vector<double> scaledThresholds;
	for (size_t i = 0; i < planeCoefficients.size(); ++i)
	{
		pcl::ModelCoefficients &coeffs = planeCoefficients[i];
		scaledThresholds.push_back(coeffs.values[0]*coeffs.values coeffs.values[1], coeffs.values[2]));
	}*/

	std::vector<int> planeIndices;
	// iterate over all points of point cloud
	#pragma omp parallel
	{
		std::vector<int> threadPlaneIndices;
		#pragma omp for schedule(guided)
		for (size_t i = 0; i < cloud->size(); i++)
		{
			const PointT &p = cloud->at(i);
			// iterate over all planes
			for (size_t j = 0; j < planeCoefficients.size(); j++)
			{
				// compute distance of point to the currently considered plane
				pcl::ModelCoefficients &coeffs = planeCoefficients[j];
				double dist = pointToPlaneDistance(p, coeffs.values[0], 
					coeffs.values[1], coeffs.values[2], coeffs.values[3]);
				if (dist < MIN_DIST_TO_PLANE)
				{
					// push cloud index of a point that belongs to a plane into a vector
					threadPlaneIndices.push_back(i);
					break;
				}
			}
		}

		// every thread copies his own thread indices into the global plane indices
		#pragma omp critical
		{
			planeIndices.insert(std::end(planeIndices), std::begin(threadPlaneIndices), std::end(threadPlaneIndices));
		}
	}	

	// filter out all points from the cloud that belong to a plane
	pcl::ExtractIndices<PointT> extract;
	pcl::PointIndices::Ptr allIndices(new pcl::PointIndices);
	allIndices->indices = planeIndices;
	extract.setInputCloud(cloud);
	extract.setIndices(allIndices);
	extract.setNegative(true);
	// store the filered cloud in cloudMinusSurfaces
	extract.filter(*cloudMinusSurfaces);
}


template<class PointT>
void PlaneInlierFinder<PointT>::updateFrame(FrameDataPtr frameData)
{
	if (frameData->cloud->size() > 0)
		filterInliers(frameData->cloud, frameData->planeCoefficients, frameData->cloudMinusSurfaces);
	notifyObservers(frameData);
}


}
#endif
