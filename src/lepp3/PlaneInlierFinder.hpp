#ifndef PLANE_INLIER_FINDER_H_
#define PLANE_INLIER_FINDER_H_

#include "lepp3/Typedefs.hpp"
#include "lepp3/FrameData.hpp"

#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/ModelCoefficients.h>

#include <vector>
#include <omp.h>

#ifdef LEPP3_ENABLE_TRACING
#include "lepp3/util/lepp3_tracepoint_provider.hpp"
#endif

namespace lepp {

template<class PointT>
class PlaneInlierFinder : public FrameDataObserver, public FrameDataSubject
{
public:
	PlaneInlierFinder(double min_distance_to_plane) :
		MIN_DIST_TO_PLANE(min_distance_to_plane) {}

	PlaneInlierFinder(double min_distance_to_plane, double max_dist_from_odo) :
		MIN_DIST_TO_PLANE(min_distance_to_plane), MAX_DIST_FROM_ODO(max_dist_from_odo)
		 { apply_max_dist_ = true; }

	/**
	* Update observer with new frame data.
	*/
	virtual void updateFrame(FrameDataPtr frameData);

private:
	const double MIN_DIST_TO_PLANE;
	double MAX_DIST_FROM_ODO;
	bool apply_max_dist_ = false;

	/**
	* Filter out all points that belong to a plane in the given cloud.
	* Remove those points from the cloud and save the resulting cloud in cloudMinusSurfaces.
	*/
	void filterInliers(PointCloudConstPtr cloud, std::vector<pcl::ModelCoefficients>
		&planeCoefficients, PointCloudPtr &cloudMinusSurfaces,
	  const std::shared_ptr<lepp::LolaKinematicsParams> &lolaKinematics);

};


template<class PointT>
void PlaneInlierFinder<PointT>::filterInliers(PointCloudConstPtr cloud,
	std::vector<pcl::ModelCoefficients> &planeCoefficients, PointCloudPtr &cloudMinusSurfaces, const std::shared_ptr<lepp::LolaKinematicsParams> &lolaKinematics)
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
		// points that are too far away from lola's coordinate center are removed
		// works similar to the bubble, only in the obstacle thread
		Eigen::Vector3f odo_pos;
		if (apply_max_dist_)
		{
			odo_pos = lepp::PoseService::getRobotPosition(*lolaKinematics);
		}

		#pragma omp for schedule(guided)
		for (size_t i = 0; i < cloud->size(); i++)
		{
			const PointT &p = cloud->at(i);

			// points that are too far away from lola's coordinate center are removed
			// works similar to the bubble, only in the obstacle thread
			if (apply_max_dist_)
			{
				Eigen::Vector3f point_pos(p.x, p.y, p.z);
				if ((point_pos - odo_pos).norm() > MAX_DIST_FROM_ODO)
				{
					threadPlaneIndices.push_back(i);
					continue;
				}
			}

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
#ifdef LEPP3_ENABLE_TRACING
    tracepoint(lepp3_trace_provider, plane_inlier_update_start);
#endif

	if (frameData->cloud->size() > 0)
		filterInliers(frameData->cloud, frameData->planeCoefficients, frameData->cloudMinusSurfaces, frameData->lolaKinematics);
	notifyObservers(frameData);

#ifdef LEPP3_ENABLE_TRACING
    tracepoint(lepp3_trace_provider, plane_inlier_update_end);
#endif
}


}
#endif
