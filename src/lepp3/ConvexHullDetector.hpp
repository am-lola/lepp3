#ifndef LEPP3_CONVEX_HULL_DETECTOR_H__
#define LEPP3_CONVEX_HULL_DETECTOR_H__

#include <vector>
#include "lepp3/Typedefs.hpp"
#include "lepp3/SurfaceAggregator.hpp"
#include <pcl/filters/project_inliers.h>
#include <pcl/surface/convex_hull.h>

class ConvexHullDetector : public SurfaceAggregator<PointT>
{
public:


private:
	virtual void updateSurfaces(std::vector<PointCloundConstPtr> surfaces,
    	PointCloudPtr &cloudMinusSurfaces, std::vector<pcl::ModelCoefficients> *&surfaceCoefficients);

	void detectConvexHull(PointCloundConstPtr surface, PointCloudPtr &hull, pcl::ModelCoefficients &coefficients);



	std::vector<PointCloudPtr> convexHulls;
};



void ConvexHullDetector::detectConvexHull(PointCloundConstPtr surface, PointCloudPtr &hull, pcl::ModelCoefficients &coefficients)
{
	
	pcl::ProjectInliers<PointT> proj;
	proj.setModelType (pcl::SACMODEL_PLANE);
	proj.setInputCloud (surface);


/*
	pcl::ModelCoefficients::Ptr coeffPtr = boost::shared_ptr<pcl::ModelCoefficients>(&coefficients);
	


	proj.setModelCoefficients (coeffPtr);
	PointCloudPtr tmp(new PointCloudT());
	proj.filter (*tmp);

	hull = boost::shared_ptr<PointCloudT>(new PointCloudT());
	pcl::ConvexHull<PointT> chull;
	chull.setInputCloud (tmp);
	chull.reconstruct (*hull);*/
}



void ConvexHullDetector::updateSurfaces(std::vector<PointCloundConstPtr> surfaces,
    	PointCloudPtr &cloudMinusSurfaces, std::vector<pcl::ModelCoefficients> *&surfaceCoefficients)
{
	
	convexHulls.resize(surfaces.size());
	for (int i = 0; i < surfaces.size(); i++)
	{

		//cout << (*surfaceCoefficients)[i].values.size() << endl;
		//detectConvexHull(surfaces[i], convexHulls[i], (*surfaceCoefficients)[i]);
	}
}





#endif