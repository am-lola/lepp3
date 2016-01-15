#ifndef LEPP3_CONVEX_HULL_DETECTOR_H__
#define LEPP3_CONVEX_HULL_DETECTOR_H__

#include <vector>
#include "lepp3/Typedefs.hpp"
#include "lepp3/SurfaceAggregator.hpp"
#include "lepp3/ConvexHullAggregator.hpp"
#include <pcl/filters/project_inliers.h>
#include <pcl/surface/convex_hull.h>

class ConvexHullDetector : public SurfaceAggregator<PointT>
{
public:
	void attachConvexHullAggregator(boost::shared_ptr<ConvexHullAggregator<PointT> > aggregator);


private:
	virtual void updateSurfaces(std::vector<PointCloudConstPtr> surfaces,
    	PointCloudPtr &cloudMinusSurfaces, std::vector<pcl::ModelCoefficients> *&surfaceCoefficients);

	void notifyAggregators(std::vector<PointCloudConstPtr> &convexHulls);

	void detectConvexHull(PointCloudConstPtr surface, PointCloudPtr &hull, pcl::ModelCoefficients &coefficients);

	
	std::vector<boost::shared_ptr<ConvexHullAggregator<PointT> > > aggregators;
};



void ConvexHullDetector::attachConvexHullAggregator(boost::shared_ptr<ConvexHullAggregator<PointT> > newAggregator)
{
	aggregators.push_back(newAggregator);
}


void ConvexHullDetector::notifyAggregators(std::vector<PointCloudConstPtr> &convexHulls)
{
	for (int i = 0; i < aggregators.size(); i++)
		aggregators[i]->updateHulls(convexHulls);
}


void ConvexHullDetector::detectConvexHull(PointCloudConstPtr surface, PointCloudPtr &hull, pcl::ModelCoefficients &coefficients)
{
	pcl::ProjectInliers<PointT> proj;
	proj.setModelType (pcl::SACMODEL_PLANE);
	proj.setInputCloud (surface);
	pcl::ModelCoefficients::Ptr coeffPtr = boost::shared_ptr<pcl::ModelCoefficients>(new pcl::ModelCoefficients(coefficients));
	proj.setModelCoefficients (coeffPtr);
	PointCloudPtr tmp(new PointCloudT());
	proj.filter (*tmp);

	hull = boost::shared_ptr<PointCloudT>(new PointCloudT());
	pcl::ConvexHull<PointT> chull;
	chull.setInputCloud (surface);
	chull.reconstruct (*hull);
}



void ConvexHullDetector::updateSurfaces(std::vector<PointCloudConstPtr> surfaces,
    	PointCloudPtr &cloudMinusSurfaces, 
    	std::vector<pcl::ModelCoefficients> *&surfaceCoefficients)
{
	std::vector<PointCloudPtr> convexHulls(surfaces.size());

	cout << surfaceCoefficients->size() << "  " << surfaces.size() << endl;

	for (int i = 0; i < surfaces.size(); i++)
	{
		detectConvexHull(surfaces[i], convexHulls[i], (*surfaceCoefficients)[i]);
		cout << convexHulls[i]->size() << " ";
	}
	cout << endl << endl;

	// cast to const pointers
	std::vector<PointCloudConstPtr> convexHullsConst;
	for (int i = 0; i < convexHulls.size(); i++)
		convexHullsConst.push_back(convexHulls[i]);

	notifyAggregators(convexHullsConst);
}

#endif