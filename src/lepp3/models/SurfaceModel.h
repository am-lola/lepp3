#ifndef SurfaceModel_H_
#define SurfaceModel_H_

#include "lepp3/Typedefs.hpp"
#include "lepp3/models/Coordinate.h"

namespace lepp {

// forward declaration needed for PlaneVisitor
class SurfaceModel;

class PlaneVisitor {
public:
	virtual void visitPlane(SurfaceModel &plane) = 0;
	virtual ~PlaneVisitor() {
	}
};

class SurfaceModel {
public:
	SurfaceModel(PointCloudConstPtr surfaceCloud, pcl::ModelCoefficients planeCoefficients) : 
		cloud(surfaceCloud), 
		planeCoefficients(planeCoefficients), 
		hull(new PointCloudT()),
		id_(0) 
	{
		computeCenterpoint();
	}

	void accept(PlaneVisitor & visitor) 
	{
		visitor.visitPlane(*this);
	}

	/**
	* Getters for class variables.
	*/
	Coordinate centerpoint() const {return center;}
	int id() const {return id_;}
	PointCloudConstPtr get_cloud() const {return cloud;}
	PointCloudConstPtr get_hull() const {return hull;}
	const pcl::ModelCoefficients& get_planeCoefficients() const {return planeCoefficients;}

	/**
	* Setters for class variables.
	*/
	void set_cloud(PointCloudConstPtr &new_cloud) {cloud = new_cloud;}
	void set_cloud(PointCloudPtr &new_cloud) {cloud = new_cloud;}
	void set_hull(PointCloudPtr &new_hull) {hull = new_hull;}
	void set_id(int id) {id_ = id;}
	void set_planeCoefficients(pcl::ModelCoefficients &new_coefficients) {planeCoefficients = new_coefficients;}

	/**
	* Translate center point by given coordinate.
	*/ 
	void translateCenterPoint(const Coordinate &tranlateVec);

private:
	int id_;
	PointCloudConstPtr cloud;
	pcl::ModelCoefficients planeCoefficients;
	PointCloudConstPtr hull;
	Coordinate center;
	PointT minPoint;
	PointT maxPoint;

	/**
	* Computer the centerpoint of the current surface cloud.
	*/
	void computeCenterpoint();
};

void SurfaceModel::computeCenterpoint()
{
	pcl::getMinMax3D(*cloud, minPoint, maxPoint);
	center.x = (minPoint.x + maxPoint.x) / 2;
	center.y = (minPoint.y + maxPoint.y) / 2;
	center.z = (minPoint.z + maxPoint.z) / 2;
}

void SurfaceModel::translateCenterPoint(const Coordinate &tranlateVec)
{
	center = center + tranlateVec;
}

}  // namespace lepp

#endif /* SurfaceModel_H_ */