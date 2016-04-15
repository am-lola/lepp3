#ifndef SurfaceModel_H_
#define SurfaceModel_H_

#include "lepp3/Typedefs.hpp"
#include "lepp3/models/Coordinate.h"


namespace lepp {

class SurfaceModel;

class SurfaceVisitor 
{
public:
	virtual void visitSurface(SurfaceModel &plane) = 0;
	virtual ~SurfaceVisitor() {}
};

class SurfaceModel
{
public:
	SurfaceModel(PointCloudConstPtr surfaceCloud, pcl::ModelCoefficients planeCoefficients) : 
		cloud(surfaceCloud), 
		planeCoefficients(planeCoefficients), 
		hull(new PointCloudT()),
		id_(0), mh_(-1), colorID_(-1)
	{
		computeCenterpoint();
		computeRadius();
	}

	void accept(SurfaceVisitor &visitor) 
	{
		visitor.visitSurface(*this);
	}

	/**
	* Getters for class variables.
	*/
	Coordinate centerpoint() const {return center;}
	int id() const {return id_;}
	PointCloudConstPtr get_cloud() const {return cloud;}
	PointCloudConstPtr get_hull() const {return hull;}
	const pcl::ModelCoefficients& get_planeCoefficients() const {return planeCoefficients;}
	int get_meshHandle() const {return mh_;}
	double get_radius() const {return radius;}
	int get_colorID() const {return colorID_;}

	/**
	* Setters for class variables.
	*/
	void set_cloud(PointCloudConstPtr &new_cloud) {cloud = new_cloud;}
	void set_cloud(PointCloudPtr &new_cloud) {cloud = new_cloud;}
	void set_hull(PointCloudPtr &new_hull) {hull = new_hull;}
	void set_hull(PointCloudConstPtr &new_hull) {hull = new_hull;}
	void set_id(int id) {id_ = id;}
	void set_planeCoefficients(pcl::ModelCoefficients &new_coefficients) {planeCoefficients = new_coefficients;}
	void set_meshHandle(mesh_handle_t mh) {mh_ = mh;}
	void set_colorID(int id) {colorID_ = id;}

	/**
	* Translate center point by given coordinate.
	*/ 
	void translateCenterPoint(const Coordinate &tranlateVec)
	{
		center = center + tranlateVec;
	}

private:
	int id_;
	mesh_handle_t mh_;
	PointCloudConstPtr cloud;
	pcl::ModelCoefficients planeCoefficients;
	PointCloudConstPtr hull;
	Coordinate center;
	double radius;
	int colorID_;
	
	/**
	* Computer the centerpoint of the current surface cloud.
	*/
	void computeCenterpoint()
	{
		Eigen::Vector4f centroid;
    	pcl::compute3DCentroid (*cloud, centroid);
    	center.x = centroid[0];
    	center.y = centroid[1];
    	center.z = centroid[2];
	}
	
	/**
	* Compute the radius of the surface. The radius is defined by the distance
	* between the surface center point and the point furthest away from it.
	*/
	void computeRadius()
	{
		Eigen::Vector4f maxPoint;
		Eigen::Vector4f centerVec(center.x, center.y, center.z, 0);
		pcl::getMaxDistance (*cloud, centerVec, maxPoint);
		radius = (center.x - maxPoint[0]) * (center.x - maxPoint[0]) 
			+ (center.y - maxPoint[1]) * (center.y - maxPoint[1])
			+ (center.z - maxPoint[2]) * (center.z - maxPoint[2]);
	}
};

}  // namespace lepp

#endif /* SurfaceModel_H_ */