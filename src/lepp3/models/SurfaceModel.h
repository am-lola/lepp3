/*
 * SUrfaceModel.h
 *
 *  Created on: Oct 14, 2015
 *      Author: iuygur
 */

#ifndef SURFACEMODEL_H_
#define SURFACEMODEL_H_
#include <iostream>
#include <pcl/common/pca.h>
#include <pcl/common/common.h>
#include "lepp3/models/ObjectModel.h"
#include "lepp3/Typedefs.hpp"

namespace lepp {

//// Forward declarations.
class PlaneVisitor;
class PlaneModel;


/*Base class for a plane representation*/

class SurfaceModel {
public:
	SurfaceModel() :
			id_(0) {
	}

	virtual Coordinate centerpoint() const = 0;

	  virtual void accept(PlaneVisitor& visitor) = 0;

	virtual ~SurfaceModel() {
	}

	virtual PointCloudConstPtr getCloud() = 0;
	/**
	 * Returns the ID associated with the surface. If the ID is 0, it means that
	 * no meaningful ID was associated.
	 *
	 * Therefore, 0 is the default value returned, unless `set_id` is called.
	 */
	int id() const {
		return id_;
	}
	/**
	 * Sets the object's ID.
	 */
	void set_id(int id) {
		id_ = id;
	}

	friend std::ostream& operator<<(std::ostream& out,
			SurfaceModel const& model);
private:
	int id_;
};

typedef boost::shared_ptr<SurfaceModel> SurfaceModelPtr;

class PlaneVisitor {
public:
	virtual void visitPlane(PlaneModel& plane) = 0;
	virtual ~PlaneVisitor() {}
};

/*Plane model for detected surfaces*/

class PlaneModel: public SurfaceModel {
public:
	PlaneModel(double area, Coordinate const& center,double inclination, PointCloudConstPtr &cloud);
	/**
	 * Returns a model-specific representation of its coefficients packed into
	 * an std::vector.
	 */
	//void accept(ModelVisitor& visitor) { visitor.visitSphere(*this); }

	double area() const {
		return area_;
	}
	Coordinate centerpoint() const {
		return center_;
	}

	PointCloudConstPtr getCloud(){
		return cloud_;
	}

	void set_area(double area) {
		area_ = area;
	}
	void set_center(Coordinate const& center) {
		center_ = center;
	}
	void set_inclination(double amount) {
		inclination_ = amount;
	}
	void accept(PlaneVisitor & visitor) { visitor.visitPlane(*this); }

	  Coordinate center_point() const { return center_; }

	friend std::ostream& operator<<(std::ostream& out, PlaneModel const& plane);

private:
	double area_;
	Coordinate center_;
	double inclination_;
	PointCloudConstPtr cloud_;

};

inline PlaneModel::PlaneModel(double area, Coordinate const& center,
		double inclination, PointCloudConstPtr &cloud) :
		area_(area), center_(center), inclination_(inclination), cloud_(cloud) {
}

inline std::ostream& operator<<(std::ostream& out, PlaneModel const& plane) {
	out << "[plane; " << "area = " << plane.area_ << "; " << "center = "
			<< plane.center_ << ";" << "inclination = " << plane.inclination_
			<< " ]";

	return out;
}




class FlattenPlaneVisitor : public PlaneVisitor {
public:
  void visitPlane(PlaneModel& plane) { planes_.push_back(&plane); }
  std::vector<SurfaceModel*> const& getPlanes() const { return planes_; }
private:
  std::vector<SurfaceModel*> planes_;
};

class PrintPlaneVisitor : public PlaneVisitor {
public:
  /**
   * Create a new `PrintVisitor` that will output in the given `std::ostream`.
   */
  PrintPlaneVisitor(std::ostream& out) : out_(out) {}

  void visitPlane(PlaneModel& plane) { out_ << plane; }
private:
  std::ostream& out_;
};

inline std::ostream& operator<<(std::ostream& out, SurfaceModel const& plane_model) {
  PrintPlaneVisitor printer(out);
  const_cast<SurfaceModel&>(plane_model).accept(printer);

  return out;
}

}  // namespace lepp

#endif /* SURFACEMODEL_H_ */
