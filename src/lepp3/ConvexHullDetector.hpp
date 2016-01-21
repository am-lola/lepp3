#ifndef LEPP3_CONVEX_HULL_DETECTOR_H__
#define LEPP3_CONVEX_HULL_DETECTOR_H__

#include <vector>
#include "lepp3/Typedefs.hpp"
#include "lepp3/SurfaceAggregator.hpp"
#include "lepp3/ConvexHullAggregator.hpp"
#include <pcl/filters/project_inliers.h>
#include <pcl/surface/convex_hull.h>
#include <set>
#include <algorithm>

struct Triangle
{	
	int num;
	PointT 	*left, *mid, *right;
	Triangle *leftTriangle, *rightTriangle;
	double	area;

	Triangle(int num, PointT *left, PointT *mid, PointT *right) : num(num), left(left), mid(mid), right(right)
	{
		computeArea();
	}

	static bool compare(Triangle *t1, Triangle *t2)
	{
		if (t1->area < t2->area)
			return true;
		else if (t1->area > t2->area)
			return false;
		else if (t1->num < t2->num)
			return true;
		else
			return false;
	}

	void initNeighborTriangles(Triangle *leftT, Triangle *rightT)
	{
		leftTriangle = leftT;
		rightTriangle = rightT;
	}

	void computeArea()
	{
		PointT a, b;
		a.x = left->x - mid->x;
		a.y = left->y - mid->y;
		a.z = left->z - mid->z;

		b.x = right->x - mid->x;
		b.y = right->y - mid->y;
		b.z = right->z - mid->z;
	
		area = (a.y * b.z - a.z * b.y) * (a.y * b.z - a.z * b.y) +
			(a.z * b.x - a.x * b.z) * (a.z * b.x - a.x * b.z) +
			(a.x * b.y - a.y * b.x) * (a.x * b.y - a.y * b.x);
	}

	void print()
	{
		cout << "Triangle " << num << "   Area: " << area << "   Neighbor Left: " << 
			leftTriangle->num << "   Neighbor Right: " << rightTriangle->num << endl;
	}
};



class PrioQ
{
private:
	std::set<Triangle *, bool (*)(Triangle *, Triangle *)> q;
public:
	PrioQ(bool (*func)(Triangle *, Triangle *)) : q(func) {}
	void insert(Triangle *t)
	{
		q.insert(t);
	}

	Triangle* deleteMin()
	{
		Triangle *res = *(q.begin());
		q.erase(q.begin());
		return res;
	}

	void changeKey(Triangle *t)
	{
		q.erase(t);
		t->computeArea();
		q.insert(t);
	}

	bool isEmpty()
	{
		return q.empty();
	}

	void print()
	{
		std::set<Triangle *, bool (*)(Triangle *, Triangle *)>::iterator it;
		for (it = q.begin(); it != q.end(); it++)
			(*it)->print();
	}

};




class ConvexHullDetector : public SurfaceAggregator<PointT>
{
public:
	void attachConvexHullAggregator(boost::shared_ptr<ConvexHullAggregator<PointT> > aggregator);


	virtual void updateSurfaces(std::vector<PointCloudConstPtr> surfaces,
    	PointCloudPtr &cloudMinusSurfaces, std::vector<pcl::ModelCoefficients> *&surfaceCoefficients);

	void notifyAggregators(std::vector<PointCloudConstPtr> &convexHulls);

	void detectConvexHull(PointCloudConstPtr surface, PointCloudPtr &hull, pcl::ModelCoefficients &coefficients);

	void reduceConvHullPoints(PointCloudPtr &hull, int numPoints);
	
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


void ConvexHullDetector::reduceConvHullPoints(PointCloudPtr &hull, int numPoints)
{
	int numHullPoints = hull->size();
	if (numHullPoints <= numPoints)
		return;

	PrioQ pq(Triangle::compare);
	Triangle *triangleRef[numHullPoints];
	for (int i = 0; i < numHullPoints; i++)
	{
		triangleRef[i] = new Triangle(i, &(hull->at((numHullPoints+i-1) % numHullPoints)), &(hull->at(i)), &(hull->at((i+1) % numHullPoints)));
		pq.insert(triangleRef[i]);
	}

	for (int i = 0; i < numHullPoints; i++)
		triangleRef[i]->initNeighborTriangles(triangleRef[(numHullPoints+i-1) % numHullPoints], triangleRef[(i+1) % numHullPoints]);
	

	for (int i = numHullPoints; i > numPoints; i--)
	{
		/*
		cout << endl << endl;
		cout << "QUEUE BEFORE" << endl;
		pq.print();*/


		Triangle *toBeDeletedTriangle = pq.deleteMin();
		Triangle *leftTri = toBeDeletedTriangle->leftTriangle;
		Triangle *rightTri = toBeDeletedTriangle->rightTriangle;

		// update left and right triangle neighbors of neighboring triangles
		leftTri->rightTriangle = toBeDeletedTriangle->rightTriangle;
		rightTri->leftTriangle = toBeDeletedTriangle->leftTriangle;

		// update left and right points of neighboring triangles
		leftTri->right = toBeDeletedTriangle->right;
		rightTri->left = toBeDeletedTriangle->left;

		// update triangles in priority queue
		pq.changeKey(leftTri);
		pq.changeKey(rightTri);

/*
		cout << endl << endl;
		cout << "QUEUE AFTER" << endl;
		pq.print();
		cout << endl << endl;	*/	
	}


	PointCloudPtr smallHull = boost::shared_ptr<PointCloudT>(new PointCloudT());
	Triangle *tri = pq.deleteMin();
	for (int i = 0; i < numPoints; i++)
	{
		smallHull->push_back(*(tri->mid));
		tri = tri->leftTriangle;
	}
	hull = smallHull;
}



void ConvexHullDetector::updateSurfaces(std::vector<PointCloudConstPtr> surfaces,
    	PointCloudPtr &cloudMinusSurfaces, 
    	std::vector<pcl::ModelCoefficients> *&surfaceCoefficients)
{
	std::vector<PointCloudPtr> convexHulls(surfaces.size());

	//cout << surfaceCoefficients->size() << "  " << surfaces.size() << endl;

	for (int i = 0; i < surfaces.size(); i++)
	{
		detectConvexHull(surfaces[i], convexHulls[i], (*surfaceCoefficients)[i]);
		reduceConvHullPoints(convexHulls[i], 8);
/*
		cout << "HULL: " << i << endl;
		for (int j = 0; j < convexHulls[i]->size(); j++)
		{
			cout << convexHulls[i]->at(j).x << "  " << convexHulls[i]->at(j).y << "  " << convexHulls[i]->at(j).z << "  " << endl;
		}
*/

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