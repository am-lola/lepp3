#ifndef LEPP3_CONVEX_HULL_DETECTOR_H__
#define LEPP3_CONVEX_HULL_DETECTOR_H__

#include "lepp3/Typedefs.hpp"
#include "lepp3/SurfaceData.hpp"
#include "lepp3/GnuplotWriter.hpp"
#include <pcl/surface/convex_hull.h>
#include <pcl/filters/project_inliers.h>
#include <set>
#include <algorithm>
#include <limits>
#include <vector>

namespace lepp {

/*
* Triangle class that is needed to reduce the number of points of the convex hull to 8 points. 
*/
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

	/*
	* Triangles are compared by their area. If the area is the same the triangles' unique identifier is compared.
	* This is necessary because Triangles are later inserted into a priority queue that relies on sets, and in
	* sets there can only exist one single object for each key.
	*/
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

	/*
	* Set neighboring triangles in convex hull.
	*/
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
};



/*
* Priority queue for Triangle pointers. The standard priority queue of C++ does not have a change key 
* and also no delete operation.
* Thus, we use sets as an underlying data structure and implement the change key operation by deleting the old element
* changing the key of the deleted element, and re-insert it. This still runs is O(log(n)) since sets in C++
* are implemented with red-black-trees as an underlying data structure.
*/
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
};



/*
* The ConvexHullDetector is a surface aggregator. It gets point clouds that represent the detected surfaces.
* For each surface then it is computing the convex hull, and in a second step reduces the number of points
* of the convex hull to a user defined number.
*/
class ConvexHullDetector : public SurfaceDataObserver, public SurfaceDataSubject
{
public:
	// inherited from the SurfaceAggregator interface
	virtual void updateSurfaces(SurfaceDataPtr surfaceData);

	/**
	* Return squared euclidean length of given vector.
	*/
	static double getVecLengthSquared(const PointT &p);

	/**
	* Projects the point p onto line segment seg1-seg2, and stores the vector from p to its projection in 'projVec'.
	*/
	static void projectPointOntoLineSegment(const PointT &seg1, const PointT &seg2, const PointT &p, PointT &projVec);

private:
	static constexpr int NUM_HULL_POINTS = 8;
	static constexpr double MERGE_UPDATE_PERCENTAGE = 0.8;

	// reduce the number of points in the given hull to 'numPoints'
	void reduceConvHullPoints(PointCloudPtr &hull, int numPoints);

	// uses the pcl function to compute the convex hull of the given point cloud.
	void detectConvexHull(PointCloudConstPtr surface, PointCloudPtr &hull);



	/**
	* Function gets a point cloud and a convex hull. It projects each point of the given cloud onto the 
	* closest position of the border of the given convex hull. Each point of the given point cloud is then
	* updated by moving it 'updatePercentage' percents in direction of its projection.
	* The projected points are stored in 'projCloud'.
	*/
	void projectCloudOntoHull(PointCloudConstPtr cloud, PointCloudConstPtr hull, PointCloudPtr &projCloud, double updatePercentage);

	/**
	* Function gets old and new convex hull of surface point cloud. It 'merges' convex hulls of 2 consecutive frames.
	* To do so every point of the new hull is projected onto the closest position of the old convex hull, and updated
	* by moving MERGE_UPDATE_PERCENTAGE in direction of its projection.
	* Conversely, every point of the old hull is projected onto the closest position of the old convex hull,
	* and updated by moving MERGE_UPDATE_PERCENTAGE in direction of its projection.
	* Then, both projected point clouds are merged and a new convex hull is computed for this point cloud. 
	* Finally, this cloud is stored in 'mergeHull'.
	*/
	void mergeConvexHulls(PointCloudConstPtr oldHull, PointCloudConstPtr newHull, PointCloudPtr &mergeHull);

	/**
	* Project the given point cloud onto the surface specified by the surfaceCoefficients.
	*/
	void projectOnPlane(PointCloudConstPtr cloud,
		const pcl::ModelCoefficients &surfaceCoefficients,
		PointCloudPtr &projCloud);
};



// use PCl to detect the convex hull of the given point cloud
void ConvexHullDetector::detectConvexHull(PointCloudConstPtr surface, PointCloudPtr &hull)
{
	hull = boost::shared_ptr<PointCloudT>(new PointCloudT());
	pcl::ConvexHull<PointT> chull;
	chull.setInputCloud (surface);
	chull.reconstruct (*hull);
}


/*
* Reduces the number of points of the given convex hull to 'numPoints'.
* Algorithm works as follows:
* For each three neighboring points ('left', 'mid', 'right') of the point cloud, compute the area of the formed triangle.
* Delete the triangle with the smallest area, and the corresponding point 'mid'.
* Update the area of the triangles that were neighbored to the deleted triangle.
* Continue this process until there are only 'numPoints' left.
* If given hull has less than 'numPoints' points, return it without running the algorithm.
*/
void ConvexHullDetector::reduceConvHullPoints(PointCloudPtr &hull, int numPoints)
{
	// return if number of points in the given hull is smaller than 'numPoints'
	int numHullPoints = hull->size();
	if (numHullPoints <= numPoints)
		return;

	// priority queue with triangles. Triangles are sorted by their area size, smalles area has highest priority.
	PrioQ pq(Triangle::compare);
	Triangle *triangleRef[numHullPoints];
	for (int i = 0; i < numHullPoints; i++)
	{
		triangleRef[i] = new Triangle(i, &(hull->at((numHullPoints+i-1) % numHullPoints)), &(hull->at(i)), &(hull->at((i+1) % numHullPoints)));
		pq.insert(triangleRef[i]);
	}

	// set up triangle data structure
	for (int i = 0; i < numHullPoints; i++)
		triangleRef[i]->initNeighborTriangles(triangleRef[(numHullPoints+i-1) % numHullPoints], triangleRef[(i+1) % numHullPoints]);
	
	// delete triangles until there are only 'numPoints' left
	for (int i = numHullPoints; i > numPoints; i--)
	{
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
	}

	// create point cloud only with the non-deleted 'mid' points.
	PointCloudPtr smallHull = boost::shared_ptr<PointCloudT>(new PointCloudT());
	Triangle *tri = pq.deleteMin();
	for (int i = 0; i < numPoints; i++)
	{
		smallHull->push_back(*(tri->mid));
		tri = tri->leftTriangle;
	}
	hull = smallHull;

	// delete all triangles
	for (int i = 0; i < numHullPoints; i++)
		delete triangleRef[i];
}


void ConvexHullDetector::projectOnPlane(
	PointCloudConstPtr cloud,
	const pcl::ModelCoefficients &surfaceCoefficients,
	PointCloudPtr &projCloud)
{
	if (cloud->size() != 0)
	{
		pcl::ProjectInliers<PointT> proj;
		proj.setModelType(pcl::SACMODEL_PLANE);
		proj.setInputCloud(cloud);
		pcl::ModelCoefficients::Ptr coeffPtr = boost::shared_ptr<pcl::ModelCoefficients>(new pcl::ModelCoefficients(surfaceCoefficients));
		proj.setModelCoefficients(coeffPtr);
		proj.filter(*projCloud);
	}
}


double ConvexHullDetector::getVecLengthSquared(const PointT &p)
{
	return p.x * p.x + p.y * p.y + p.z * p.z;
}

void ConvexHullDetector::projectPointOntoLineSegment(const PointT &seg1, const PointT &seg2, const PointT &p, PointT &projVec)
{
	// vector pointing along the line segment
	PointT segment(seg2.x - seg1.x, seg2.y - seg1.y, seg2.z - seg1.z);
	// vector pointing from seg1 to given new hull point
	PointT seg2Point(p.x - seg1.x, p.y - seg1.y, p.z - seg1.z);
	double segLen = getVecLengthSquared(segment);
	// t is on [0,1]. It gives the procentual position of projected point p between seg1 and seg2
	double t = std::max(0.0, std::min(1.0, (segment.x * seg2Point.x + segment.y * seg2Point.y + segment.z * seg2Point.z) / segLen));

	// compute vector between given point of new hull and its projection on line segment seg1->seg2
	projVec.x = seg1.x + t * segment.x - p.x;
	projVec.y = seg1.y + t * segment.y - p.y;
	projVec.z = seg1.z + t * segment.z - p.z;
}


void ConvexHullDetector::projectCloudOntoHull(PointCloudConstPtr cloud, PointCloudConstPtr hull, PointCloudPtr &projCloud, double updatePercentage)
{
	for (int i = 0; i < cloud->size(); i++)
	{
		PointT shortestProjVec;
		double shortestDist = std::numeric_limits<double>::max();
		const PointT &currentPoint = cloud->at(i);

		// find shortest distance between point of cloud and boundary of the hull
		// iterate over all line segments of the hull
		for (int j = 0; j < hull->size(); j++)
		{
			// compute distance between point of cloud and all line segments of the hull and take minimum
			PointT projVec;
			projectPointOntoLineSegment(hull->at(j), hull->at((j+1) % hull->size()), currentPoint, projVec);
			double projVecDist = getVecLengthSquared(projVec);
			if (projVecDist < shortestDist)
			{
				shortestDist = projVecDist;
				shortestProjVec.x = projVec.x;
				shortestProjVec.y = projVec.y;
				shortestProjVec.z = projVec.z;
			}
		}

		if (hull->size() != 0)
		{
			// go from point of cloud 'updatePercentage' percent in direction of its projection onto the hull
			PointT updatedPoint;
			updatedPoint.x = currentPoint.x + updatePercentage * shortestProjVec.x;
			updatedPoint.y = currentPoint.y + updatePercentage * shortestProjVec.y;
			updatedPoint.z = currentPoint.z + updatePercentage * shortestProjVec.z;
			projCloud->push_back(updatedPoint);
		}
		else
		{
			projCloud->push_back(currentPoint);
		}
	}
}


void ConvexHullDetector::mergeConvexHulls(PointCloudConstPtr oldHull, PointCloudConstPtr newHull, PointCloudPtr &mergeHull)
{
	// Project points of old hull onto new hull
	PointCloudPtr projNewOntoOld(new PointCloudT());
	projectCloudOntoHull(newHull, oldHull, projNewOntoOld, MERGE_UPDATE_PERCENTAGE);

	// Project points of new hull onto old hull
	PointCloudPtr projOldOntoNew(new PointCloudT());
	projectCloudOntoHull(oldHull, newHull, projOldOntoNew, 1-MERGE_UPDATE_PERCENTAGE);

	// merge both projections
	PointCloudPtr combinedProj(new PointCloudT());
	*combinedProj += *projNewOntoOld;
	*combinedProj += *projOldOntoNew;

	// compute convex hull of combined projection and reduce point size
	detectConvexHull(combinedProj, mergeHull);
	reduceConvHullPoints(mergeHull, NUM_HULL_POINTS);
}


void ConvexHullDetector::updateSurfaces(SurfaceDataPtr surfaceData)
{
	for (int i = 0; i < surfaceData->surfaces.size(); i++)
	{
		// detect new convex hull
		PointCloudPtr newHull(new PointCloudT());
		detectConvexHull(surfaceData->surfaces[i]->get_cloud(), newHull);
		reduceConvHullPoints(newHull, NUM_HULL_POINTS);

		// project old and new hull onto the same surface
		PointCloudPtr projNewHull(new PointCloudT());
		PointCloudPtr projOldHull(new PointCloudT());
		projectOnPlane(PointCloudConstPtr(newHull), surfaceData->surfaces[i]->get_planeCoefficients(), projNewHull);
		projectOnPlane(surfaceData->surfaces[i]->get_hull(), surfaceData->surfaces[i]->get_planeCoefficients(), projOldHull);


		// merge convex hull with old convex hull of same surface. If the surface is detected for the first time,
		// simply take the new cloud.
		PointCloudPtr mergeHull(new PointCloudT());
		mergeConvexHulls(projOldHull, projNewHull, mergeHull);
		surfaceData->surfaces[i]->set_hull(mergeHull);

		// write out files
		//GnuplotWriter::writeHulls(surfaceData->frameNum, i, PointCloudConstPtr(projOldHull), 
		//	PointCloudConstPtr(projNewHull), surfaceData->surfaces[i]->get_hull());		
	}
	notifyObservers(surfaceData);
}


}// namespace lepp
#endif