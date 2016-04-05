#ifndef LEPP3_SURFACE_DATA_H__
#define LEPP3_SURFACE_DATA_H__

#include <vector>
#include "lepp3/models/SurfaceModel.h"
#include "lepp3/Typedefs.hpp"

namespace lepp
{

struct SurfaceData
{
	SurfaceData(long num) : frameNum(num) {}
	long 								frameNum;
	std::vector<SurfaceModelPtr>		surfaces;
	std::vector<PointCloudPtr> 			planes;
	std::vector<pcl::ModelCoefficients> planeCoefficients;
};



class SurfaceDataObserver 
{
public:
	/**
	* Virtual deconstructor.
	*/
	virtual ~SurfaceDataObserver() {}

	/**
	* Update observer with new surface data.
	*/
	virtual void updateSurfaces(SurfaceDataPtr surfaceData) = 0;
};



class SurfaceDataSubject 
{
public:
	/**
	* Virtual deconstructor.
	*/
	virtual ~SurfaceDataSubject() {}

	/**
	* Attach new observer to observer list.
	*/
	void attachObserver(boost::shared_ptr<SurfaceDataObserver> observer)
	{
	  observers.push_back(observer);
	}

protected:
	/**
	* Notify all attached observers.
	*/
    void notifyObservers(SurfaceDataPtr surfaceData)
    {
	  for (size_t i = 0; i < observers.size(); i++) 
	  {
	    observers[i]->updateSurfaces(surfaceData);
	  }
	}

private:
	// vector holding all attached observers of this subject
	std::vector<boost::shared_ptr<SurfaceDataObserver> > observers;
};

}
#endif
