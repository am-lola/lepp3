#ifndef LEPP3_FRAME_DATA_H__
#define LEPP3_FRAME_DATA_H__

#include <vector>
#include "lepp3/models/SurfaceModel.h"
#include "lepp3/models/ObjectModel.h"
#include "lepp3/Typedefs.hpp"

namespace lepp
{

struct FrameData
{
	FrameData(long num) : frameNum(num), 
		cloudMinusSurfaces(new PointCloudT()), 
		surfaceFrameNum(-1) {}
	long 							frameNum;
	long 							surfaceFrameNum;
	long							surfaceReferenceFrameNum;
	PointCloudConstPtr				cloud;
	PointCloudPtr					cloudMinusSurfaces;
	std::vector<SurfaceModelPtr>	surfaces;
	std::vector<ObjectModelPtr> 	obstacles;
	std::vector<PointCloudPtr>		obstacleClouds;
};



class FrameDataObserver 
{
public:
	/**
	* Virtual deconstructor.
	*/
	virtual ~FrameDataObserver() {}

	/**
	* Update observer with new frame data.
	*/
	virtual void updateFrame(FrameDataPtr frameData) = 0;
};



class FrameDataSubject 
{
public:
	/**
	* Virtual deconstructor.
	*/
	virtual ~FrameDataSubject() {}

	/**
	* Attach new observer to observer list.
	*/
	void attachObserver(boost::shared_ptr<FrameDataObserver> observer)
	{
	  observers.push_back(observer);
	}

protected:
	/**
	* Notify all attached observers.
	*/
    void notifyObservers(FrameDataPtr frameData)
    {
	  for (size_t i = 0; i < observers.size(); i++) 
	  {
	    observers[i]->updateFrame(frameData);
	  }
	}

private:
	// vector holding all attached observers of this subject
	std::vector<boost::shared_ptr<FrameDataObserver> > observers;
};

}
#endif
