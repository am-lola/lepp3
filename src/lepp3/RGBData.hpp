#ifndef LEPP3_RGB_DATA_H__
#define LEPP3_RGB_DATA_H__

#include <opencv2/core/mat.hpp>
#include <vector>
#include "lepp3/Typedefs.hpp"

namespace lepp
{

struct RGBData
{
  RGBData(long frameNum, cv::Mat const& image)
    : frameNum(frameNum), image(image) {}
	long frameNum;
  cv::Mat const& image;
};


class RGBDataObserver 
{
public:
	/**
	* Virtual deconstructor.
	*/
	virtual ~RGBDataObserver() {}

	/**
	* Update observer with new rgb data.
	*/
	virtual void updateFrame(RGBDataPtr rgbData) = 0;
};



class RGBDataSubject 
{
public:
	/**
	* Virtual deconstructor.
	*/
	virtual ~RGBDataSubject() {}

	/**
	* Attach new observer to observer list.
	*/
	void attachObserver(boost::shared_ptr<RGBDataObserver> observer)
	{
	  observers.push_back(observer);
	}

protected:
	/**
	* Notify all attached observers.
	*/
    void notifyObservers(RGBDataPtr rgbData)
    {
	  for (size_t i = 0; i < observers.size(); i++) 
	  {
	    observers[i]->updateFrame(rgbData);
	  }
	}

private:
	// vector holding all attached observers of this subject
	std::vector<boost::shared_ptr<RGBDataObserver> > observers;
};

}

#endif
