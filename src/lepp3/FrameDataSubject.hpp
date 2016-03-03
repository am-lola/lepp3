#ifndef FRAME_DATA_SUBJECT_H_
#define FRAME_DATA_SUBJECT_H_

#include "lepp3/FrameData.hpp"

namespace lepp {

class FrameDataSubject 
{
public:
	/**
	* Attach new observer to observer list.
	*/
	void attachObserver(boost::shared_ptr<FrameDataObserver> observer);

protected:
	/**
	* Notify all attached observers.
	*/
    void notifyObservers(FrameDataPtr frameData);

private:
	// vector holding all attached observers of this subject
	std::vector<boost::shared_ptr<FrameDataObserver> > observers;
};


void FrameDataSubject::attachObserver(boost::shared_ptr<FrameDataObserver> observer) 
{
  observers.push_back(observer);
}


void FrameDataSubject::notifyObservers(FrameDataPtr frameData) 
{
  for (size_t i = 0; i < observers.size(); i++) 
  {
    observers[i]->updateFrame(frameData);
  }
}


}  // namespace lepp

#endif