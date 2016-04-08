#ifndef lepp3_SMOOTH_SURFACE_AGGREGATOR_H__
#define lepp3_SMOOTH_SURFACE_AGGREGATOR_H__

#include "lepp3/SurfaceData.hpp"
#include "lepp3/Typedefs.hpp"
#include <pcl/surface/concave_hull.h>
#include <pcl/surface/convex_hull.h>

#include <list>
#include <vector>
#include <map>

namespace lepp {

/**
 * A visitor implementation that will perform a translation of models that it
 * visits by the given translation vector.
 */
class BlendVisitors : public SurfaceVisitor {
public:
	/**
	 * Create a new `BlendVisitor` will update the given surface in the argument using the class parameters.
	 */
	BlendVisitors(model_id_t id, Coordinate translation_vec, PointCloudConstPtr hull, 
		pcl::ModelCoefficients oldCoefficients) :
			id(id), 
			translation_vec(translation_vec), 
			hull(hull), 
			oldCoefficients(oldCoefficients) {
	}
	void visitSurface(SurfaceModel &newPlane) 
	{
		// set id of new plane
		newPlane.set_id(id);

		// update center point
		newPlane.translateCenterPoint(translation_vec);

		// set convex hull to old convex hull (this is used by the convex hull detector later on)
		newPlane.set_hull(hull);	

		// Take average of old and new model coefficients
		pcl::ModelCoefficients mergeCoefficients (newPlane.get_planeCoefficients());			
		for (int i = 0; i < 4; i++)
			mergeCoefficients.values[i] = 0.5 * (mergeCoefficients.values[i] + oldCoefficients.values[i]);
		newPlane.set_planeCoefficients(mergeCoefficients);
	}

private:
	model_id_t id;
	Coordinate const translation_vec;
	PointCloudConstPtr hull;
	pcl::ModelCoefficients oldCoefficients;
};



/**
 * A `SurfaceAggregator` decorator.
 *
 * It takes the surfaces found by  surface detector and applies some
 * postprocessing in order to "smooth out" the surfaces being passed on to the
 * final output.
 *
 * It does so by tracking which of the surfaces detected in the new frame have
 * also been previously found and outputting only those that have been found in
 * a sufficient number of consecutive frames, so as to give us some certainty
 * that its appearance is not due to sensor noise.
 *
 * Conversely, it also tracks which surfaces have disappeared, propagating the
 * disappearance only if the surface has been gone in a sufficient number of
 * consecutive frames.
 *
 * It emits the surfaces that it considers real in each frame to all
 * aggregators that are attached to it.
 */
template<class PointT>
class SurfaceTracker: public SurfaceDataObserver, public SurfaceDataSubject {

public:
	/**
	 * Creates a new `SurfaceTracker`.
	 */
	SurfaceTracker();

	/**
	 * The member function that all concrete aggregators need to implement in
	 * order to be able to process newly detected surfaces.
	 */
	virtual void updateSurfaces(SurfaceDataPtr surfaceData);

// Private types
private:
	/**
	 * Computes the matching of the new surfaces to the surfaces that are being
	 * tracked already.
	 *
	 * If a new surface does not have a match in the ones being tracked, a new
	 * ID is assigned to it and it is added to the `tracked_models_`.
	 *
	 * The returned map represents a mapping of model IDs (found in the
	 * `tracked_models_`) to the index of this surface in the `new_surfaces`
	 * list.
	 */
	std::map<int, size_t> matchToPrevious(std::vector<SurfaceModelPtr> const& new_surfaces);

	/**
	 * Goes through all the tracked models, which are also detected in the new detection
	 * to recalculate necessary differences -like a small shift in the center of the surface
	 * and adjust the matching criterias,
	 */
	void adaptTracked(std::map< int, size_t> const& correspondence,
			std::vector<SurfaceModelPtr> const& new_surfaces);

	/**
	 * Updates the internal `frames_found_` and `frames_lost_` counters for each
	 * mode, based on the given new matches description, i.e. increments the
	 * seen counter for all models that were already tracked and found in the new
	 * frame (i.e. included in the `new_matches` (matching of the already tracked models with
	 * the addition of newly appeared surfaces in the latest scene)
	 * increments the lost counter for
	 * all models that were tracked, but not found in the new frame.
	 * The format of the given parameter is the one returned by the
	 * `matchToPrevious` member function.
	 * ****************************************************************
	 * Here the important point is, frames_found and frames_lost do not keep
	 * ids of materialized models. They keep the track of sequential information.
	 * Those lists stand between "being dropped" and "being materialized".
	 */

	void updateLostAndFound(std::map<int,size_t> const& new_matches);

	/**
	 * Drops any surface that has been lost too many frames in a row.
	 * This means that the surface is removed from tracked surface, as well as no
	 * longer returned as a "real" (materialized) surface.
	 */
	void dropLostSurface();

	/**
	 * Materializes any surface that has been seen enough frames in a row.
	 * This means that tracked surfaces that seem to be stable are "graduated up"
	 * to a "real" surface and from there on out presented to the underlying
	 * aggregator. Those surfaces are removed from frames_found list as well.
	 */
	void materializeFoundSurfaces();


	/**
	 * The function returns the next available model ID. It makes sure that no
	 * models are ever assigned the same ID.
	 */
	 model_id_t nextModelId();

	/**
	 * Returns the ID of a model tracked that matches the given model.
	 *
	 * The matching is done in such a way to return the model that whose
	 * characteristic point has the smallest distance from the point of the given
	 * mode, but given that the distance is below a certain threshold.
	 *
	 * Surfaces for which no such match can be found are given a new ID and this ID
	 * is returned.
	 */
	model_id_t getMatchByDistance(SurfaceModelPtr model);

	// Private members
	/**
	 * Keeps track of which model ID is the next one that can be assigned.
	 */
	model_id_t next_model_id_;

	/**
	 * A mapping of model IDs to their `SurfaceModel` representation.
	 */
	std::map<model_id_t, SurfaceModelPtr > tracked_models_;

	/**
	 * A mapping of the model ID to the number of subsequent frames that the model
	 * was found in.
	 */
	std::map<model_id_t, int> frames_found_;

	/**
	 * A mapping of the model ID to the number of subsequent frames that the model
	 * was no longer found in.
	 */
	std::map<model_id_t, int> frames_lost_;

	/**
	 * Contains those models that are currently considered "real", i.e. not simply
	 * perceived in one frame, but with sufficient certainty in many frames that
	 * we can claim it's a real surface (therefore, it got "materialized").
	 * We use a linked-list for this purpose since we need efficient removals
	 * without copying elements or (importantly) invalidating iterators.
	 */
	std::list<SurfaceModelPtr> materialized_models_;

	/**
	 * Maps the model to their position in the linked list so as to allow removing
	 * surfaces efficiently.
	 * Theoretically, the removal would asymptotically be dominated by the map
	 * lookup. In practice, since
	 * the number of surfaces will always be extremely small, it may as well be
	 * O(1).
	 */
	std::map<model_id_t, std::list<SurfaceModelPtr>::iterator> model_idx_in_list_;

	static const int LOST_LIMIT = 5;
	static const int FOUND_LIMIT = 5;
};

template<class PointT>
SurfaceTracker<PointT>::SurfaceTracker() :
		next_model_id_(0) {
}

template<class PointT>
model_id_t SurfaceTracker<PointT>::nextModelId() {
	return next_model_id_++;
}

/*!!!!!min_dist definition, should be checked !!!! */
template<class PointT>
model_id_t SurfaceTracker<PointT>::getMatchByDistance(
		SurfaceModelPtr newSurface) {

	Coordinate const query_point = newSurface->centerpoint();
	size_t newSurfaceSize = newSurface->get_cloud()->size();

	bool found = false;
	double min_dist = 1e10;

	model_id_t match = 0;

	for(std::map<model_id_t,SurfaceModelPtr>::const_iterator it = tracked_models_.begin(); it!=tracked_models_.end(); ++it)
	{
		/*Iterate through tracked models for the same surface check. Now the only criteria for the check is the center point
		 * distance error.
		 * This has to be extended to inclination and area criteria
		 * */
			//TODO include area and maybe eliminate the point clouds with too small areas from the beginning.?
		Coordinate const p(it->second->centerpoint());
		double const dist = (p.x - query_point.x) * (p.x - query_point.x)
				+ (p.y - query_point.y) * (p.y - query_point.y)
				+ (p.z - query_point.z) * (p.z - query_point.z);

		//std::cout << "Distance was " << dist << " ";

		size_t trackedSurfaceSize = it->second->get_cloud()->size();

		if ((dist <= 0.05) && (std::abs((1.0*trackedSurfaceSize/newSurfaceSize) - 1) < 0.5))
		{
			//std::cout << " accept";
			if (!found)
				min_dist = dist;
			found = true;

			if (dist <= min_dist) {
				min_dist = dist;
				match = it->first;
			}
		}
		//std::cout << std::endl;
	}

	if (found) {
		return match;
	} else {
		return nextModelId();
	}
}

template<class PointT>
std::map<int, size_t> SurfaceTracker<PointT>::matchToPrevious(std::vector<SurfaceModelPtr> const& new_surfaces) {

	// Maps the ID of the model to its index in the new list of surfaces.
	// This lets us know the new approximation of each currently tracked surface.
	// If the model ID is not in the `tracked_models_`, it means we've got a new
	// surface in the stream.
	// Model_ID in the tracking side, and the index of the surface in the "vector of new_surfaces"
	//which is basically the surfaces in the current frame.
	//template <typename model_id_t,typename size_t>
	//using correspondence= std::map < model_id_t, size_t >;
	std::map<int,size_t> correspondence;

	// Keeps a list of surfaces that are new in the frame (the ID and its index
	// in the `new_surfaces`). This is done in order to insert the new surfaces
	// after the matching step, since we don't want some of the new surfaces to
	// accidentaly get matched to one of the other new ones. Therefore, we defer
	// the update of the tracked surfaces 'till after the matching step.

	//template <typename model_id_t ,typename size_t>
	//using TPair = std::pair<model_id_t,size_t>;

	//template <typename model_id_t,typename size_t>
	//using new_in_frame = std::vector<TPair<model_id_t,size_t>>;
	std::vector < std::pair<int, size_t> > new_in_frame;

	// First we match each new surface to one of the models that is currently
	// being tracked or give it a brand new model ID, if we are unable to find a
	// match.
	for (size_t i = 0; i < new_surfaces.size(); ++i) {
		//std::cout << "Matching " << i << " (" << *new_surfaces[i] << ")"
		//		<< std::endl;
		//If the surface is new, then it will get a new model_id. If the surface has already
		//been tracked, then its existing model_id is going to be returned.
		 model_id_t const model_id = getMatchByDistance(new_surfaces[i]);
		//std::cout << "Matched " << i << " --> " << model_id << std::endl;
	     correspondence[model_id] = i;
		// If this ID wasn't in the tracked models before, add it to new in frame!

		if (tracked_models_.find(model_id) == tracked_models_.end()) {
			//Could not be found, so make a pair of model_id and the index number i in new_surface
			 new_in_frame.push_back(std::make_pair(model_id, i));
		}
	}

	// We start tracking each surface for which we were unable to find a match
	// in the currently tracked list of surfaces. To those, that are just added:
	for (size_t i = 0; i < new_in_frame.size(); ++i) {

		model_id_t const& model_id = new_in_frame[i].first; //model_id in the whole tracking
		size_t const corresp = new_in_frame[i].second; //corresponding index number in new_surface
		//std::cout << "Inserting previously untracked model " << model_id
		//		<< std::endl;
		tracked_models_[model_id] = new_surfaces[corresp]; // corresp=i basically
		frames_lost_[model_id] = 0;
		frames_found_[model_id] = 0;

		// We assign an ID to our newly appeared surface in our frame here too!
		new_surfaces[corresp]->set_id(model_id);
	}

	return correspondence;
}


template<class PointT>
void SurfaceTracker<PointT>::adaptTracked(
		std::map<int, size_t> const& correspondence,
		std::vector<SurfaceModelPtr> const& new_surfaces) {

	for (std::map<int, size_t>:: const_iterator it =
			correspondence.begin(); it != correspondence.end(); ++it) 
	{
		model_id_t const& model_id = it->first;
		int const& i = it->second;

		// exchange the tracked surface model with the new surface model
		SurfaceModelPtr oldSurfaceModel = tracked_models_[model_id];
		tracked_models_[model_id] = new_surfaces[i];

		// update surface model poointer in the materialized models if necessary
		if (model_idx_in_list_.find(model_id) != model_idx_in_list_.end())
			*model_idx_in_list_[model_id] = tracked_models_[model_id];


		// Blend the new representation into the one we're tracking
		Coordinate const translation_vec = (oldSurfaceModel->centerpoint() - new_surfaces[i]->centerpoint()) / 2;

		// Blend the old surface into the new one
		BlendVisitors blender(oldSurfaceModel->id(), translation_vec, oldSurfaceModel->get_hull(), 
			oldSurfaceModel->get_planeCoefficients());

		tracked_models_[model_id]->accept(blender);
	}
}

template <class PointT>
void SurfaceTracker<PointT>::updateLostAndFound(std::map<int, size_t> const& new_matches) {

	for (std::map<model_id_t, boost::shared_ptr<SurfaceModel> >::const_iterator it =
			tracked_models_.begin(); it != tracked_models_.end(); ++it) {

		model_id_t const& model_id = it->first;

		//New matches is the correspondence map (model_id, i (index in new surfaces))
		if (new_matches.find(model_id) != new_matches.end()) {

			// Update the seen count only if the surface isn't already materialized.
			if (model_idx_in_list_.find(model_id) == model_idx_in_list_.end()) {

				//The model id is no not in model_idx_in_list
				//std::cout << "Incrementing the seen count for Surface ID: " << model_id
				//		<< std::endl;
				++frames_found_[model_id];
			}
			// ...but always reset its lost counter, since we've now seen it.
			frames_lost_[model_id] = 0;
		} else {
			//std::cout << "Incrementing the lost count for Surface ID: " << model_id
			//		<< std::endl;
			++frames_lost_[model_id];
			frames_found_[model_id] = 0;
		}
	}
}

template<class PointT>
void SurfaceTracker<PointT>::dropLostSurface() {
	// Drop surfaces that haven't been seen in a while

	//cout<<"DropLostSurface..."<<std::endl;
	std::map<model_id_t, int>::iterator it = frames_lost_.begin();

	while (it != frames_lost_.end()) {
		if (it->second >= LOST_LIMIT) {

			//std::cout << "Surface ID: " << it->first
			//		<< "is not found 10 times in a row: DROPPING" << std::endl;

			// Stop tracking the model, since it's been gone for a while.
			if (tracked_models_.find(it->first) != tracked_models_.end()) {
				tracked_models_.erase(tracked_models_.find(it->first));
			}
			// If the model was also materialized, make sure we drop it from there too
			if (model_idx_in_list_.find(it->first)
					!= model_idx_in_list_.end()) {

				materialized_models_.erase(model_idx_in_list_[it->first]);
				model_idx_in_list_.erase(model_idx_in_list_.find(it->first));

			}

			// Remove the helper tracking data too.
			if (frames_found_.find(it->first) != frames_found_.end()) {
				frames_found_.erase(frames_found_.find(it->first));
			}

			frames_lost_.erase(it++);
		} else {
			//std::cout << "Surface ID:" << it->first
			//		<< " is not lost enough times to be dropped. Only:" << " ("
			//		<< it->second << ") times lost." << std::endl;
			++it;
		}
	}
}
template<class PointT>
void SurfaceTracker<PointT>::materializeFoundSurfaces() {
	//std::cout<<"Materialize Surfaces"<<std::endl;
	std::map<model_id_t, int>::iterator it = frames_found_.begin();
	while (it != frames_found_.end()) {
		// Deconstruct the iterator pair for convenience
		model_id_t const model_id = it->first;
		int const seen_count = it->second;
		if (seen_count >= FOUND_LIMIT) {
			//std::cout << "Surface found " << model_id
			//		<< " 5 times in a row: Materializing!" << std::endl;
			// Get the corresponding model
			SurfaceModelPtr const& model =
					tracked_models_.find(model_id)->second;
			// Materialize it...
			materialized_models_.push_back(model);
			// ...and make sure we know where in the list it got inserted.
			std::list<SurfaceModelPtr>::iterator pos =
					materialized_models_.end();
			--pos;
			model_idx_in_list_[model_id] = pos;
			// Finally, make sure we remove it from the mapping so that we don't end
			// up adding it more than once to the list of materialized surfaces.
			frames_found_.erase(it++);

		} else {
			//std::cout << "Surface ID: " << it->first
			//		<< " is not seen enough times to be materialized" << " ("
			//		<< it->second << ") Times seen" << std::endl;
			++it;
		}
	}
}


template<class PointT>
void SurfaceTracker<PointT>::updateSurfaces(SurfaceDataPtr surfaceData)
{
	std::map<int, size_t> correspondence = matchToPrevious(surfaceData->surfaces);
    updateLostAndFound(correspondence);
	adaptTracked(correspondence, surfaceData->surfaces);
	dropLostSurface();
    materializeFoundSurfaces();

    // copy materialized models    
    std::vector<SurfaceModelPtr> materializedModelsCopy(materialized_models_.begin(), materialized_models_.end());
    surfaceData->surfaces = materializedModelsCopy;

	notifyObservers(surfaceData);
}

}// namespace lepp

#endif
