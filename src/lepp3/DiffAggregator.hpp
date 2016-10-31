#ifndef LEPP3_DIFF_AGGREGATOR_H__
#define LEPP3_DIFF_AGGREGATOR_H__

#include "lepp3/FrameData.hpp"

#include <set>
#include <vector>
#include <algorithm>

#include <boost/bind.hpp>
#include <boost/function.hpp>

namespace lepp {

/**
 * An implementation of an `FrameDataObserver` that finds diffs between
 * obstacles detected in subsequent snapshots.
 *
 * A snapshot is taken after a certain number of frames, provided in the
 * constructor.
 *
 * For each difference between the previous snapshot and the current one,
 * the appropriate callback is fired, if provided, so that the client can
 * take appropriate actions if a diff is detected.
 */
class DiffAggregator : public FrameDataObserver {
public:
  // Callback typedefs
  typedef boost::function<void (ObjectModel&, long)> NewObstacleCallback;
  typedef boost::function<void (ObjectModel&, long)> ModifiedObstacleCallback;
  typedef boost::function<bool (ObjectModel&, long)> DeletedObstacleCallback;

  typedef boost::function<void (SurfaceModel&, long)> NewSurfaceCallback;
  typedef boost::function<void (SurfaceModel&, long)> ModifiedSurfaceCallback;
  typedef boost::function<bool (SurfaceModel&, long)> DeletedSurfaceCallback;


  /**
   * Create a new `DiffAggregator` that will output the diff between frames
   * after every `frequency` frames.
   */
  DiffAggregator(int frequency)
      : freq_(frequency), new_obstacle_cb_(0), mod_obstacle_cb_(0), del_obstacle_cb_(0), new_surface_cb_(0), mod_surface_cb_(0), del_surface_cb_(0) {}

  /**
   * Sets a function that will be called for every new obstacle.
   */
  void set_new_obstacle_callback(NewObstacleCallback new_obstacle_cb) { new_obstacle_cb_ = new_obstacle_cb; }
  /**
   * Sets a function that will be called for every modified obstacle.
   */
  void set_modified_obstacle_callback(ModifiedObstacleCallback mod_obstacle_cb) { mod_obstacle_cb_ = mod_obstacle_cb; }
  /**
   * Sets a function that will be called for every deleted obstacle.
   */
  void set_deleted_obstacle_callback(DeletedObstacleCallback del_obstacle_cb) { del_obstacle_cb_ = del_obstacle_cb; }

//////////////////////////////////////////////////////////////////////////
  void set_new_surface_callback(NewSurfaceCallback new_surface_cb) { new_surface_cb_ = new_surface_cb; }
  /**
   * Sets a function that will be called for every modified obstacle.
   */
  void set_modified_surface_callback(ModifiedSurfaceCallback mod_surface_cb) { mod_surface_cb_ = mod_surface_cb; }
  /**
   * Sets a function that will be called for every deleted obstacle.
   */
  void set_deleted_surface_callback(DeletedSurfaceCallback del_surface_cb) { del_surface_cb_ = del_surface_cb; }
  /**
   * Implementation of the `FrameDataObserver` interface.
   */
  virtual void updateFrame(FrameDataPtr frameData);
private:
  /**
   * The number of frames after which the difference to the previous snapshot
   * should be found.
   */
  int freq_;
  /**
   * The current frame number.
   */
  int curr_;
  /**
   * A set of model IDs found in the previous snapshot.
   */
  std::set<int> previous_ids_;
  std::set<int> previous_surface_ids_;


  /**
   * Maps the ID of an obstacle to its ObjectModel smart pointer.
   * Contains the obstacles that the `DiffAggregator` currently knows
   * about.
   */
  std::map<int, ObjectModelPtr> current_obstacles_;
    std::map<int, SurfaceModelPtr> current_surfaces_;


  // Callbacks that are invoked in the appropriate event.
  NewObstacleCallback new_obstacle_cb_;
  ModifiedObstacleCallback mod_obstacle_cb_;
  DeletedObstacleCallback del_obstacle_cb_;

  //  Callbacks that are invoked in the appropriate event
  NewSurfaceCallback new_surface_cb_;
  ModifiedSurfaceCallback mod_surface_cb_;
  DeletedSurfaceCallback del_surface_cb_;
};

// TODO This is made inline to facilitate keeping lepp3 header-only for now.
inline void DiffAggregator::updateFrame(FrameDataPtr frameData) {
  const std::vector<ObjectModelPtr> &obstacles = frameData->obstacles;
  const std::vector<SurfaceModelPtr> &surfaces = frameData->surfaces;

  ++curr_;
  if (curr_ % freq_ != 0) return;

  // All IDs in the given list are either new or a modified representation of an
  // obstacle found in the previous snapshot.
  std::set<int> current_ids;
  size_t const sz = obstacles.size();
  for (size_t i = 0; i < sz; ++i) {
    int const id = obstacles[i]->id();
    current_ids.insert(id);
    // Start tracking (or update) the obstacle model.
    current_obstacles_[id] = obstacles[i];
    // Check if the obstacle was found in the previous snapshot...
    if (previous_ids_.find(id) == previous_ids_.end()) {
      // This is a new obstacle.
      if (new_obstacle_cb_) new_obstacle_cb_(*obstacles[i], frameData->frameNum);
    } else {
      // This is a modified obstacle.
      if (mod_obstacle_cb_) mod_obstacle_cb_(*obstacles[i], frameData->frameNum);
    }
    // ..and now remember it for the future.
    previous_ids_.insert(id);
  }

  std::set<int> current_surface_ids;
  size_t const sz_ = surfaces.size();
  std::cout << "DiffAggregator::updateFrame> Surfaces: " << sz_ << std::endl;
  for (size_t i = 0; i < sz_; ++i) {
    int const id = surfaces[i]->id();
    current_surface_ids.insert(id);
    // Start tracking (or update) the obstacle model.
    current_surfaces_[id] = surfaces[i];
    // Check if the obstacle was found in the previous snapshot...
    if (previous_surface_ids_.find(id) == previous_surface_ids_.end()) {
      // This is a new obstacle.
      if (new_surface_cb_) new_surface_cb_(*surfaces[i], frameData->frameNum);
    } else {
      // This is a modified obstacle.
      if (mod_surface_cb_) mod_surface_cb_(*surfaces[i], frameData->frameNum);
    }
    // ..and now remember it for the future.
    previous_surface_ids_.insert(id);
  }

  // The ids that are in the previous set, but not the current ones are deleted
  // obstacles
  std::vector<int> deleted;
  std::set_difference(previous_ids_.begin(), previous_ids_.end(),
                      current_ids.begin(), current_ids.end(),
                      std::back_inserter(deleted));
  for (size_t i = 0; i < deleted.size(); ++i) {
    bool drop = true;
    int const del_id = deleted[i];
    if (del_obstacle_cb_) {
      // Notify the callback that the object should be deleted
      drop = del_obstacle_cb_(*current_obstacles_[del_id], frameData->frameNum);
    }
    if (drop) {
      // If the callback says that the object should be deleted (or there
      // was no callback set) then really finally drop it.
      current_obstacles_.erase(del_id);
      previous_ids_.erase(del_id);
    }
  }

  std::vector<int> deleted_surfaces;
  std::set_difference(previous_surface_ids_.begin(), previous_surface_ids_.end(),
                      current_surface_ids.begin(), current_surface_ids.end(),
                      std::back_inserter(deleted_surfaces));
  for (size_t i = 0; i < deleted_surfaces.size(); ++i) {
    bool drop = true;
    int const del_id = deleted_surfaces[i];
    if (del_surface_cb_) {
      // Notify the callback that the object should be deleted
      drop = del_surface_cb_(*current_surfaces_[del_id], frameData->frameNum);
    }
    if (drop) {
      // If the callback says that the object should be deleted (or there
      // was no callback set) then really finally drop it.
      current_surfaces_.erase(del_id);
      previous_surface_ids_.erase(del_id);
    }
  }
}

}  // namespace lepp
#endif
