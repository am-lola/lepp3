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
 * An implementation of an `ObstacleAggregator` that finds diffs between
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
  typedef boost::function<void (ObjectModel&)> NewObstacleCallback;
  typedef boost::function<void (ObjectModel&)> ModifiedObstacleCallback;
  typedef boost::function<bool (ObjectModel&)> DeletedObstacleCallback;

  /**
   * Create a new `DiffAggregator` that will output the diff between frames
   * after every `frequency` frames.
   */
  DiffAggregator(int frequency)
      : freq_(frequency), new_cb_(0), mod_cb_(0), del_cb_(0) {}

  /**
   * Sets a function that will be called for every new obstacle.
   */
  void set_new_callback(NewObstacleCallback new_cb) { new_cb_ = new_cb; }
  /**
   * Sets a function that will be called for every modified obstacle.
   */
  void set_modified_callback(ModifiedObstacleCallback mod_cb) { mod_cb_ = mod_cb; }
  /**
   * Sets a function that will be called for every deleted obstacle.
   */
  void set_deleted_callback(DeletedObstacleCallback del_cb) { del_cb_ = del_cb; }
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

  /**
   * Maps the ID of an obstacle to its ObjectModel smart pointer.
   * Contains the obstacles that the `DiffAggregator` currently knows
   * about.
   */
  std::map<int, ObjectModelPtr> current_obstacles_;

  // Callbacks that are invoked in the appropriate event.
  NewObstacleCallback new_cb_;
  ModifiedObstacleCallback mod_cb_;
  DeletedObstacleCallback del_cb_;
};

// TODO This is made inline to facilitate keeping lepp3 header-only for now.
inline void DiffAggregator::updateFrame(FrameDataPtr frameData) {
  const std::vector<ObjectModelPtr> &obstacles = frameData->obstacles;
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
      if (new_cb_) new_cb_(*obstacles[i]);
    } else {
      // This is a modified obstacle.
      if (mod_cb_) mod_cb_(*obstacles[i]);
    }
    // ..and now remember it for the future.
    previous_ids_.insert(id);
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
    if (del_cb_) {
      // Notify the callback that the object should be deleted
      drop = del_cb_(*current_obstacles_[del_id]);
    }
    if (drop) {
      // If the callback says that the object should be deleted (or there
      // was no callback set) then really finally drop it.
      current_obstacles_.erase(del_id);
      previous_ids_.erase(del_id);
    }
  }
}

}  // namespace lepp
#endif
