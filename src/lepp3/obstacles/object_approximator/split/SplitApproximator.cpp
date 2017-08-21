#include "SplitApproximator.hpp"

#include <deque>

lepp::SplitObjectApproximator::SplitObjectApproximator(boost::shared_ptr<ObjectApproximator> approx,
                                                       boost::shared_ptr<SplitStrategy> splitter)
    : approximator_(approx),
      splitter_(splitter) {
}

lepp::ObjectModelPtr lepp::SplitObjectApproximator::approximate(const ObjectModelParams& object_params) {
  boost::shared_ptr<CompositeModel> approx(new CompositeModel);
  approx->set_id(object_params.id);
  std::deque<std::pair<int, PointCloudPtr> > queue;
  queue.push_back(std::make_pair(0, object_params.obstacleCloud));

  while (!queue.empty()) {
    int const depth = queue[0].first;
    PointCloudPtr const current_cloud = queue[0].second;
    queue.pop_front();

    if (current_cloud->size() < 3) {
      continue;
    }

    ObjectModelParams current_params = object_params;
    current_params.obstacleCloud = nullptr; current_params.obstacleCloud = current_cloud;
    current_params.id = 100000 + approx->id() * 1000 + (approx->count()+1);
    // Delegates to the wrapped approximator for each part's approximation.
    ObjectModelPtr model = approximator_->approximate(current_params);

    // TODO Decide whether the model fits well enough for the current cloud.
    // For now we fix the number of split iterations.
    // The approximation should be improved. Try doing it for the split clouds
    std::vector<PointCloudPtr> const splits = splitter_->split(depth, current_cloud);
    // Add each new split section into the queue as children of the current
    // node.
    if (splits.size() != 0) {
      for (size_t i = 0; i < splits.size(); ++i) {
        queue.push_back(std::make_pair(depth + 1, splits[i]));
      }
    } else {
      // Keep the approximation
      approx->addModel(model);
    }
  }

  return approx;
}
