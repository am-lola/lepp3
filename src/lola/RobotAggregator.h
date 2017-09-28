#ifndef LEPP3_LOLA_LOLA_AGGREGATOR_H__
#define LEPP3_LOLA_LOLA_AGGREGATOR_H__

#include "lepp3/DiffAggregator.hpp"
#include "lepp3/models/ObjectModel.h"
#include "lepp3/FrameData.hpp"
#include "lepp3/RGBData.hpp"

#include "lola/RobotService.h"
#include "lola/Robot.h"
#include <iface_vision_msg.hpp>

#include <boost/array.hpp>
#include <boost/asio.hpp>

using namespace lepp;
using am2b_iface::RGBMessage;
using am2b_iface::VisionMessage;
using am2b_iface::SurfaceMessage;
using am2b_iface::ObstacleMessage;
using am2b_iface::PointCloudMessage;
using am2b_iface::Message_Type;

/**
 * An `FrameDataObserver` implementation that sends notifications to the robot
 * after every certain amount of frames, informing it of changes in the known
 * obstacles since the previous message.
 *
 * The implementation relies on a `DiffAggregator` to find this diff and uses
 * this to send appropriate messages to the robot. The most notable adjustment
 * that needs to be made is "flattening" a composite model (one made of several
 * primitive models) into its most primitive components and sending msesages for
 * each of those separately to the robot.
 */
class RobotAggregator : public lepp::FrameDataObserver, public lepp::RGBDataObserver {
public:
  /**
   * Create a new `RobotAggregator` that will use the given service to
   * communicate to the robot and send status updates after every `freq` frames.
   */
  RobotAggregator(boost::shared_ptr<RobotService> service,
                  int freq,
                  std::vector<std::string> datatypes,
                  Robot& robot,
                  double min_surface_height = 0,
                  double surface_normal_tolerance = 0
                );
  /**
   * `FrameDataObserver` interface implementation.
   */
  void updateFrame(FrameDataPtr frameData) {
    // Just pass it on to find the diff!
    diff_.updateFrame(frameData);

    if (send_pointclouds_)
    {
      sendPointCloud(frameData->cloud, frameData->frameNum);
    }
  }
  /**
   * `RGBDataObserver` interface implementation.
   */
  void updateFrame(RGBDataPtr rgbData) {
    if (send_images_)
    {
      sendRGBImage(rgbData->image, rgbData->frameNum);
    }

  }
private:
  /**
   * The function is passed as a callback to the underlying `DiffAggregator` for
   * when new models are discovered.
   */
  void new_obstacle_cb_(ObjectModel& model, long frame_num);

  /**
   * The function is passed as a callback to the underlying `DiffAggregator` for
   * when models are discovered to be deleted.
   */
  bool del_obstacle_cb_(ObjectModel& model, long frame_num);
  /**
   * The function is passed as a callback to the underlying `DiffAggregator` for
   * when a model has been modified.
   */
  void mod_obstacle_cb_(ObjectModel& model, long frame_num);

  /**
   * The function is passed as a callback to the underlying `DiffAggregator` for
   * when new surfaces are discovered.
   */
  void new_surface_cb_(SurfaceModel& model, long frame_num);

  /**
   * The function is passed as a callback to the underlying `DiffAggregator` for
   * when surfaces are discovered to be deleted.
   */
  bool del_surface_cb_(SurfaceModel& model, long frame_num);
  /**
   * The function is passed as a callback to the underlying `DiffAggregator` for
   * when a surface has been modified.
   */
  void mod_surface_cb_(SurfaceModel& model, long frame_num);

  /**
   * Obtains a list of pointers to the primitives that the given model is
   * composed of.
   * This list is safe to use only while the model reference is in scope.
   */
  std::vector<ObjectModel*> getPrimitives(ObjectModel& model) const;

  /**
   * Sends a message to the robot informing it of a new model.
   */
  void sendNew(ObjectModel& new_model, int model_id, int part_id, long frame_num);
  /**
   * Sends a message to the robot informing it of a new surface.
   */
  void sendNew(SurfaceModel& new_surface, long frame_num);
  /**
   * Sends a message to the robot informing it of a deleted model.
   */
  void sendDeleteObstacle(int id, long frame_num);
  /**
   * Sends a message to the robot informing it of a deleted part of a model.
   */
  void sendDeleteObstaclePart(int model_id, int part_id, long frame_num);
  /**
   * Sends a message to the robot informing it of a deleted surface.
   */
  void sendDeleteSurface(int id, long frame_num);
  /**
   * Sends a message to the robot informing it of a modified model.
   */
  void sendModify(ObjectModel& model, int model_id, int part_id, long frame_num);
  /**
   * Sends a message to the robot informing it of a modified surface.
   */
  void sendModify(SurfaceModel& surface, long frame_num);
  /**
   * Sends a point cloud to the remote host
   */
  void sendPointCloud(PointCloudConstPtr cloud, long frame_num);
  /**
   * Sends an RGB image to the remote host
   */
  void sendRGBImage(cv::Mat const& image, long frame_num);
  /**
   * Obtains the next ID that should be used for a primitive that the robot is
   * notified of.
   */
  int nextId() { return next_id_++; }
  /**
   * Determines whether the given surface should be sent to the planning system.
   * Surfaces which fail this test have their convex hulls replaced with dummy_hull.
   */
  bool shouldSendSurface(SurfaceModel& model);

  /**
   * A handle to the service that is used to send notifications to the robot.
   */
  boost::shared_ptr<RobotService> service_;
  /**
   * A handle to the robot facade.
   */
  Robot& robot_;
  /**
   * An instance of a `DiffAggregator` that this one delegates to in order to
   * detect the differences between the checkpoint frames.
   */
  DiffAggregator diff_;

  /**
   * Maps the approximation ID to a list of IDs that the robot will know for
   * each primitive SSV object that is found in the model composition.
   */
  std::map<int, std::vector<int> > robot_ids_;
  /**
   * The ID that can be assigned to the next new model (or rather model part).
   */
  int next_id_;

  /**
   * Flags for enabling/disabling specific types of data from being collected
   * and sent by the aggregator.
   */
  bool send_obstacles_   = false;
  bool send_surfaces_    = false;
  bool send_images_      = false;
  bool send_pointclouds_ = false;

  /**
   * Minimum distance from the ground plane a surface must meet in order to
   * be sent to be published. Any surface bellow this height will not be
   * published or will be transformed to prevent it from being considered.
   */
  double min_surface_height = 0.0;

  /**
   * If a surface fails the min_surface_height test, we may still consider it
   * if the surface's normal is more than surface_normal_tolerance radians from
   * the world vertical axis (0, 0, 1).
   */
  double surface_normal_tolerance = 0.0;

  /**
   * A dummy convex hull used to prevent some surfaces from being used by the
   * planning system. This is sent instead of the original hull to keep all the
   * surface IDs consistent on both sides.
   */
  PointCloudPtr dummy_hull;
};

#endif
