#ifndef LEPP3_LOLA_LOLA_AGGREGATOR_H__
#define LEPP3_LOLA_LOLA_AGGREGATOR_H__

#include "lepp3/DiffAggregator.hpp"
#include "lepp3/models/ObjectModel.h"
#include "lepp3/FrameData.hpp"

#include "lola/RobotService.h"
#include "lola/Robot.h"
#include "VisionMessage.h"

#include <boost/array.hpp>
#include <boost/asio.hpp>

using namespace lepp;

/**
 * A LOLA-specific implementation of an `FrameDataObserver`.
 *
 * It serializes the received obstacles into a format where each obstacle is
 * represented by 11 integers. Each integer is serialized with machine-specific
 * endianess; no care is taken to perform integer serialization for network
 * transfer, i.e. big-endian. In general, this is not the safest assumption to
 * make, but the LOLA controller has been working under this assumption for now,
 * so enforcing a big-endian encoding would just introduce additional complexity.
 *
 * The serialized representation of each obstacle is packed into a single
 * datagram and sent over UDP to the remote host described by the initial
 * constructor parameters.
 */
class LolaAggregator : public lepp::FrameDataObserver {
public:
  /**
   * Creates a new `LolaAggregator` where the remote host to which the obstacle
   * list is sent is identified by the given host name and port number.
   */
  LolaAggregator(std::string const& remote_host, int remote_port);
  ~LolaAggregator();
  /**
   * `FrameDataObserver` interface implementation.
   */
  virtual void updateFrame(FrameDataPtr frameData);
private:
  /**
   * A helper function that builds the datagram payload based on the given
   * obstacles.
   */
  std::vector<char> buildPayload(std::vector<ObjectModelPtr> const& obstacles) const;

  boost::asio::io_service io_service_;
  boost::asio::ip::udp::socket socket_;
  boost::asio::ip::udp::endpoint remote_endpoint_;
};

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
class RobotAggregator : public lepp::FrameDataObserver {
public:
  /**
   * Create a new `RobotAggregator` that will use the given service to
   * communicate to the robot and send status updates after every `freq` frames.
   */
  RobotAggregator(RobotService& service, int freq, Robot& robot);
  /**
   * `FrameDataObserver` interface implementation.
   */
  void updateFrame(FrameDataPtr frameData) {
    // Just pass it on to find the diff!
    diff_.updateFrame(frameData);
  }
private:
  /**
   * The function is passed as a callback to the underlying `DiffAggregator` for
   * when new models are discovered.
   */
  void new_cb_(ObjectModel& model);

  /**
   * The function is passed as a callback to the underlying `DiffAggregator` for
   * when models are discovered to be deleted.
   */
  bool del_cb_(ObjectModel& model);
  /**
   * The function is passed as a callback to the underlying `DiffAggregator` for
   * when a model has been modified.
   */
  void mod_cb_(ObjectModel& model);
  /**
   * Obtains a list of pointers to the primitives that the given model is
   * composed of.
   * This list is safe to use only while the model reference is in scope.
   */
  std::vector<ObjectModel*> getPrimitives(ObjectModel& model) const;

  /**
   * Sends a message to the robot informing it of a new model.
   */
  void sendNew(ObjectModel& new_model, int model_id, int part_id);
  /**
   * Sends a message to the robot informing it of a deleted model.
   */
  void sendDelete(int id);
  /**
   * Sends a message to the robot informing it of a deleted part of a model.
   */
  void sendDeletePart(int model_id, int part_id);
  /**
   * Sends a message to the robot informing it of a modified model.
   */
  void sendModify(ObjectModel& model, int model_id, int part_id);
  /**
   * Obtains the next ID that should be used for a primitive that the robot is
   * notified of.
   */
  int nextId() { return next_id_++; }

  /**
   * A handle to the service that is used to send notifications to the robot.
   */
  RobotService& service_;
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
};

#endif
