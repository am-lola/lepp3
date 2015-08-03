#ifndef LOLA_POSE_SERVICE_H__
#define LOLA_POSE_SERVICE_H__

#include <boost/asio.hpp>
#include <boost/array.hpp>

#include "lepp2/models/Coordinate.h"
using boost::asio::ip::udp;

/**
 * A struct wrapping the parameters LOLA-provided kinematics parameters that are
 * used to construct the transformation matrices between the camera frame and
 * the world coordinate system as LOLA knows it.
 */
struct LolaKinematicsParams {
  double t_wr_cl[3];
  double R_wr_cl[3][3];
  double t_stance_odo[3];
  double phi_z_odo;
  double stance;
  int stamp;
};

/*!
  Robot pose data

  vectors given in world coordinate frame

  ub = upper body
  fl = left foot
  fr = right foot
  cl = left camera
  cr = right camera
  im = IMU,

  wr = world frame
  odo = drift frame

  Struct reused verbatim from the original LOLA source code to keep compatibility.
*/
#pragma pack(push)
#pragma pack(1)
struct HR_Pose
{
  enum{RIGHT=0,LEFT=1};
  //!UDP port
  enum{PORT=0xD001};
  //!data rate [ms] (actually determined by hardware trigger!)
  enum{RATE=50};
  //!number of segments in detailed robot model.
  enum{N_SEGMENTS=25};

  //rot = rotation
  //add = adduction
  //flx = flexion
  enum SegmentIndex
    {
      //torso
      seg_torso=0,
      //pelvis
      seg_pelvis_rot,seg_pelvis_add,
      //right leg
      seg_hip_rot_r,seg_hip_add_r,seg_hip_flx_r,seg_knee_flx_r,seg_ankle_add_r,seg_ankle_flx_r,seg_toe_flx_r,
      //left leg
      seg_hip_rot_l,seg_hip_add_l,seg_hip_flx_l,seg_knee_flx_l,seg_ankle_add_l,seg_ankle_flx_l,seg_toe_flx_l,
      //right arm
      seg_shoulder_flx_r,seg_shoulder_add_r,seg_elbow_flx_r,
      //left arm
      seg_shoulder_flx_l,seg_shoulder_add_l,seg_elbow_flx_l,
      //head (tilt: both cameras without convergence joint)
      seg_head_pan, seg_head_tilt
    };

  //!segment pose
  struct SegmentPose
  {
    //!transform matrix
    float R[3*3];
    //!position
    float t[3];
  };

  //////////////////////////////////////////////////
  //// 1 -- header
  //!data struct version
  uint32_t version;
  //! tick counter
  uint64_t tick_counter;
  //!<stance leg (RIGHT/LEFT)
  uint8_t stance;
  //!<padding
  uint8_t zero[3];

  uint64_t stamp;


  //////////////////////////////////////////////////
  //// 2 -- simplified /abstract robot model (feet, cameras, upper body)
  //!vector from world frame to left leg in world frame
  float t_wr_fr[3];
  //!vector from world frame to right leg in world frame
  float t_wr_fl[3];
  //!vector from world frame to left camera in world frame
  float t_wr_cl[3];
  //!vector from world frame to right camera in world frame
  float t_wr_cr[3];

  //!transformation matrix from left leg to world frame
  float R_wr_fr[3*3];
  //!transformation matrix from right leg to world frame
  float R_wr_fl[3*3];
  //!transformation matrix from left camera to world frame
  float R_wr_cl[3*3];
  //!transformation matrix from right camera to world frame
  float R_wr_cr[3*3];
  /*!
    transformation matrix from upper body coordinate frame
    to inertial frame measured by IMU (world frame)
    (identity matrix, if robot is standing upright)
  */
  float R_wr_ub[3*3];
  //!vector from world frame to upper body frame in world frame
  float t_wr_ub[3];

  //////////////////////////////////////////////////
  //// 3 -- full robot pose
  SegmentPose seg_pose[N_SEGMENTS];

  //////////////////////////////////////////////////
  //// 4 -- "drift pose" (odometry)
  //!stance leg in drift frame
  float t_stance_odo[3];
  //!stance foot rotation in drift frame
  float phi_z_odo;

  //////////////////////////////////////////////////
  //// 5 -- "drift pose" (odometry)
  //!<currently active velocity in x-direction [m/s]
  float vx_act;
  //!<currently active velocity in y-direction [m/s]
  float vy_act;
  //!<currently active angular velocity [rad/s]
  float om_act;
};
#pragma pack (pop)

/**
 * A class that provides the ability to run a local service that listens to
 * LOLA pose messages on a particular UDP port. It provides an API for other
 * components to get the current pose information, without worrying about running
 * the networking communication infrastructure or threading.
 */
class PoseService {
public:
  /**
   * Create a new `PoseService` that will listen on the given local (UDP) socket
   * for new pose messages coming from the robot. It does not need to know the
   * network address of the robot itself.
   */
  PoseService(std::string const& host, int port)
      : host_(host),
        port_(port),
        socket_(io_service_) {}
  /**
   * Starts the `PoseService`.
   *
   * This spins up a thread behind the scenes in order to allow the messages to
   * be processed asynchronously.
   */
  void start();
  /**
   * Obtains the current pose information. Using this method is completely
   * thread safe.
   */
  HR_Pose getCurrentPose() const;

  /**
   * Returns the "World" origin in ODO coordinate system.
   */
  lepp::Coordinate getRobotPosition() const;
  /**
   * Returns the parameters relevant for coordinate system
   * transformations, extracted from the currently known robot pose.
   */
  LolaKinematicsParams getParams() const;
private:
  /**
   * Internal helper method. The callback that is passed to the async receive.
   */
  void read_handler(
      const boost::system::error_code& ec,
      std::size_t bytes_transferred);
  /**
   * Internal helper method. Represents the function that will be ran in the
   * service thread that is spawned once `start` is called.
   */
  void service_thread();
  /**
   * Binds the `socket_` to the local address represented by the parameters
   * given in the constructor.
   */
  void bind();
  /**
   * Queues a new asynchronous receive operation. After each handled receive a
   * new one needs to be queued if we wish to keep running the pose service.
   */
  void queue_recv();

  /**
   * The local host on which the service will listen.
   */
  std::string const host_;
  /**
   * The port number where the service will listen.
   */
  int const port_;

  /**
   * The IO service that will handle the async reads and callback invocation.
   */
  boost::asio::io_service io_service_;
  /**
   * The socket used for UDP communication.
   */
  boost::asio::ip::udp::socket socket_;
  /**
   * The buffer into which the socket will place its read data.
   */
  boost::array<char, sizeof(HR_Pose)> recv_buffer_;

  /**
   * A pointer to the last known pose information.
   * Updated by the service on every newly received packet.
   */
  boost::shared_ptr<HR_Pose> pose_;

};

#endif
