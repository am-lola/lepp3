/*!
  @file iface_vis.h

  Interface definition for robot/vision communication.

  everything in SI units

  (rotation) matrices are stored in *column major order*,
  i.e. the matrix
  [R00 | R01 | R02]
  [R10 | R11 | R12]
  [R20 | R21 | R22]
  is saved as
  double R[9] = [R00, R10, R20, R01, R11, R21, R02, R12, R22]

  * version 4->5
  - all pose data now in world frame
  - removed phi_z from HR_Pose.
  * version 6: added version variables
  * version 7: added knee positions
  * version 8: + added drift pose
  + added view target
  * version 9: HR_Control + desired hardware trigger rate
  * version A: - HR_Pose: added full robot pose 
               - switched to floats
*/

#ifndef __IFACE_VIS_H__
#define __IFACE_VIS_H__


/**************************************************
 make sure we have c99 integer types
  and
 plattform independent structure packing/alignment
**************************************************/

#if (defined WIN32)||(defined WIN64)
typedef unsigned char uint8_t;
typedef unsigned int uint32_t;
typedef unsigned __int64 uint64_t;
#else
#include <stdint.h>
#endif

#if (defined WIN32) || (defined WIN64)
#pragma pack(push,1)
#else
_Pragma("pack(1)")
#endif//WIN32//64

//!Interface version
const uint32_t IFACE_VIS_VERSION=0x000A;

/*!
  data structure for controlling robot motion
  at velocity level.
*/
struct HR_Control
{
  enum{PORT=0xD000} _enumports;//!<UDP port
  enum{RATE=100} _enumrate;//!<data rate [ms]
  uint32_t version;
  uint64_t stamp;
  float vx;//!<velocity in x-direction [m/s]
  float vy;//!<velocity in y-direction [m/s]
  float om;//!<angular velocity [rad/s]
  float r_view_target_odo[3];//!<view target in drift frame
  float triggerRate;//! desired hardware trigger rate [1/s]
  //!control flags
  uint32_t flags;
  //!meaning of bits in flags variable
  enum BITS
    {
      //!reset tick counter (in HR_Pose)
      RESET_TICK_COUNTER=0x1,
      BIT2=0x2
    };
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
*/
struct HR_Pose
{
  enum{RIGHT=0,LEFT=1} _enumfeet;
  //!UDP port
  enum{PORT=0xD001} _enumport;
  //!data rate [ms] (actually determined by hardware trigger!)
  enum{RATE=50} _enumrate;
  //!number of segments in detailed robot model.
  enum{N_SEGMENTS=25} _enumsegment;

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

/*!
  Robot pose data - new version for sending reduced packages to vision system
  (porblem of sending whole HR_Pose: enum do not seem to be well defined (type) 
  - should change with c11)

  vectors given in world coordinate frame

  ub = upper body
  fl = left foot
  fr = right foot
  cl = left camera
  cr = right camera
  im = IMU,

  wr = world frame
  odo = drift frame
*/
struct HR_Pose_Red
{
  //////////////////////////////////////////////////
  //// 1 -- header
  //!data struct version
  uint32_t version;
  //! tick counter
  uint64_t tick_counter;
  //!<stance leg (RIGHT/LEFT)
  uint8_t stance;

  uint64_t stamp;


  //////////////////////////////////////////////////
  //// 2 -- simplified /abstract robot model (feet, cameras, upper body)
  //!vector from world frame to left camera in world frame
  float t_wr_cl[3];
  //!transformation matrix from left camera to world frame
  float R_wr_cl[3*3];

  //////////////////////////////////////////////////
  //// 4 -- "drift pose" (odometry)
  //!stance leg in drift frame
  float t_stance_odo[3];
  //!stance foot rotation in drift frame
  float phi_z_odo;

  /*!
    transformation matrix from upper body coordinate frame
    to inertial frame measured by IMU (world frame)
    (identity matrix, if robot is standing upright)
  */
  float R_wr_ub[3*3];
  //!vector from world frame to upper body frame in world frame
  float t_wr_ub[3];
};


/*****************************DANIEL STRUCT*************************************/
/*
  Obstacles list (modelled by spheres)
  
  each sphere has 3 data: 2 correspond to the cener location (world coordinates, with z = 0) and one to the radius

  for the first version, the number of valid objects is sent first and then 100 objects are always sent (the unused bytes are 0)
  
*/
struct VIS_Spheres
{
  enum{RIGHT=0,LEFT=1} _enumfeet;
  //!<UDP port
  enum{PORT=0xD002} _enumport;
  //!<data rate [ms] (actually determined by hardware trigger!)
  enum{RATE=50} _enumrate;
  //!data struct version
  uint16_t version;
  //! number of objects
  uint16_t numobj;
  //! list of the total objects to be transmitted
  int16_t objlist[100][11];
  //debugs
  int16_t worldNOR;
  int16_t worldNOC;
  int16_t backrows;
  int16_t cellsize;
  int16_t mapcenter[2];
  // size of the 2,5D map
  int16_t worldmap[14400];
  // 2,5D map
};

// for new vision system - to send different lists of objects
struct VIS_Spheres_New
{
  enum{RIGHT=0,LEFT=1};
  //!<UDP port
  enum{PORT=0xD002};
  //!<data rate [ms] (actually determined by hardware trigger!)
  enum{RATE=50};
  //!data struct version
  uint16_t version;
  //! number of objects
  uint16_t numobj_ssv_vis;
  uint16_t numobj_camera;
  uint16_t numobj_control;
  //! list of the objects in ssv_map_vis
  int16_t obj_ssv_vis[100][11];
  //! list of the objects recognized by camera
  int16_t obj_camera[100][11];
  //! list of the objects sent to control
  int16_t obj_control[100][11];
};


/*****************************DANIEL STRUCT*************************************/


/****************************************************************************************************/
/************************************old versions of HR_POSE/HR_CONTROL...***************************/
/****************************************************************************************************/
/***************************************************************************************************/


/*!
  data structure for controlling robot motion
  at velocity level.
*/
struct HR_Control_V9
{
  enum{PORT=0xD000};//!<UDP port
  enum{RATE=100};//!<data rate [ms]
  uint32_t version;
  uint64_t stamp;
  double vx;//!<velocity in x-direction [m/s]
  double vy;//!<velocity in y-direction [m/s]
  double om;//!<angular velocity [rad/s]
  double r_view_target_odo[3];//!<view target in drift frame
  float triggerRate;//! desired hardware trigger rate [1/s]
  //!control flags
  uint32_t flags;
  //!meaning of bits in flags variable
  enum BITS
    {
      //!reset tick counter (in HR_Pose)
      RESET_TICK_COUNTER=0x1,
      BIT2=0x2
    };
};


struct HR_Pose_V9
{
  enum{RIGHT=0,LEFT=1};
  //!<UDP port
  enum{PORT=0xD001};
  //!<data rate [ms] (actually determined by hardware trigger!)
  enum{RATE=50};
  //!data struct version
  uint32_t version;
  //! tick counter
  uint64_t tick_counter;
  //!<stance leg (RIGHT/LEFT)
  uint8_t stance;
  //!<padding
  uint8_t zero[3];
  //!vector from world frame to left leg in world frame
  double t_wr_fr[3];
  //!vector from world frame to right leg in world frame
  double t_wr_fl[3];
  //!vector from world frame to left camera in world frame
  double t_wr_cl[3];
  //!vector from world frame to right camera in world frame
  double t_wr_cr[3];

  //!transformation matrix from left leg to world frame
  double R_wr_fr[3*3];
  //!transformation matrix from right leg to world frame
  double R_wr_fl[3*3];
  //!transformation matrix from left camera to world frame
  double R_wr_cl[3*3];
  //!transformation matrix from right camera to world frame
  double R_wr_cr[3*3];
  /*!
    transformation matrix from upper body coordinate frame
    to inertial frame measured by IMU (world frame)
    (identity matrix, if robot is standing upright)
  */
  double R_wr_ub[3*3];
  //!vector from world frame to upper body frame in world frame
  double t_wr_ub[3];

  //!vector from world frame to right knee in world frame
  double t_wr_kr[3];
  //!vector from world frame to left knee in world frame
  double t_wr_kl[3];

  //!stance leg in drift frame
  double t_stance_odo[3];
  //!stance foot rotation in drift frame
  double phi_z_odo;
  //debug
  //uint64_t stamp;
};

struct HR_Control_V8
{
  enum{PORT=0xD000};//!<UDP port
  enum{RATE=100};//!<data rate [ms]
  uint32_t version;
  uint64_t stamp;
  double vx;//!<velocity in x-direction [m/s]
  double vy;//!<velocity in y-direction [m/s]
  double om;//!<angular velocity [rad/s]
  double r_view_target_odo[3];//!<view target in drift frame
  //!control flags
  uint32_t flags;
  //!meaning of bits in flags variable
  enum BITS
    {
      //!reset tick counter (in HR_Pose)
      RESET_TICK_COUNTER=0x1,
      BIT2=0x2
    };
};

struct HR_Pose_V7
{
  enum{RIGHT=0,LEFT=1};
  //!<UDP port
  enum{PORT=0xD001};
  //!<data rate [ms] (actually determined by hardware trigger!)
  enum{RATE=50};
  //!data struct version
  uint32_t version;
  //! tick counter
  uint64_t tick_counter;
  //!<stance leg (RIGHT/LEFT)
  uint8_t stance;
  //!<padding
  uint8_t zero[3];
  //!vector from world frame to left leg in world frame
  double t_wr_fr[3];
  //!vector from world frame to right leg in world frame
  double t_wr_fl[3];
  //!vector from world frame to left camera in world frame
  double t_wr_cl[3];
  //!vector from world frame to right camera in world frame
  double t_wr_cr[3];

  //!transformation matrix from left leg to world frame
  double R_wr_fr[3*3];
  //!transformation matrix from right leg to world frame
  double R_wr_fl[3*3];
  //!transformation matrix from left camera to world frame
  double R_wr_cl[3*3];
  //!transformation matrix from right camera to world frame
  double R_wr_cr[3*3];
  /*!
    transformation matrix from upper body coordinate frame
    to inertial frame measured by IMU (world frame)
    (identity matrix, if robot is standing upright)
  */
  double R_wr_ub[3*3];
  //!vector from world frame to upper body frame in world frame
  double t_wr_ub[3];
  //--------------------------------------------------------------------------- version 7 |
  //                                                                                      v
  //!vector from world frame to right knee in world frame
  double t_wr_kr[3];
  //!vector from world frame to left knee in world frame
  double t_wr_kl[3];
};
struct HR_Pose_V6
{
  enum{RIGHT=0,LEFT=1};
  //!<UDP port
  enum{PORT=0xD001};
  //!<data rate [ms] (actually determined by hardware trigger!)
  enum{RATE=50};
  //!data struct version
  uint32_t version;
  //! tick counter
  uint64_t tick_counter;
  //!<stance leg (RIGHT/LEFT)
  uint8_t stance;
  //!<padding
  uint8_t zero[3];
  //!vector from world frame to left leg in world frame
  double t_wr_fr[3];
  //!vector from world frame to right leg in world frame
  double t_wr_fl[3];
  //!vector from world frame to left camera in world frame
  double t_wr_cl[3];
  //!vector from world frame to right camera in world frame
  double t_wr_cr[3];

  //!transformation matrix from left leg to world frame
  double R_wr_fr[3*3];
  //!transformation matrix from right leg to world frame
  double R_wr_fl[3*3];
  //!transformation matrix from left camera to world frame
  double R_wr_cl[3*3];
  //!transformation matrix from right camera to world frame
  double R_wr_cr[3*3];
  /*!
    transformation matrix from upper body coordinate frame
    to inertial frame measured by IMU (world frame)
    (identity matrix, if robot is standing upright)
  */
  double R_wr_ub[3*3];
  //!vector from world frame to upper body frame in world frame
  double t_wr_ub[3];
};


#if (defined WIN32) || (defined WIN64)
#pragma pack(pop)
#else
_Pragma("pack()")
#endif//WIN32//64
#endif//__IFACE_VIS_H__
