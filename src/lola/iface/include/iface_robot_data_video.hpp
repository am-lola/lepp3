/*!
  @file wpatt_data_johnnie.hpp
 
 logging of pose data to frame i of video

  Copyright (C)  Institute of Applied Mechanics, TU-Muenchen
  All rights reserved.
  
  
*/
#ifndef __IFACE_ROBOT_DATA_VIDEO_HPP__
#define __IFACE_ROBOT_DATA_VIDEO_HPP__
#include <stdint.h>

#include <begin_pack.h>
namespace am2b_iface
{
  namespace lola
  {
    struct RobotDataVideo
    {
      double r_cam_i[3];
      double AT_cam_i[9];
      double r_stance_odo[3];
      double phi_z_odo;
      uint64_t stance;
      //number of frame in video
      uint64_t frame;
      //stamp of robot data
      uint64_t stamp;
      
    };
  } //lola
}//am2b_iface
#include <end_pack.h>
#endif //__IFACE_ROBOT_DATA_VIDEO_HPP__
