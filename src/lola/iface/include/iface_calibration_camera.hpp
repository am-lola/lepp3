/*!
  @file iface_calibration_camera.hpp

data for camera calibration (A and r of camera)  

  Copyright (C)  Institute of Applied Mechanics, TU-Muenchen
  All rights reserved.
  
  
*/
#ifndef __IFACE_CALIB_CAM_LOLA_HPP__
#define __IFACE_CALIB_CAM_LOLA_HPP__
#include <stdint.h>

#include <begin_pack.h>
namespace am2b_iface
{
  namespace lola
  {
    struct CalibCamData
    {
      double r_cam_i[3];//without calibration matrix ;)
      double AT_cam_i[9];//without calibration matrix ;)
    };
  } //lola
}//am2b_iface
#include <end_pack.h>
#endif//__IFACE_WPATT_LOLA_HPP__
