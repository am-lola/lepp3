//emacs -*-Mode: C++;-*-
/*!
  @file iface_sig_vis.hpp

  signals for vision system domain

   Copyright (C)  Institute of Applied Mechanics, TU-Muenchen
   All rights reserved.
   

*/
#ifndef __IFACE_SIG_VIS_HPP__
#define __IFACE_SIG_VIS_HPP__
#include <stdint.h>
#include <iface_sig.hpp>
#include <begin_pack.h>

namespace am2b_iface
{
  
  /***********************************************************************
   *    PARAMETERS FOR VISION SYSTEM
   ***********************************************************************/
  //!enable/disable control input from vision system
  const MsgId SIG_VIS_CONTROL_ON                 =   __MSG_ID_DEF_GLOBAL(__DOM_VIS,0x1);
  //!reset pose counter 
  const MsgId SIG_VIS_RESET_HRPOSE_CNT           =   __MSG_ID_DEF_GLOBAL(__DOM_VIS,0x2);
  //!enable/disable viewing target tracking
  //const MsgId SIG_VIS_TRACKING_ON                =   __MSG_ID_DEF_GLOBAL(__DOM_VIS,0x3);

  //!enable/disable walking control (walking velocity input)
  const MsgId SIG_VIS_WALK_CONTROL_ON            =   __MSG_ID_DEF_GLOBAL(__DOM_VIS,0x3);
  //!enable/disable walking control (velocity input)
  const MsgId SIG_VIS_VIEW_CONTROL_ON            =   __MSG_ID_DEF_GLOBAL(__DOM_VIS,0x4);
  //!new odometry data (VisPoseWorld)
  const MsgId SIG_NEW_VIS_POSE_WORLD             =   __MSG_ID_DEF_GLOBAL(__DOM_VIS,0x5);
  //!(desired) walking velocity (VisVelAct)
  const MsgId SIG_NEW_VIS_VEL_ACT                =   __MSG_ID_DEF_GLOBAL(__DOM_VIS,0x6);


  //Camera calibration data
  const MsgId R_C_CONV_X                         =   __MSG_ID_DEF_GLOBAL(__DOM_VIS,0x7);
  const MsgId R_C_CONV_Y                         =   __MSG_ID_DEF_GLOBAL(__DOM_VIS,0x8);
  const MsgId R_C_CONV_Z                         =   __MSG_ID_DEF_GLOBAL(__DOM_VIS,0x9);
  const MsgId CAM_CALIB_ALPHA_X                  =   __MSG_ID_DEF_GLOBAL(__DOM_VIS,0xA);
  const MsgId CAM_CALIB_ALPHA_Y                  =   __MSG_ID_DEF_GLOBAL(__DOM_VIS,0xB);
  const MsgId CAM_CALIB_ALPHA_Z                  =   __MSG_ID_DEF_GLOBAL(__DOM_VIS,0xC);

  struct VisPoseWorld
  {
    int32_t stance_leg;
    int32_t wpatt_state;
    float   r_stance_odo[3];
    float   phi_z_odo;
  };

  struct VisVelAct
  {
    float vx_act;
    float vy_act;
    float om_act;
  };
}
#include <end_pack.h>
#endif//__IFACE_SIG_VIS_HPP__
