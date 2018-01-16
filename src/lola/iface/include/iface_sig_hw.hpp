//emacs -*-Mode: C++;-*-
/*!
  @file iface_sig_hw.hpp

   hardware I/O, low level control

   Copyright (C)  Institute of Applied Mechanics, TU-Muenchen
   All rights reserved.
   

*/
#ifndef __IFACE_SIG_HW_HPP__
#define __IFACE_SIG_HW_HPP__
#include <stdint.h>
#include <iface_sig.hpp>
namespace am2b_iface
{
  /**********************************************************************
   *     LOLA LOW LEVEL CONTROL AND I/O MESSAGE IDS
   **********************************************************************/
  //!reset force/torque sensor
  const MsgId SIG_HW_RESET_FTS                 =   __MSG_ID_DEF_GLOBAL(__DOM_HW,0x5);
  //!optimize ad-baudrate for fts
  const MsgId SIG_HW_TUNE_AD_FTS               =   __MSG_ID_DEF_GLOBAL(__DOM_HW,0x6);
    //!reset inertial measurement unit
  const MsgId SIG_HW_RESET_IMU                 =   __MSG_ID_DEF_GLOBAL(__DOM_HW,0x7);
    //!reset errors
  const MsgId SIG_HW_RESET_ERROR               =   __MSG_ID_DEF_GLOBAL(__DOM_HW,0x8);
    //!goto idle mode
  const MsgId SIG_HW_GOTO_IDLE                 =   __MSG_ID_DEF_GLOBAL(__DOM_HW,0x9);
    //!goto mfree mode
  const MsgId SIG_HW_GOTO_HOMING               =   __MSG_ID_DEF_GLOBAL(__DOM_HW,0xA);
    //!goto position control mode
  const MsgId SIG_HW_GOTO_PCONT                =   __MSG_ID_DEF_GLOBAL(__DOM_HW,0xB);
    //!goto position control mode
  const MsgId SIG_HW_GOTO_FAULT                =   __MSG_ID_DEF_GLOBAL(__DOM_HW,0xC);
    //!goto optimize velocity gain mode
  const MsgId SIG_HW_GOTO_OPT                  =   __MSG_ID_DEF_GLOBAL(__DOM_HW,0xD);

  //!set proportional gain for position joint control
  const MsgId SET_HW_ELMO_PCONT_KP               =   __MSG_ID_DEF_GLOBAL(__DOM_HW,0x13);
  //!set proportional gain for velocity joint control
  const MsgId SET_HW_ELMO_VCONT_KP               =   __MSG_ID_DEF_GLOBAL(__DOM_HW,0x14);
  //!set derivative gain for velocity joint control
  const MsgId SET_HW_ELMO_VCONT_KI               =   __MSG_ID_DEF_GLOBAL(__DOM_HW,0x15);
  //!set proportional gain for current joint control
  const MsgId SET_HW_ELMO_ICONT_KP               =   __MSG_ID_DEF_GLOBAL(__DOM_HW,0x16);
  //!set derivative gain for current joint control
  const MsgId SET_HW_ELMO_ICONT_KI               =   __MSG_ID_DEF_GLOBAL(__DOM_HW,0x17);
  //!set inverse dynamics feedforward for current joint control
  const MsgId SET_HW_ELMO_ICONT_FF               =   __MSG_ID_DEF_GLOBAL(__DOM_HW,0x18);


  //!current HWIOLola data 
  const MsgId SIG_HW_IO_DATA_LOLA                =   __MSG_ID_DEF_GLOBAL(__DOM_HW,0x101);

  //! add offset to IMU data
  const MsgId SET_HW_IMU_OFFSET_PHI0            =   __MSG_ID_DEF_GLOBAL(__DOM_HW,0x201); 
  const MsgId SET_HW_IMU_OFFSET_PHI1            =   __MSG_ID_DEF_GLOBAL(__DOM_HW,0x202);   
}
#endif//__IFACE_SIG_COMMANDS_HPP__

