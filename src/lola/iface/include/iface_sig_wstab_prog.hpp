/*!
  @file iface_sig_wstab_prog.hpp

  signals / command ids for WSTAB_PROG domain

  Copyright (C)  Institute of Applied Mechanics, TU-Muenchen
  All rights reserved.
  

*/
#ifndef __IFACE_SIG_WSTAB_PROG_HPP__
#define __IFACE_SIG_WSTAB_PROG_HPP__
#include <stdint.h>
#include <iface_sig.hpp>
namespace am2b_iface
{
  //!next control cycle
  const MsgId SIG_WSTAB_PROG_TICK       =  __MSG_ID_DEF_LOCAL(__DOM_WSTAB_PROG,0x1);
  //!goto mfree state
  const MsgId SIG_WSTAB_PROG_GOTO_MFREE =  __MSG_ID_DEF_GLOBAL(__DOM_WSTAB_PROG,0x2);
  //!goto walking control state
  const MsgId SIG_WSTAB_PROG_GOTO_WALK  =  __MSG_ID_DEF_GLOBAL(__DOM_WSTAB_PROG,0x3);

  //!new fsm state
  const MsgId SIG_WSTAB_FSM_STATE_CHANGED  =  __MSG_ID_DEF_GLOBAL(__DOM_WSTAB_PROG,0x4);


  struct WStabProgFsmStates
  {
    enum STATE{INIT=0,
               MFREE=1,
               WAIT_ELMO_OP=2,
               GOTO_WPATT_POSE=3,
               WALK=4,
               FAULT=5,
               NUM_STATES=6};
  };
}
#endif//__IFACE_SIG_WSTAB_PROG_HPP__

