/*!
  @file iface_wpatt_fsm.hpp
  
  (shm) interface to walking pattern fsm data
  
  Copyright (C)  Institute of Applied Mechanics, TU-Muenchen
  All rights reserved.
  
  
*/
#ifndef __IFACE_WPATT_FSM_HPP__
#define __IFACE_WPATT_FSM_HPP__
#include <stdint.h>
#include <begin_pack.h>
namespace am2b_iface
{
  /*!
    FSM walking state of robot
  */
  struct WPattFSMState
  {
    enum{RIGHT=0,LEFT=1} _enumfeet;
#ifndef __APPLE__
    enum{FALSE=0,TRUE=1} _enumbool;
#endif
    enum{STAND=0,WALK_START,WALK,WALK_STOP} _enumstate;
    
    //data
    //!stance leg (right/left)
    uint8_t stance;
    //!double support?
    //uint8_t double_support;
    //!left/right leg support
    //uint8_t support[2];
  };
  //shm path
  const char * const wpatt_fsm_state_shm_path="/am2b_wpatt_fsm_state";
  //utility macros
  //#define DECL_WPATT_SHM_SOURCE(x) ShmSource<am2b_iface::WPattFSMState> x(wpatt_fsm_state_shm_path)
}
#include <end_pack.h>
#endif//__IFACE_WPATT_FSM_HPP__
