/*!
  @file iface_rtview.hpp
  
  interface definition for real-time robot animation
  
  Copyright (C)  Institute of Applied Mechanics, TU-Muenchen
  All rights reserved.
  
  
*/
#ifndef __IFACE_RTVIEW_HPP__
#define __IFACE_RTVIEW_HPP__
#include <stdint.h>
#include <begin_pack.h>
namespace am2b_iface
{
  //!state of one robot segment (for visualisation)
  struct RTViewSegmentState
  {
    //!transform matrix
    float R[3*3];
    //!position
    float r[3];
  };
  
  struct RTViewRobotState
  {
    enum{IFACE_RTVIEW_VERSION=0x3};
    uint32_t  version;
    float     time;
    uint64_t  stamp;
    RTViewSegmentState state[1];
  };
}
#include <end_pack.h>
#endif// __IFACE_RTVIEW_HPP__
