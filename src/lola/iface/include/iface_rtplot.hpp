/*!
  @file iface_rtplot.hpp
  
  interface definition for real-time data plot
  
  Copyright (C)  Institute of Applied Mechanics, TU-Muenchen
  All rights reserved.
  
  
*/
#ifndef __IFACE_RTPLOT_HPP__
#define __IFACE_RTPLOT_HPP__
#include <stdint.h>
#include <begin_pack.h>
namespace am2b_iface
{
  const size_t RTPLOT_STEPS_PER_SEND=50;
  
  //!real-time plot data
  struct RTPlotData
  {
    float time[RTPLOT_STEPS_PER_SEND];
    float F[2][3][RTPLOT_STEPS_PER_SEND];
    float T[2][3][RTPLOT_STEPS_PER_SEND];
    float phi[3][RTPLOT_STEPS_PER_SEND];
    float dot_phi[3][RTPLOT_STEPS_PER_SEND];
    float d_debug[10][RTPLOT_STEPS_PER_SEND];
  };
}
#include <end_pack.h>
#endif// __IFACE_RTVIEW_HPP__
