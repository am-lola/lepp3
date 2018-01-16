/*!
  @file iface_rtlog_lola.hpp
  
  Interface definition real-time data log
*/
#ifndef __IFACE_RTLOG_LOLA_H__
#define __IFACE_RTLOG_LOLA_H__
#include <stdint.h>
#include <iface_hwio_lola.hpp>
#include <iface_wpatt_lola.hpp>
#include <begin_pack.h>
namespace am2b_iface
{
  namespace lola
  {
    struct rtlog_data_lola
    {
      LolaSensorDataShm hwo;
      LolaInputDataShm hwi;
      //soll_zustand_lola soll_zustand;
      int   stance;//stance/swing leg
      float T_id[2][3];//ideal torques (traj. planning)
      float F_id[2][3];//ideal forces  (traj. planning)
      float T_fc[2][3];//input to force control
      float F_fc[2][3];//input to force control
      
    };
  }
}
#include <end_pack.h>
#endif//__IFACE_RTLOG_LOLA_H__
