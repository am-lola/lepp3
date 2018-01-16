/*!
@file iface_ports.hpp

 port numbers for TCP/IP communication

 Copyright (C)  Institute of Applied Mechanics, TU-Muenchen
 All rights reserved.
 

*/
#ifndef __IFACE_PORTS_HPP__
#define __IFACE_PORTS_HPP__
#include <stdint.h>
namespace am2b_iface
{
  //port range not reserved by IANA
  const uint16_t PORT_PRIVATE_MIN=0xC000;
  const uint16_t PORT_PRIVATE_MAX=0xFFFF;
  //port range used for robot
  const uint16_t PORT_MIN=0xF000;
  const uint16_t PORT_MAX=0xFFFF;

  //!port for wpatt rpc server
  const uint16_t PORT_WPATT_RPC=PORT_MIN;
  //!port for wpatt data server
  const uint16_t PORT_WPATT_DATA=PORT_MIN+0x1;  
  //!rpc for lola movic demo
  const uint16_t PORT_LOLA_MOVIC_RPC=PORT_MIN+0x2;  
  //! for test purposes
  const uint16_t PORT_TEST=PORT_MIN+0x3;
  //!data server port for devr-hwj
  const uint16_t PORT_HWJ_DATA=PORT_MIN+0x4;
  //!data server port for devr-hwl
  const uint16_t PORT_HWL_DATA=PORT_MIN+0x5;
  //!real-time robot animation
  const uint16_t PORT_RTVIEW=PORT_MIN+0x6;
  //!real-time data plot
  const uint16_t PORT_RTPLOT=PORT_MIN+0x7;
  //!publish/subscribe tcp server port
  const uint16_t PORT_PS=PORT_MIN+0x8;
}
#endif//__PORTS_HPP__
