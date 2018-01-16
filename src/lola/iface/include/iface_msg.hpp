//emacs -*-Mode: C++;-*-
/*!
  @file msg.hpp

   structs for basic message based communication

   Copyright (C)  Institute of Applied Mechanics, TU-Muenchen
   All rights reserved.
   

*/
#ifndef __IFACE_MSG_HPP__
#define __IFACE_MSG_HPP__

#include <stdint.h>
#include <iface_sig.hpp>
#include <begin_pack.h>
namespace am2b_iface
{
  /*!
    Message header
    len is the length of the message
    id is the message id
  */
  struct MsgHeader
  {
    am2b_iface::MsgId id;
    am2b_iface::MsgLen len;
  };

  /**************************************************
    remote procedure call layer
  **************************************************/
  // struct RPC_DATA_INT
  // {
  //   am2b_iface::MsgId  id;
  //   int32_t arg;
  // };
  // struct RPC_DATA_FLT
  // {
  //   am2b_iface::MsgId id;
  //   float  arg;  //should be IEEE floating point variable, 32bit.
  //                // le/be isn't checked
  // };

}//am2b_iface
#include <end_pack.h>
#endif//__IFACE_MSG_HPP__
