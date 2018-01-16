//emacs -*-Mode: C++;-*-
/*!
  @file iface_sig_debug.hpp

  signals for debug domain

   Copyright (C)  Institute of Applied Mechanics, TU-Muenchen
   All rights reserved.
   

*/
#ifndef __IFACE_SIG_DEBUG_HPP__
#define __IFACE_SIG_DEBUG_HPP__
#include <stdint.h>
#include <iface_sig.hpp>
namespace am2b_iface
{
  const MsgId SET_DEBUG_INT_0                =   __MSG_ID_DEF_GLOBAL(__DOM_DEBUG,0x00);
  const MsgId SET_DEBUG_INT_1                =   __MSG_ID_DEF_GLOBAL(__DOM_DEBUG,0x01);

}
#endif//__IFACE_SIG_DEBUG_HPP__

