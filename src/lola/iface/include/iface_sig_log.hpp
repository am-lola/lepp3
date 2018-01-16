//emacs -*-Mode: C++;-*-
/*!
  @file iface_sig_log.hpp

  signals for log domain (data logging)

   Copyright (C)  Institute of Applied Mechanics, TU-Muenchen
   All rights reserved.
   

*/
#ifndef __IFACE_SIG_LOG_HPP__
#define __IFACE_SIG_LOG_HPP__
#include <stdint.h>
#include <iface_sig.hpp>
namespace am2b_iface
{
  //////////////////////////////////////////////////
  // data logging
  ///////////////////////////////////////////////////
  //dump data buffer
  const MsgId SIG_LOG_DUMP                   = __MSG_ID_DEF_GLOBAL(__DOM_LOG,0x1);
  //start data log
  const MsgId SIG_LOG_START                  = __MSG_ID_DEF_GLOBAL(__DOM_LOG,0x2);
  //stop data log
  const MsgId SIG_LOG_STOP                   = __MSG_ID_DEF_GLOBAL(__DOM_LOG,0x3);
}


#endif//__IFACE_SIG_COMMANDS_HPP__

