//emacs -*-Mode: C++;-*-
/*!
  @file iface_sig_msg.hpp

  signals  for basic message based communication

   Copyright (C)  Institute of Applied Mechanics, TU-Muenchen
   All rights reserved.
   

*/
#ifndef __IFACE_SIG_MSG_HPP__
#define __IFACE_SIG_MSG_HPP__
#include <stdint.h>
#include <stdint.h>
#include <iface_sig.hpp>


namespace am2b_iface
{
  /*
    note: ids are local, since they should not be published. 
    i.e. if they are used for communication between processes, 
    they are explicitly sent via e.g. TCP/IP and not 
    forwarded by the local publish/subscribe broker singleton
   */
  //TCP/IP communication, also publish/subscribe stuff
  const MsgId COM_RAW                   = __MSG_ID_DEF_LOCAL(__DOM_COM,0x0);       //raw data packet
  const MsgId COM_TERM                  = __MSG_ID_DEF_LOCAL(__DOM_COM,0x1);       //terminate connection 
  const MsgId COM_ERR                   = __MSG_ID_DEF_LOCAL(__DOM_COM,0x2);       //com error
  const MsgId COM_EOK                   = __MSG_ID_DEF_LOCAL(__DOM_COM,0x3);       //com ok
  const MsgId COM_FAIL                  = __MSG_ID_DEF_LOCAL(__DOM_COM,0x4);       //com failure (processing error, request failed)
  const MsgId COM_NIMPL                 = __MSG_ID_DEF_LOCAL(__DOM_COM,0x5);       //com function not implemented
  const MsgId SIG_TICK                  = __MSG_ID_DEF_LOCAL(__DOM_COM,0x6);       //new tick of the clock
  //rpc-functionality	                 
  const MsgId RPC_INT_INT               = __MSG_ID_DEF_LOCAL(__DOM_COM,0x10);       //rpc, int argument and int result
  const MsgId RPC_FLT_FLT               = __MSG_ID_DEF_LOCAL(__DOM_COM,0x11);       //rpc, float argument and result
  const MsgId RPC_FLT_INT               = __MSG_ID_DEF_LOCAL(__DOM_COM,0x12);       //rpc, float argument int result
  //const MsgId RPC_BLOB_INT              = __MSG_ID_DEF_LOCAL(__DOM_COM,0x13);       //rpc, blob argument int result
  //const MsgId RPC_INT_BLOB              = __MSG_ID_DEF_LOCAL(__DOM_COM,0x14);       //rpc, int argument blob result
  //data server functionality
  const MsgId DATA_SERV_GET             = __MSG_ID_DEF_LOCAL(__DOM_COM,0x30);       //request for data
  const MsgId DATA_SERV_STREAM_START    = __MSG_ID_DEF_LOCAL(__DOM_COM,0x31);       //request for data
  const MsgId DATA_SERV_STREAM_STOP     = __MSG_ID_DEF_LOCAL(__DOM_COM,0x32);       //request for data
  const MsgId DATA_SERV_SET_PACKNO      = __MSG_ID_DEF_LOCAL(__DOM_COM,0x33);      //set no. of packets to send at once
  //const MsgId DATA_SERV_STOP   = __MSG_ID_DEF_LOCAL(__DOM_COM,0x10);       //stop getting data 

}
#endif//__IFACE_SIG_MSG_HPP__
