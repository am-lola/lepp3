#ifndef __IFACE_SIG_HPP__
#define __IFACE_SIG_HPP__
/*!
  @file iface_sig.hpp
  
  define global signal ranges here to get globally unique identifyers
  
  Copyright (C)  Institute of Applied Mechanics, TU-Muenchen
  All rights reserved.
  
  
*/
#include <stdint.h>
#include <stdint.h>

namespace am2b_iface
{
  
  typedef uint8_t MsgDom;
  typedef uint16_t MsgSubId;
  typedef uint32_t MsgId;// [Domain,SubId]
  typedef uint32_t MsgLen;
  
  /*
    MsgId Structure:
    bit31:       global/local
    bit24..30:   reserved (future use)
    bit16..23:   domain
    bit0..15:    subid
   */
// #include <begin_pack.h>
//   typedef union{
//   uint32_t id;
//   struct{
//   uint8_t global:1;
//   uint8_t reserved:7;
//   uint8_t  domain:8;
//   uint16_t subid:16;
// }part;
// } MsgId;

// #include <end_pack.h>


//   //!true if signal is global (not local)
// #define __MSG_ID_IS_GLOBAL(id) (id.part.global)
//   //!true if signal is local (not global)
// #define __MSG_ID_IS_LOCAL(id) (!__MSG_ID_IS_GLOBAL(id))
//   //!get msg domain from signal
// #define __MSG_ID_DOM(id) (id.part.domain)
//   //!get msg subid from id
// #define __MSG_ID_SUBID(id) (id.part.subid)
//   //!define local message id
// #define __MSG_ID_DEF_LOCAL(dom,sig)   ({})
//   //!define global message id
// #define __MSG_ID_DEF_GLOBAL(dom,sig)  (0x80000000 | ((dom&0xFF)<<0x10) | (sig&0xFFFF))

// #define __MSG_ID_MASK_DOM(dom) (dom & 0xFF)

  //!global if first bit is set
#define __MSG_ID_IS_GLOBAL(SIG) (SIG & 0x80000000) 
  //!local if first bit is not set
#define __MSG_ID_IS_LOCAL(SIG) (!(__MSG_ID_IS_GLOBAL(SIG))
  //!message domain (bit 16..24)
#define __MSG_ID_DOM(SIG) ((SIG&0xFF0000)>>0x10)
  //!message sub-id (bit 0..15)
#define __MSG_ID_SUBID(SIG) ((SIG&0xFFFF))
  //!define local message id
#define __MSG_ID_DEF_LOCAL(dom,sig)   (((dom&0xFF)<<0x10) | (sig&0xFFFF))
  //!define global message id
#define __MSG_ID_DEF_GLOBAL(dom,sig)  (0x80000000 | ((dom&0xFF)<<0x10) | (sig&0xFFFF))

#define __MSG_ID_MASK_DOM(dom) (dom & 0xFF)

  //!communication layer
#define __DOM_COM           0x0
  //!internal fsm signals
#define __DOM_FSM           0x1
  //!walking pattern FSM (internal)
#define __DOM_WPATT_FSM     0x2
  //!walking control
#define __DOM_CONTROL       0x3
  //!walking pattern generation
#define __DOM_WPATT         0x4
#define __DOM_MAIN          0x5
  //!vision system 
#define __DOM_VIS           0x6
  //!robot model
#define __DOM_MOD           0x7
  //!meta object system
#define __DOM_META          0x8
  //!test slot  
#define __DOM_TEST          0x9
  //!debug
#define __DOM_DEBUG         0xA
  //!movic demo lola  
#define __DOM_MOVIC         0xB
  //!hardware I/O, low level control
#define __DOM_HW            0xC
  //!publish/subscribe system 
#define __DOM_PS            0x10
  //!data log/ ring buffer dump
#define __DOM_LOG           0x11

  //!joint test program
#define __DOM_JTEST         0x12

  //!walking control program
#define __DOM_WSTAB_PROG    0x13

  //!lola low level controller fsm
#define __DOM_HWL_FSM       0x14

//! Named topics via PubSubConfig
#define __DOM_NAMED         0x15

  //!remote syscall (test)
#define __DOM_REMOTE_SYSCALL 0x20

  const MsgId SIG_INVALID=0xFFFF;
}
#endif//__IFACE_SIG_HPP__
