#ifndef __IFACE_PS_HPP__
#define __IFACE_PS_HPP__
#include <stdint.h>
#include <iface_sig.hpp>
#include <begin_pack.h>
namespace am2b_iface
{
  namespace ps
  {
    //////////////////////////////////////////////////
    // queue names
    ///////////////////////////////////////////////////
    //!queue for service messages
    const char mq_svc_name[]="/ps_svc\0";
    //!queue for real-time messages (publish to this queue)
    const char mq_pub_name[]="/ps_evt\0";
    
    //////////////////////////////////////////////////
    // publish/subscribe service signals
    ///////////////////////////////////////////////////
    //Note: Ids are local, since they are not meant to be published
    // //!register message queue 
    const MsgId SIG_PS_REGISTER  =__MSG_ID_DEF_LOCAL(__DOM_PS,   0x1);
    // //!unregister message queue 
    const MsgId SIG_PS_UNREGISTER  =__MSG_ID_DEF_LOCAL(__DOM_PS, 0x2);
    
    
    //!subscribe to a signal domain
    const MsgId SIG_PS_SUBSCRIBE   =__MSG_ID_DEF_LOCAL(__DOM_PS, 0x3);
    //!unsubscribe from a signal domain
    const MsgId SIG_PS_UNSUBSCRIBE =__MSG_ID_DEF_LOCAL(__DOM_PS, 0x4);
    
    //!shutdown ps-broker program (for debugging purposes only!)
    const MsgId SIG_PS_BROKER_SHUTDOWN =__MSG_ID_DEF_LOCAL(__DOM_PS, 0xABCD);

    //!service message 
    struct svc_msg
    {
      enum{MAXSZ=128};
      //!message id
      MsgId id;
      //!mqueue name
      char name[MAXSZ];
      //!client process id
      //int32_t pid;
    };
  }//ps
}//am2b_iface
#include <end_pack.h>
#endif//__IFACE_PS_HPP__
