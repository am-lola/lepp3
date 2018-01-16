#ifndef __IFACE_PRIO_HPP__
#define __IFACE_PRIO_HPP__
#include <stdint.h>
#include <pthread.h>
#include <errno.h>

#ifndef EOK //LINUX doesn't define EOK
#define EOK 0
#endif

//scheduling priorities. assumtion: prio_max-prio_min>50. linux has 100, qnx 255
//

inline int PRIO_EC_TIMING() {
  // Ec timing task prio
  return sched_get_priority_max(SCHED_FIFO);
}

inline int PRIO_EC_JOBTASK() {
  // Ec frame dispatching thread prio
  return sched_get_priority_max(SCHED_FIFO)-1;
}

inline int PRIO_HWIO()
{
  // hwl_ec main loop prio
  return sched_get_priority_max(SCHED_FIFO)-2;
}

inline int PRIO_WD()
{
  // watchdog and gpio prio
  return sched_get_priority_max(SCHED_FIFO)-3;
}

inline int PRIO_CONT()
{
  return sched_get_priority_max(SCHED_FIFO)-10;
}

inline int PRIO_WPATT()
{
  return sched_get_priority_max(SCHED_FIFO)-20;
}

inline int PRIO_PS_SVC()
{
  return sched_get_priority_min(SCHED_FIFO)+20;
}

inline int PRIO_PS_EVT()
{
  return sched_get_priority_max(SCHED_FIFO)-30;
}


inline int PRIO_VIS()
{
  return sched_get_priority_max(SCHED_FIFO)-40;
}

inline int PRIO_LOG()
{
  //default priority 10 on QNX
  return sched_get_priority_min(SCHED_FIFO)+10;
}

// Added Valerio--
inline int PRIO_JCONT()
{
  return sched_get_priority_max(SCHED_FIFO)-5;
}

inline int PRIO_SCHED()
{
  return sched_get_priority_max(SCHED_FIFO);
}
//--------------------END

inline int SET_PRIO(int prio)
{
  struct sched_param param;
  int policy;
  if(EOK!= pthread_getschedparam(pthread_self(),
				 &policy,
				 &param))
    {
      //perr_ffl("error getting scheduling parameters");
      return -1;
    }
  policy=SCHED_FIFO;
  param.sched_priority=prio;
  if(EOK!= pthread_setschedparam(pthread_self(),
				 policy,
				 &param))
    {
      //perr_ffl("error setting scheduling parameters\n");
      return -1;
    }
  return 0;
}

#endif//__IFACE_PRIO_HPP__
