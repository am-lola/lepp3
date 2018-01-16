#ifndef __IFACE_RTVIEW_LOLA_HPP__
#define __IFACE_RTVIEW_LOLA_HPP__
#include <stdint.h>
#include <iface_rtview.hpp>
#include <iface_model_lola.hpp>
#include <begin_pack.h>
namespace am2b_iface
{
  namespace lola
  {
    class RTViewLolaState: 
      public RTViewRobotState
    {
#ifdef HEAD_LOLA
      RTViewSegmentState state_[N_SEGMENTS-1+2];
#else
      RTViewSegmentState state_[N_SEGMENTS-1];
#endif
    };
  } //lola
}//am2b_iface
#include <end_pack.h>
#endif//__IFACE_RTVIEW_LOLA_HPP__
