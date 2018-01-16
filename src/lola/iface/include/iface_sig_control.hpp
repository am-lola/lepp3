//emacs -*-Mode: C++;-*-
/*!
  @file iface_sig_control.hpp

  signals for control domain

  Copyright (C)  Institute of Applied Mechanics, TU-Muenchen
  All rights reserved.
  

*/
#ifndef __IFACE_SIG_CONTROL_HPP__
#define __IFACE_SIG_CONTROL_HPP__
#include <stdint.h>
#include <iface_sig.hpp>
namespace am2b_iface
{
  //!enable/disable force control (EventInt)
  const MsgId SIG_FCTRL_ENABLE                      =   __MSG_ID_DEF_GLOBAL(__DOM_CONTROL,0x17);

  const MsgId REGDPSI_AN                     =   __MSG_ID_DEF_GLOBAL(__DOM_CONTROL,0x18);
  const MsgId SET_REGDPSI_KP_PHI0            =   __MSG_ID_DEF_GLOBAL(__DOM_CONTROL,0x19);
  const MsgId SET_REGDPSI_KD_PHI0	     =   __MSG_ID_DEF_GLOBAL(__DOM_CONTROL,0x1A);
  const MsgId SET_REGDPSI_KI_TX  	     =	 __MSG_ID_DEF_GLOBAL(__DOM_CONTROL,0x1B);
  const MsgId SET_REGDPSI_KP_PHI1	     =	 __MSG_ID_DEF_GLOBAL(__DOM_CONTROL,0x1C);
  const MsgId SET_REGDPSI_KD_PHI1	     =	 __MSG_ID_DEF_GLOBAL(__DOM_CONTROL,0x1D);
  const MsgId SET_REGDPSI_KI_TY              =   __MSG_ID_DEF_GLOBAL(__DOM_CONTROL,0x1E);
  const MsgId SET_REGDPSI_D_OFFSET_FUSSLAGE  =   __MSG_ID_DEF_GLOBAL(__DOM_CONTROL,0x1F);

  const MsgId REGDZ_AN                       =   __MSG_ID_DEF_GLOBAL(__DOM_CONTROL,0x20);
  const MsgId SET_REGDZ_K_BODEN_STOSS        =   __MSG_ID_DEF_GLOBAL(__DOM_CONTROL,0x21);
  const MsgId SET_REGDZ_T1_STOSS             =   __MSG_ID_DEF_GLOBAL(__DOM_CONTROL,0x22);
  const MsgId SET_REGDZ_DZ_STOSS_MAX         =   __MSG_ID_DEF_GLOBAL(__DOM_CONTROL,0x23);
  const MsgId SET_T1_EINFEDERUNG_DZ          =   __MSG_ID_DEF_GLOBAL(__DOM_CONTROL,0x24);

  const MsgId EXIT_CONTROL                   =   __MSG_ID_DEF_GLOBAL(__DOM_CONTROL,0x25);

  //   const MsgId SET_OFFSET_IMU_PHI0            =   __MSG_ID_DEF_GLOBAL(__DOM_CONTROL,0x3A);
  //   const MsgId SET_OFFSET_IMU_PHI1            =   __MSG_ID_DEF_GLOBAL(__DOM_CONTROL,0x3B);


  //ICONT_COG_3D
  const MsgId SET_ICONTCOG3D_KPX             =   __MSG_ID_DEF_GLOBAL(__DOM_CONTROL,0x30);
  const MsgId SET_ICONTCOG3D_KDX             =   __MSG_ID_DEF_GLOBAL(__DOM_CONTROL,0x31);
  const MsgId SET_ICONTCOG3D_KPY             =   __MSG_ID_DEF_GLOBAL(__DOM_CONTROL,0x32);
  const MsgId SET_ICONTCOG3D_KDY             =   __MSG_ID_DEF_GLOBAL(__DOM_CONTROL,0x33);
  const MsgId SET_ICONTCOG3D_KPZ             =   __MSG_ID_DEF_GLOBAL(__DOM_CONTROL,0x34);
  const MsgId SET_ICONTCOG3D_KDZ             =   __MSG_ID_DEF_GLOBAL(__DOM_CONTROL,0x35);
  //swing leg control
  //  const MsgId SET_ICONTCOG3D_SLC             =   __MSG_ID_DEF_GLOBAL(__DOM_CONTROL,0x36);
  const MsgId SET_ICONTCOG3D_DXL             =   __MSG_ID_DEF_GLOBAL(__DOM_CONTROL,0x37);
  const MsgId SET_ICONTCOG3D_DYL             =   __MSG_ID_DEF_GLOBAL(__DOM_CONTROL,0x38);
  const MsgId SET_ICONTCOG3D_DXL_ENABLE      =   __MSG_ID_DEF_GLOBAL(__DOM_CONTROL,0x39);
  const MsgId SET_ICONTCOG3D_DYL_ENABLE      =   __MSG_ID_DEF_GLOBAL(__DOM_CONTROL,0x40);




  //position/force control
  const MsgId SET_FPIK_KTX                   =    __MSG_ID_DEF_GLOBAL(__DOM_CONTROL,0x50);
  const MsgId SET_FPIK_KTY                   =    __MSG_ID_DEF_GLOBAL(__DOM_CONTROL,0x51);
  const MsgId SET_FPIK_KTZ                   =    __MSG_ID_DEF_GLOBAL(__DOM_CONTROL,0x52);
  const MsgId SET_FPIK_KFX                   =    __MSG_ID_DEF_GLOBAL(__DOM_CONTROL,0x53);
  const MsgId SET_FPIK_KFY                   =    __MSG_ID_DEF_GLOBAL(__DOM_CONTROL,0x54);
  const MsgId SET_FPIK_KFZ                   =    __MSG_ID_DEF_GLOBAL(__DOM_CONTROL,0x55);
  const MsgId SET_FPIK_KLX                   =    __MSG_ID_DEF_GLOBAL(__DOM_CONTROL,0x56);
  const MsgId SET_FPIK_INT_KL                =    __MSG_ID_DEF_GLOBAL(__DOM_CONTROL,0x57);

  const MsgId SET_FPIK_KX_GAIN               =    __MSG_ID_DEF_GLOBAL(__DOM_CONTROL,0x58);
  const MsgId SET_FPIK_KP_ICONT              =    __MSG_ID_DEF_GLOBAL(__DOM_CONTROL,0x59);

  const MsgId SET_SLEG_KX                    =    __MSG_ID_DEF_GLOBAL(__DOM_CONTROL,0xF0);
  const MsgId SET_SLEG_KY                    =    __MSG_ID_DEF_GLOBAL(__DOM_CONTROL,0xF1);

  const MsgId SET_ICONT_KP_ZCOM              =    __MSG_ID_DEF_GLOBAL(__DOM_CONTROL,0xF2);
  const MsgId SET_ICONT_KD_ZCOM              =    __MSG_ID_DEF_GLOBAL(__DOM_CONTROL,0xF3);

  const MsgId SET_ICONT_FZ_STAB_FACTOR       =    __MSG_ID_DEF_GLOBAL(__DOM_CONTROL,0xF4);

  //debug/test
  //!set reference force amplitude (EventFlt)
  const MsgId SET_FPIK_DBG_FD_WAVE_AMP_TX       =    __MSG_ID_DEF_GLOBAL(__DOM_CONTROL,0x101);
  //!set reference force amplitude (EventFlt)
  const MsgId SET_FPIK_DBG_FD_WAVE_AMP_TY       =    __MSG_ID_DEF_GLOBAL(__DOM_CONTROL,0x102);
  //!set reference force amplitude (EventFlt)
  const MsgId SET_FPIK_DBG_FD_WAVE_AMP_FZ       =    __MSG_ID_DEF_GLOBAL(__DOM_CONTROL,0x103);
  //!set reference motion amplitude / work space foot motion (EventFlt)
  const MsgId SET_FRICT_DBG_WAVE_AMP_TX         =    __MSG_ID_DEF_GLOBAL(__DOM_CONTROL,0x104);
  //!set  reference motion amplitude/ work space foot motion (EventFlt)
  const MsgId SET_FRICT_DBG_WAVE_AMP_TY         =    __MSG_ID_DEF_GLOBAL(__DOM_CONTROL,0x105);
  //!set  reference motion period/ work space foot motion (EventFlt)
  const MsgId SET_FRICT_DBG_WAVE_PER_TX         =    __MSG_ID_DEF_GLOBAL(__DOM_CONTROL,0x106);
  //!set  reference motion period/ work space foot motion (EventFlt)
  const MsgId SET_FRICT_DBG_WAVE_PER_TY         =    __MSG_ID_DEF_GLOBAL(__DOM_CONTROL,0x107);

  //deformation compensation
  //!stiffness for deformation compensation
  //const MsgId SET_DEFORM_COMP_CZ             =    __MSG_ID_DEF_GLOBAL(__DOM_CONTROL,0x201);

  //inclination compensation
  //!inclination compensation translation z component gain EventFlt [0..1]
  const MsgId SET_INCL_COMP_Z_GAIN               =   __MSG_ID_DEF_GLOBAL(__DOM_CONTROL,0x201);
  //!inclination compensation angle 0 component gain EventFlt [0..1]
  const MsgId SET_INCL_COMP_PSI0_GAIN            =   __MSG_ID_DEF_GLOBAL(__DOM_CONTROL,0x202);
  //!inclination compensation angle 1 component gain EventFlt [0..1]
  const MsgId SET_INCL_COMP_PSI1_GAIN            =   __MSG_ID_DEF_GLOBAL(__DOM_CONTROL,0x203);
  
  //Robert
  //!set dynamics error compensation to on/off [0..1]
  const MsgId SET_DYN_ERR_COMP_GAIN 	=   __MSG_ID_DEF_GLOBAL(__DOM_CONTROL,0x301);
  //!set controller parameter for error compensation
  const MsgId SET_DYN_ERR_COMP_TD		=   __MSG_ID_DEF_GLOBAL(__DOM_CONTROL,0x302);
  //!set controller parameter for error compensation
  const MsgId SET_DYN_ERR_COMP_TR		=   __MSG_ID_DEF_GLOBAL(__DOM_CONTROL,0x303);
  //!set controller parameter for error compensation
  const MsgId SET_DYN_ERR_COMP_KR		=   __MSG_ID_DEF_GLOBAL(__DOM_CONTROL,0x304);
  //!enable dynamics error compensation with moment error as input
  const MsgId SET_DYN_ERR_COMP_T		= 	__MSG_ID_DEF_GLOBAL(__DOM_CONTROL,0x305);
  //Robert ende
}
#endif//__IFACE_SIG_CONTROL_HPP__

