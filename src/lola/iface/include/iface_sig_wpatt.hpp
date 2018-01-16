//emacs -*-Mode: C++;-*-
/*!
  @file iface_sig_wpatt.hpp

  signals for walking pattern generator

  Copyright (C)  Institute of Applied Mechanics, TU-Muenchen
  All rights reserved.
  

*/
#ifndef __IFACE_SIG_WPATT_HPP__
#define __IFACE_SIG_WPATT_HPP__
#include <stdint.h>
#include <iface_sig.hpp>
namespace am2b_iface
{
  /***********************************************************************
   *    PARAMETERS FOR WALKING PATTERN GENERATOR
   ***********************************************************************/
  
  //! Signal Start walking
  const MsgId SIG_TRAJ_WALK                 =  __MSG_ID_DEF_GLOBAL(__DOM_WPATT,0x0);

  //! Step size in x-direction [m]
  const MsgId SET_TRAJ_LX_DES	            =  __MSG_ID_DEF_GLOBAL(__DOM_WPATT,0x1);
  
  //! Step size in y-direction [m]
  const MsgId SET_TRAJ_LY_DES	            =  __MSG_ID_DEF_GLOBAL(__DOM_WPATT,0x2);
  
  //! FIXME: ?
  const MsgId SET_TRAJ_B_DES	            =  __MSG_ID_DEF_GLOBAL(__DOM_WPATT,0x3);
  
  //! Desired CoG height [m]
  const MsgId SET_TRAJ_H_DES	            =  __MSG_ID_DEF_GLOBAL(__DOM_WPATT,0x4);

  //! Desired step time [s]
  const MsgId SET_TRAJ_T_DES               =   __MSG_ID_DEF_GLOBAL(__DOM_WPATT,0x5);
  
  //! Percentage double-support phase [%]
  const MsgId SET_TRAJ_PHASE_DS            =   __MSG_ID_DEF_GLOBAL(__DOM_WPATT,0x6);
  
  //! FIXME: ?
  const MsgId SET_TRAJ_PHASE_MSW           =   __MSG_ID_DEF_GLOBAL(__DOM_WPATT,0x7);
  
  //! Foot height during step (clearance) [m]
  const MsgId SET_TRAJ_DZ_CLEAR            =   __MSG_ID_DEF_GLOBAL(__DOM_WPATT,0x8);

  //! FIXME: UNUSED?
  const MsgId SET_TRAJ_ALPHAO_DES          =   __MSG_ID_DEF_GLOBAL(__DOM_WPATT,0x9);
  
  //! FIXME: UNUSER?
  const MsgId SET_TRAJ_BETAO_DES           =   __MSG_ID_DEF_GLOBAL(__DOM_WPATT,0xA);
  
  //! Curve-angle for curve walking [rad]
  const MsgId SET_TRAJ_GAMMAO_DES          =   __MSG_ID_DEF_GLOBAL(__DOM_WPATT,0xB);
  
  //! FIXME: NOt sure if this is right:
  /* CONT: controlled CoP to support polygon margins 
     PLAN: planned CoP to support polygon margins
  */
  const MsgId SET_TRAJ_DCOP_X_CONT          =   __MSG_ID_DEF_GLOBAL(__DOM_WPATT,0xC);
  const MsgId SET_TRAJ_DCOP_Y_CONT          =   __MSG_ID_DEF_GLOBAL(__DOM_WPATT,0xD);
  const MsgId SET_TRAJ_DCOP_X_PLAN          =   __MSG_ID_DEF_GLOBAL(__DOM_WPATT,0xE);
  const MsgId SET_TRAJ_DCOP_Y_PLAN          =   __MSG_ID_DEF_GLOBAL(__DOM_WPATT,0xF);
  
  //! Relation factor between foot and upper body mass (for step planner)
  const MsgId SET_SPKT_MLM                  =   __MSG_ID_DEF_GLOBAL(__DOM_WPATT,0x10);
  const MsgId SET_TRAJ_CSIN_ABS             =   __MSG_ID_DEF_GLOBAL(__DOM_WPATT,0x11);
  const MsgId SET_TRAJ_CSIN_REL		    =   __MSG_ID_DEF_GLOBAL(__DOM_WPATT,0x12);
  const MsgId SET_TRAJ_CSIN_MIN_REL	    =   __MSG_ID_DEF_GLOBAL(__DOM_WPATT,0x13);

  //!set x/y swing leg lead 
  const MsgId SET_TRAJ_XY_SWING_LEAD        =   __MSG_ID_DEF_GLOBAL(__DOM_WPATT,0x17);
  const MsgId SET_TRAJ_XY_SWING_LAG         =   __MSG_ID_DEF_GLOBAL(__DOM_WPATT,0x18);
  //!set csin swing leg lag
  const MsgId SET_TRAJ_CS_SWING_LAG         =   __MSG_ID_DEF_GLOBAL(__DOM_WPATT,0x19);
  

  //! Desired velocity x-direction, EventFlt
  const MsgId SET_TRAJ_VX                   =   __MSG_ID_DEF_GLOBAL(__DOM_WPATT,0x20);
  
  //! Desired velocity y-direction, EventFlt
  const MsgId SET_TRAJ_VY                   =   __MSG_ID_DEF_GLOBAL(__DOM_WPATT,0x21);
  
  //! FIXME: Desired turn rate?? EventFlt
  const MsgId SET_TRAJ_VG                   =   __MSG_ID_DEF_GLOBAL(__DOM_WPATT,0x22);
  
  //! FIXME: collection of VX, VY, VG?
  //EventVecFlt3
  const MsgId SET_TRAJ_V                    =   __MSG_ID_DEF_GLOBAL(__DOM_WPATT,0x23);

  //! signal to bow
  const MsgId SIG_TRAJ_BOW                  =   __MSG_ID_DEF_GLOBAL(__DOM_WPATT,0x24);
  
  //! frontal bow angle
  const MsgId SET_TRAJ_BOW_ANGLE0           =   __MSG_ID_DEF_GLOBAL(__DOM_WPATT,0x25);
  
  //! side bow angle
  const MsgId SET_TRAJ_BOW_ANGLE1           =   __MSG_ID_DEF_GLOBAL(__DOM_WPATT,0x26);


  //! Start to wave
  const MsgId SIG_TRAJ_WAVE                  =   __MSG_ID_DEF_GLOBAL(__DOM_WPATT,0x30);


  //! Stiffness of the ground in z-direction [N/m]
  const MsgId SET_TRAJ_COMP_CZ              =   __MSG_ID_DEF_GLOBAL(__DOM_WPATT, 0x40);

  //!walking pattern generator fsm state change
  const MsgId SIG_WPATT_FSM_STATE_CHANGED   =   __MSG_ID_DEF_GLOBAL(__DOM_WPATT, 0x50);
  //!enable / disable view target tracking in drift space
  const MsgId SIG_TRAJ_VIEW_TARGET_TRACKING =   __MSG_ID_DEF_GLOBAL(__DOM_WPATT, 0x51);
  //!set viewing target (EventVecFlt3)
  const MsgId SET_TRAJ_VIEW_TARGET           =   __MSG_ID_DEF_GLOBAL(__DOM_WPATT, 0x52);
  //!set x component of viewing target (EventFlt)
  const MsgId SET_TRAJ_VIEW_TARGET_X         =   __MSG_ID_DEF_GLOBAL(__DOM_WPATT, 0x53);
  //!set y component of viewing target (EventFlt)
  const MsgId SET_TRAJ_VIEW_TARGET_Y         =   __MSG_ID_DEF_GLOBAL(__DOM_WPATT, 0x54);
  //!set z component of viewing target (EventFlt)
  const MsgId SET_TRAJ_VIEW_TARGET_Z         =   __MSG_ID_DEF_GLOBAL(__DOM_WPATT, 0x55);
  //!set head pan angle (if view_target_tracking is disabled)
  const MsgId SET_TRAJ_VIEW_PAN              =   __MSG_ID_DEF_GLOBAL(__DOM_WPATT, 0x56);
  //!set head tilt angle (if view_target_tracking is disabled)
  const MsgId SET_TRAJ_VIEW_TILT             =   __MSG_ID_DEF_GLOBAL(__DOM_WPATT, 0x57);
  // //!set head convergence angle (if view_target_tracking is disabled)
  // const MsgId SET_TRAJ_VIEW_CONV             =   __MSG_ID_DEF_GLOBAL(__DOM_WPATT, 0x58);
  //!head trajectory duration
  const MsgId SET_TRAJ_VIEW_VMAX             =   __MSG_ID_DEF_GLOBAL(__DOM_WPATT, 0x59);
  //!head trajectory tracking low pass filter
  const MsgId SET_TRAJ_VIEW_TLP              =   __MSG_ID_DEF_GLOBAL(__DOM_WPATT, 0x60);
  //!arm trajectory x offset (EventFlt)
  const MsgId SET_TRAJ_ARM_X_OFFSET          =   __MSG_ID_DEF_GLOBAL(__DOM_WPATT, 0x70);
  //!arm trajectory y offset (EventFlt)
  const MsgId SET_TRAJ_ARM_Y_OFFSET          =   __MSG_ID_DEF_GLOBAL(__DOM_WPATT, 0x71);
  //!arm trajectory z offset (EventFlt)
  const MsgId SET_TRAJ_ARM_Z_OFFSET          =   __MSG_ID_DEF_GLOBAL(__DOM_WPATT, 0x72);
  //!arm trajectory scale factor (relative to leg trajectory)
  const MsgId SET_TRAJ_ARM_FACTOR            =   __MSG_ID_DEF_GLOBAL(__DOM_WPATT, 0x73);

  //!enable phase reset (transition from swing ==> stance) (EventInt)
  const MsgId SET_TRAJ_PHASE_RESET           =  __MSG_ID_DEF_GLOBAL(__DOM_WPATT, 0x80);
  //!enable replan event
  const MsgId SET_TRAJ_REPLAN                =  __MSG_ID_DEF_GLOBAL(__DOM_WPATT, 0x81);

  const MsgId SIG_TRAJ_DISTURBED             =  __MSG_ID_DEF_GLOBAL(__DOM_WPATT, 0x82);
  //!enable foot step modifications
  const MsgId SET_STEP_MOD                   =  __MSG_ID_DEF_GLOBAL(__DOM_WPATT, 0x83);
  //!enable cog modifications
  const MsgId SET_COG_MOD                    =  __MSG_ID_DEF_GLOBAL(__DOM_WPATT, 0x84);
  const MsgId SET_STEP_MOD_3D                =  __MSG_ID_DEF_GLOBAL(__DOM_WPATT, 0x85);

  //!step modification configuration
  const MsgId SET_STEP_ITER_X                =  __MSG_ID_DEF_GLOBAL(__DOM_WPATT, 0x86);
  const MsgId SET_STEP_ITER_Y                =  __MSG_ID_DEF_GLOBAL(__DOM_WPATT, 0x87);
  const MsgId SET_STEP_INIT_KX               =  __MSG_ID_DEF_GLOBAL(__DOM_WPATT, 0x88);
  const MsgId SET_STEP_INIT_KY               =  __MSG_ID_DEF_GLOBAL(__DOM_WPATT, 0x89);
  const MsgId SET_STEP_UPD_KX                =  __MSG_ID_DEF_GLOBAL(__DOM_WPATT, 0x8A);
  const MsgId SET_STEP_UPD_KY                =  __MSG_ID_DEF_GLOBAL(__DOM_WPATT, 0x8B);
  //!cog modification configuration
  const MsgId SET_COG_XQX                    =  __MSG_ID_DEF_GLOBAL(__DOM_WPATT, 0x8C);
  const MsgId SET_COG_XSX                    =  __MSG_ID_DEF_GLOBAL(__DOM_WPATT, 0x8D);
  const MsgId SET_COG_YQX                    =  __MSG_ID_DEF_GLOBAL(__DOM_WPATT, 0x8E);
  const MsgId SET_COG_YSX                    =  __MSG_ID_DEF_GLOBAL(__DOM_WPATT, 0x8F);

  //!enable collision avoidance
  const MsgId SET_COLL_AV                    =  __MSG_ID_DEF_GLOBAL(__DOM_WPATT, 0xB0);
  //!enable trajectory optimization
  const MsgId SET_TRAJ_OPT                   =  __MSG_ID_DEF_GLOBAL(__DOM_WPATT, 0xB1);
  //!enable cog z optimization
  const MsgId SET_COG_Z_OPT                   =  __MSG_ID_DEF_GLOBAL(__DOM_WPATT, 0xB2);


  const MsgId SET_TRAJ_KNEE_STRETCH_KP       =  __MSG_ID_DEF_GLOBAL(__DOM_WPATT, 0xF0);
  const MsgId SET_TRAJ_KNEE_STRETCH_MAX      =  __MSG_ID_DEF_GLOBAL(__DOM_WPATT, 0xF1);
  const MsgId SET_TRAJ_KNEE_STRETCH_MIN      =  __MSG_ID_DEF_GLOBAL(__DOM_WPATT, 0xF2);






  //!set current "skill" to normal walking
  const MsgId SIG_TRAJ_SKILL_NORMAL          =  __MSG_ID_DEF_GLOBAL(__DOM_WPATT,0x100);
  //!set current "skill" to walking up / a platform (EventFlt: with flt= platform height)
  const MsgId SIG_TRAJ_SKILL_PLATFORM        =  __MSG_ID_DEF_GLOBAL(__DOM_WPATT,0x101);
  //!set current "skill" to stepping up on a standart platform
  const MsgId SIG_TRAJ_SKILL_PLATFORM_UP     =  __MSG_ID_DEF_GLOBAL(__DOM_WPATT,0x102);
  //!set current "skill" to stepping down from on a standart platform
  const MsgId SIG_TRAJ_SKILL_PLATFORM_DOWN   =  __MSG_ID_DEF_GLOBAL(__DOM_WPATT,0x103);
  //!set current "skill" to fixed step sequence (Event<SkillWalkParam>)
  const MsgId SIG_TRAJ_SKILL_WALK_SEQ        =  __MSG_ID_DEF_GLOBAL(__DOM_WPATT,0x104);
  //!stop walking sequence: stop execution of walking sequence
  const MsgId SIG_TRAJ_SKILL_WALK_SEQ_ABORT  =  __MSG_ID_DEF_GLOBAL(__DOM_WPATT,0x105);


  /**************************************************
            current odometry data / etc
  **************************************************/
  //!estimated current foot position (odomentry, EventVecFlt3)
  const MsgId SIG_ODO_R_STANCE_ODO          =  __MSG_ID_DEF_GLOBAL(__DOM_WPATT,0x201);
  //!estimated current foot rotation (odomentry, EventFlt)
  const MsgId SIG_ODO_PHI_STANCE_ODO        =  __MSG_ID_DEF_GLOBAL(__DOM_WPATT,0x202);

  /**************************************************
           obstacle avoidance
  **************************************************/
  //! data for ssv construction ("odomentry", EventFlt15)
  const MsgId SET_SSV                       =  __MSG_ID_DEF_GLOBAL(__DOM_WPATT,0x203);
  //!set current "skill" to obstacle avoidance
  const MsgId SET_SSV_SEQ                   =  __MSG_ID_DEF_GLOBAL(__DOM_WPATT,0x204);
  //!reset r_odo
  const MsgId SET_RESET_R_ODO               =  __MSG_ID_DEF_GLOBAL(__DOM_WPATT,0x205);
  //! modify data of ssv  ("odomentry", EventFlt15)
  const MsgId MODIFY_SSV                    =  __MSG_ID_DEF_GLOBAL(__DOM_WPATT,0x206);
  //! remove ssv segment  
  const MsgId REMOVE_SSV_WHOLE_SEGMENT      =  __MSG_ID_DEF_GLOBAL(__DOM_WPATT,0x207);
  //! remove data of all ssv 
  const MsgId RESET_SSVMAP                  =  __MSG_ID_DEF_GLOBAL(__DOM_WPATT,0x208);
  //! remove ssv part
  const MsgId REMOVE_SSV_ONLY_PART          =  __MSG_ID_DEF_GLOBAL(__DOM_WPATT,0x209);
  /**************************************************
           surfaces
  **************************************************/
  //! data for surface construction - lepp2 can not send surface data
  //! so it makes no sense to subscribe to old vision msg for surface messages
  const MsgId SET_SURFACE                    =  __MSG_ID_DEF_GLOBAL(__DOM_WPATT,0x301);
  //! modify surface 
  const MsgId MODIFY_SURFACE                 =  __MSG_ID_DEF_GLOBAL(__DOM_WPATT,0x302);
  //! remove surface  
  const MsgId REMOVE_SURFACE                 =  __MSG_ID_DEF_GLOBAL(__DOM_WPATT,0x303);
  //! remove all surfaces 
  const MsgId RESET_SURFACEMAP               =  __MSG_ID_DEF_GLOBAL(__DOM_WPATT,0x304);

  /**************************************************
           Astar Search
  **************************************************/
  //! should A*-search search for a goal psoition 
  const MsgId SET_GOAL_SEARCH                       =  __MSG_ID_DEF_GLOBAL(__DOM_WPATT,0x401);
  //! Goal position x (in odo frame)
  const MsgId SET_GOAL_X                       =  __MSG_ID_DEF_GLOBAL(__DOM_WPATT,0x402);
  //! Goal position y (in odo frame)
  const MsgId SET_GOAL_Y                       =  __MSG_ID_DEF_GLOBAL(__DOM_WPATT,0x403);
  //! Goal position y (in odo frame)
  const MsgId SET_GOAL_PHI                       =  __MSG_ID_DEF_GLOBAL(__DOM_WPATT,0x404);
  //! should A*-search use a 2D path pre-planning? 
  const MsgId SET_2D_PLANNER                       =  __MSG_ID_DEF_GLOBAL(__DOM_WPATT,0x405);

  //! results of parameter optimization to ar
  const MsgId STEPSEQ_AR_VIZUALIZATION                =  __MSG_ID_DEF_GLOBAL(__DOM_WPATT,0x501);

  //! event for new communication system with lepp3
  const MsgId VISION_MESSAGE                =  __MSG_ID_DEF_GLOBAL(__DOM_WPATT,0x502);



}
#endif//__IFACE_SIG_WPATT_HPP__

