//emacs -*-Mode: C++;-*-
/*!
  @file iface_ssv.hpp

sent stepseq

  Copyright (C)  Institute of Applied Mechanics, TU-Muenchen
   All rights reserved.
   

*/
#ifndef __IFACE_STEPSEQ_HPP__
#define __IFACE_STEPSEQ_HPP__
#include <stdint.h>
#include <begin_pack.h>

// #include <xdebug.c>

namespace am2b_iface
{


  struct struct_data_stepseq_ssv_log
  // : 
  //     public struct_data_ssv,
  //     public struct_data_stepseq
  {

    struct_data_stepseq_ssv_log()
    {
      stamp_gen = -1;
      //----------------------------------------------------------------------
      // struct data stepseq base

      L0 = 0;
      L1 = 0;
      B = 0;
      phiO =0;
      dz_clear =0;
      dz_step =0;
      dy =0;
      H =0;
      T =0; // step time    

      //----------------------------------------------------------------------
      // struct data stepseq

      // stance foot (?)
      start_x =0; 
      start_y =0; 
      start_z =0;
      start_phi_z =0;
      phi_leg_rel =0;
      stance =0;
      //! goal for stepplanner
      use_2D_planner = 0;
      // if 0 - no goal search but astar search calculates internally goal for heuristic
      goal_search =0;
      //! goal position in 2D 
      goal_x =0;
      goal_y =0;
      goal_phi =0;

      //----------------------------------------------------------------------
      // Optimization results
      dz_clear_ref =0;
      dz_clear_opt =0;
      dy_ref =0;
      dy_opt =0;
      H_cog_ref =0;
      H_cog_opt =0;
      V_old_opt =0;
      V_new_opt =0;

      //----------------------------------------------------------------------
      // iface ssv 
      //! id of segment
      id = -1;
      //! id of ssv object (not used : 0)
      id_ssv = -1;
      //! together with delete event: should only ssv obj be deleted or whole segment?
      remove_ssv_obj = -1;
      //! type -> 0:PSS, 1:LSS, 2:TSS
      type =0;
      //! radius
      radius =0;
      //! Which is the coresseponding surface? 
      surface =-1;
      //! vectors
      for(uint16_t ii = 0;ii<3;++ii)
        {
          p0[ii] =0;
          p1[ii] =0;
          p2[ii] =0;
        }
      //----------------------------------------------------------------------
      // iface surface 
      //! id of segment
      surface_id =-1;

      for(uint16_t ii = 0;ii<3;++ii)
        {
          surface_n[ii] =0;
          surface_p0[ii] =0;
          surface_p1[ii] =0;
          surface_p2[ii] =0;
          surface_p3[ii] =0;
          surface_p4[ii] =0;
          surface_p5[ii] =0;
          surface_p6[ii] =0;
          surface_p7[ii] =0;
        }
      
    }
    // offsetof is not able to deal with inheritance
    //
    uint64_t stamp_gen;
    //----------------------------------------------------------------------
    // struct data stepseq base

    float L0;
    float L1;
    float B;
    float phiO;
    float dz_clear;
    float dz_step;
    float dy;
    float H;
    float T; // step time    

    //----------------------------------------------------------------------
    // struct data stepseq

    // stance foot (?)
    float start_x; 
    float start_y; 
    float start_z;
    float start_phi_z;
    float phi_leg_rel;
    int32_t stance;
    //! goal for stepplanner
    // if 0 - no goal search but astar search calculates internally goal for heuristic
    int32_t goal_search;    
    // if use a 2D pre planning?
    int32_t use_2D_planner;
    //! goal position in 2D 
    float goal_x;
    float goal_y;
    float goal_phi;

    //----------------------------------------------------------------------
    // Optimization results
    float dz_clear_ref;
    float dz_clear_opt;
    float dy_ref;
    float dy_opt;
    float H_cog_ref;
    float H_cog_opt;
    float V_old_opt;
    float V_new_opt;

    //----------------------------------------------------------------------
    // iface ssv 
    //! id of segment
    int32_t id;
    //! id of ssv object (not used : 0)
    int32_t id_ssv;
    //! together with delete event: should only ssv obj be deleted or whole segment?
    int32_t remove_ssv_obj;
    //! type -> 0:PSS, 1:LSS, 2:TSS
    int32_t type;
    //! radius
    float radius;
    //! Which is the coresseponding surface? 
    int32_t surface;
    //! vectors
    float p0[3];
    float p1[3];
    float p2[3];

    //----------------------------------------------------------------------
    // iface surface 
    //! id of segment
    int32_t surface_id;
    //! n
    float surface_n[3];
    //! corner points
    float surface_p0[3];
    float surface_p1[3];
    float surface_p2[3];
    float surface_p3[3];
    float surface_p4[3];
    float surface_p5[3];
    float surface_p6[3];
    float surface_p7[3];


  };

}
#include <end_pack.h>
#endif//__IFACE_SSV_HPP__
