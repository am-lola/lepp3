/*!
  @file wpatt_data_johnnie.hpp
  
  time sequence data output from walking pattern generator
  
  Copyright (C)  Institute of Applied Mechanics, TU-Muenchen
  All rights reserved.
  
  
*/
#ifndef __IFACE_WPATT_LOLA_HPP__
#define __IFACE_WPATT_LOLA_HPP__
#include <stdint.h>

#include <begin_pack.h>
namespace am2b_iface
{
  namespace lola
  {
    struct WPattGenDataLola
    {
      enum{NX=29};
      //incremented every update
      uint64_t stamp_gen;
      uint64_t stamp_plan;
      //current time
      double   time_gen;
      double   time_plan;
      //load factor
      double stance_load;
      //stance leg index
      int stance;
      //reference trajectory point
      double x_ideal[NX];
      //d(x_ideal)/dt
      double dot_x_ideal[NX];
      //d^2(x_ideal)/dt^2
      double ddot_x_ideal[NX];
      //T_cog of 3-mass model
      double T_cog_3m[2];

      //robert: step-modification variables:
      double initial_state_x[2];
      double initial_state_y[2];
      double phi_imu[4];

      //number of stabilisation directions
      enum{NSTAB=2};     
      //!swing-leg
      double dr_swing_stab[NSTAB+1];
      double dcsin_swing_stab[NSTAB];
      //!stance-leg
      double dr_stance_stab[NSTAB+1];
      double dcsin_stance_stab[NSTAB];

      double dx_step_new;
      double dy_step_new;
      double dz_step_new;
      double dphi_x_step_new;
      double dphi_y_step_new;
      double l_init[2];
      double l_coll[2];

      double dx_cog, dot_dx_cog, ddot_dx_cog;
      double dy_cog, dot_dy_cog, ddot_dy_cog;

      //Arne - compare struct_data_stepseq
      // 3 - 3 steps
      float L0[3];
      float L1[3];
      float B[3];
      float phiO[3];
      float dz_clear[3];
      float dy[3];

    };
  } //lola
}//am2b_iface
#include <end_pack.h>
#endif//__IFACE_WPATT_LOLA_HPP__
