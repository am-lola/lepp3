/*!
  @file iface_wstab_to_wpatt.hpp

  Interface for sending data from controller to pattern generator
*/
#ifndef __IFACE_WSTAB_TO_WPATT_HPP__
#define __IFACE_WSTAB_TO_WPATT_HPP__
#include <am2b_config.h>
#include <stdint.h>

#include <begin_pack.h>
namespace am2b_iface
{
  //!data structure for communicating with wpatt
  struct WStabToWPattShm
  {
    //!set to data valid/invalid
    uint32_t valid;
    //!integer time-stamp
    uint64_t stamp;
    //!time stamp
    double   time;
    //!cog position in x
    double x_cog;
    //!cog position in y
    double y_cog;
    //!cog velocity in x
    double dot_x_cog;
    //!cog velocity in y
    double dot_y_cog;
    //!cog acceleration in x
    double ddot_x_cog;
    //!cog acceleration in y
    double ddot_y_cog;
    //!current stance leg
    double stance;
    //!foot angle mod. due to force control
    double csin0_right_force;
    double csin1_right_force;
    double csin0_left_force;
    double csin1_left_force;
    double dot_csin0_right_force;
    double dot_csin1_right_force;
    double dot_csin0_left_force;
    double dot_csin1_left_force;
    double dz_right_force_control;
    double dz_left_force_control;
    //!contact torque error from inverse dynamics
    double err_Tx, err_Ty;
  };
	
#define WSTAB_TO_WPATT_SHM_PORT_NAME "/am2b_wstab_to_wpatt"
}

#include <end_pack.h>
#endif//__IFACE_WSTAB_TO_WPATT_HPP__

