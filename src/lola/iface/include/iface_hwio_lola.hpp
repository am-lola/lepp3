/*!
  @file iface_hwio_lola.hpp

  Interface definition for johnnie's hardware input/output
*/
#ifndef __IFACE_HWIO_LOLA_HPP__
#define __IFACE_HWIO_LOLA_HPP__
#include <am2b_config.h>
#include <stdint.h>
#include <event.hpp>
#include <iface_model_lola.hpp>
#include <iface_ec_lola.hpp>

#include <begin_pack.h>



namespace am2b_iface
{
  namespace lola
  {
    //for compatibility
    enum{
      HW_IDLE=0,
      HW_BOOT=1,
      HW_MFREE=2,
      HW_TCONT=3,
      HW_VCONT=4,
      HW_PCONT_DSP=5,
      HW_PCONT_ELMO=6,
      HW_PCONT_PC=7,
      HW_FAULT=8,
      HW_NONE=9,
      HW_UNKNOWN=10
    };

    /**************************************************
                  data structures
    **************************************************/
    //!Interface version
    const uint32_t IFACE_HWIO_LOLA_VERSION=0x0005;

    //!lolas's sensor data
    struct LolaSensorDataShm
    {
      //enum provides access to array dimensions for templates etc.
      enum{N_THETA=N_JOINTS};
      //!set to data valid/invalid
      uint32_t valid;
      //!integer time-stamp
      uint64_t stamp;
      //!time stamp
      double   time;
      //HW_IDLE, ...
      int32_t  hw_mode;
      //ethercat state
      uint16_t ec_state;
      //!force sensor data : forces
      double F[2][3];
      //!force sensor data : torques
      double T[2][3];
      //!filtered contact forces (currently PT1 filter with T=0.02)
      double Fc_filt[2];
      //!contact state
      uint16_t cont_state[2];
      //!knee joint force/torqu sensors :
      //right [force,moment]
      //left  [force,moment]
      double knee_FTS[2][2];
      //!knee joint force sensor AD values
      uint16_t knee_FTS_ADC[2][2];

      //!IMU angles
      double phi[3];
      //!d(phi)/dt
      double dot_phi[3];
      //!angular velocity in imu frame
      double imu_omgs[3];
      //!r/p/y angles as output from the IMU
      double imu_rpy[3];
      //!imu acceleration (in sensor frame)
      double imu_accs[3];
      //!imu status word
      uint16_t imu_status;


      //!elmo state
      ElmoState elmo_group_state;
      ElmoState elmo_state[N_JOINTS];
      //!joint angles
      double theta[N_JOINTS];
      //!joint speed -- Computed on Elmo
      double dot_theta[N_JOINTS];
      //!joint angles measured by link side sensors
      double theta_endat[N_JOINTS];
      //!joint speed (link side sensors)
      double dot_theta_endat[N_JOINTS];
      //!raw endat data (received from DSPs)
      int16_t endat_raw[N_JOINTS];

      //!measured current
      double i_meas[N_JOINTS];
      //! motor encoder values
      int32_t menc_raw[N_JOINTS];
      //! motor encoder values
      int32_t dot_menc_raw[N_JOINTS];

      //! measured dc link voltage
      double dc_link_voltage[N_JOINTS];

      // motor angle
      double m_angle[N_JOINTS];
      double dot_m_angle[N_JOINTS];

      // external analog/digital measurements 
      float adc[4];
      bool di[2];
    };



    /*!
      input data to low-level controller<\br>
      depending on the operating mode,
      either position, velocity, position+velocity
      or current inputs may be valid/used
    */
    struct LolaInputDataShm
    {
      //enum provides access to array dimensions for templates etc.
      enum{N_THETA=N_JOINTS};
      //!set to data valid/invalid
      uint32_t valid;
      //!integer time stamp
      uint64_t stamp;
      //!time
      double   time;
      //!target control mode (for safety purposes) (HW_PCONT, ...)
      int32_t  hw_mode;
      //!desired joint angles
      double   theta[N_JOINTS];
      //!desired joint velocities
      double   dot_theta[N_JOINTS];
      //!desired joint torque
      double   tau[N_JOINTS];
      //!desired motor current
      double   I_des[N_JOINTS];
      //!stance
      int32_t  stance;
    };


    
    //!data for log-buffer
    struct HWIOLola
    {
      enum{N_DEBUG=10};
      LolaSensorDataShm hwo;

      LolaInputDataShm hwi;
      // //!motor encoder position target values sent to dsp
      // int32_t menc_raw_d[N_JOINTS];
      // //!motor encoder speed target values sent to dsp
      // int32_t dot_menc_raw_d[N_JOINTS];
      // //int32_t hwl_state;//devr-hwl fsm state
      // //idbg/ddbg provide "debug channels" for integer/floating point data
      // int32_t idbg[N_DEBUG];
      // double  ddbg[N_DEBUG];
    };


    struct Int32Double
    {
      int32_t idata;
      double  ddata;
    };

    const unsigned int MOTENC_TICKS     = 4*2880;
    const unsigned int MOTENC_TICKS_TOE = 4*1000;
    const unsigned int ENDAT_TICKS      = 0xFFFF;

    
    //!Information about devr-hwl process
    struct HWLInfo
    {
      //one of HWLCtrlMode
      uint32_t ctrl_mode;
      //interface versions (this file)
      uint32_t iface_version_hwio_lola;
      //sercos cycle time in usec
      uint32_t sampling_time;
    };


    //union of all possible input and output data types
    typedef union
    {
      int32_t                          i32;
      uint32_t                         u32;
      double                           dbl;
      am2b_iface::lola::HWLInfo        hwl_info;
      am2b::EventIntFlt::Data          elmo_dat;
    }IO_DEVCTL_DATA;

  }//lola
}//am2b_iface
#include <end_pack.h>
#endif//__IFACE_HWIO_LOLA_HPP__
