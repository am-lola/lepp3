#ifndef __IFACE_MODEL_LOLA_HPP__
#define __IFACE_MODEL_LOLA_HPP__
#include <stdint.h>
#include <cmath>

#include <am2b_config.h>//head lola
#include <begin_pack.h>

namespace am2b_iface
{
  namespace lola
  {
    //!interface version
    const uint32_t IFACE_MODEL_VERSION=0x0004;

#ifdef HEAD_LOLA
    //!number of joints
    //main joints + head as pan/tilt unit
    const uint32_t N_JOINTS=22+2;
    //!number of segments
    //joints + 1 -1 (-1 because convergence doesnt have a dof
    //in the mech. model)
    const uint32_t N_SEGMENTS=N_JOINTS+1;
#else
    //!number of joints
    const uint32_t N_JOINTS=22;
    //!number of segments
    const uint32_t N_SEGMENTS=N_JOINTS+1;
#endif


    /////////////////////////////////////////
    //DoFs for rigid gear model
    /////////////////////////////////////////
    //!number of mechanical dofs
#ifdef HEAD_LOLA
    const uint32_t N_MECH_DOF_W_MOTORS_RIGID=N_JOINTS+6;
    const uint32_t N_MECH_DOF_WO_MOTORS=N_JOINTS+6;
#else
    const uint32_t N_MECH_DOF_W_MOTORS_RIGID=N_JOINTS+6;
    const uint32_t N_MECH_DOF_WO_MOTORS=N_JOINTS+6;
#endif


#ifdef HEAD_LOLA
    //!number of mechanical dofs
    //2*(number of joint in mbs model) + 6 torso dofs -2*3 (screw drives) -2 (head joints)
    const uint32_t N_MECH_DOF_W_MOTORS_GEARS_ELASTIC=2*(N_JOINTS-1)+6-2*3;
#else
    //!number of mechanical dofs
    const uint32_t N_MECH_DOF_W_MOTORS_GEARS_ELASTIC=2*N_JOINTS+6-2*3;
#endif

    //!index of joint angles
    typedef enum {tbr=0,tba,
		  thrr,thar,thfr,tkfr,tsar,tsfr,tzfr,//2..8
		  thrl,thal,thfl,tkfl,tsal,tsfl,tzfl,//9..15
		  tafr,taar,tefr,//16..18
		  tafl,taal,tefl//19..21
#ifdef HEAD_LOLA
		  ,tvp, tvt //22,23,24
#endif//HEAD_LOLA
    }ThetaIndex;

    //!motor angle indices
    typedef enum {ma_br=0,ma_ba,
                  ma_hrr,ma_har,ma_hfr,ma_kfr,ma_amir,ma_amor,ma_zfr,//2..8
                  ma_hrl,ma_hal,ma_hfl,ma_kfl,ma_amil,ma_amol,ma_zfl,//9..15
                  ma_afr,ma_aar,ma_efr,//16..18
                  ma_afl,ma_aal,ma_efl,//19..21
                  ma_vp, ma_vt //22,23,24
    }MotorAngleIndex;


    const char joint_name[N_JOINTS][8]={"tbr","tba",
                                "thrr", "thar", "thfr", "tkfr", "tsar", "tsfr", "tzfr",
                                "thrl", "thal", "thfl", "tkfl", "tsal", "tsfl", "tzfl",
                                "tafr", "taar", "tefr",
                                "tafl", "taal", "tefl"
#ifdef HEAD_LOLA
                                ,"tvp", "tvt"
#endif//HEAD_LOLA
    };


    //!list of segment names (strings)
    const char segment_name[][8]={"torso","br","ba",
				  "hrr","har","hfr","kfr","sar","sfr","zfr",
				  "hrl","hal","hfl","kfl","sal","sfl","zfl",
				  "afr","aar","efr","afl","aal","efl"
#ifdef HEAD_LOLA
				  ,"vp","vt"
#endif//HEAD_LOLA
    };
    //!segment index
    typedef enum{seg_torso=0,seg_br,seg_ba,
		 seg_hrr,seg_har,seg_hfr,seg_kfr,seg_sar,seg_sfr,seg_zfr,
		 seg_hrl,seg_hal,seg_hfl,seg_kfl,seg_sal,seg_sfl,seg_zfl,
		 seg_afr,seg_aar,seg_efr,seg_afl,seg_aal,seg_efl
#ifdef HEAD_LOLA
		 ,seg_vp,seg_vt
#endif//HEAD_LOLA
    }SegmentIndex;

#undef IFACE_MODEL_LOLA_OLD_TORAD
#ifdef TORAD
#define IFACE_MODEL_LOLA_OLD_TORAD TORAD
#undef TORAD
#endif
#define TORAD(x) (M_PI/180.0*(x))

const double joint_limit_relax=TORAD(0.0);
    
//safety limit for control
const double joint_limit_safety=TORAD(5.0);

// measured values, date: 2011-07-18, favot and schwienbacher
//!maximum addmissible joint angle values (physical/sensor data)
const double theta_max_hw[N_JOINTS]={TORAD(19.4)+joint_limit_relax, TORAD(14.7)+joint_limit_relax,//pelvis
                                         TORAD(24.7)+joint_limit_relax, TORAD(40.4)+joint_limit_relax,TORAD(44.9)+joint_limit_relax,//right hip
                                         TORAD(120.0)+joint_limit_relax,//right knee
                                         TORAD(23.9)+joint_limit_relax, TORAD(43.8)+joint_limit_relax,//right ankle
                                         TORAD(14.3)+joint_limit_relax,//right toe
                                         TORAD(25.7)+joint_limit_relax, TORAD(38.1)+joint_limit_relax,TORAD(91.3)+joint_limit_relax,//left hip
                                         TORAD(4.4)+joint_limit_relax,//left knee
                                         TORAD(24.5)+joint_limit_relax, TORAD(48.0)+joint_limit_relax,//left ankle
                                         TORAD(48.8)+joint_limit_relax,//left toe
                                         // TORAD(60.0)+joint_limit_relax,TORAD(-1.0)+joint_limit_relax,TORAD(5.0)+joint_limit_relax,//right arm
                                         // TORAD(170.0)+joint_limit_relax,TORAD(1.8)+joint_limit_relax,TORAD(120.0)+joint_limit_relax//left arm
                                         TORAD(60.0)+joint_limit_relax,TORAD(1.5)+joint_limit_relax,TORAD(5.0)+joint_limit_relax,//right arm
                                         TORAD(170.0)+joint_limit_relax,TORAD(1.5)+joint_limit_relax,TORAD(120.0)+joint_limit_relax//left arm
#ifdef HEAD_LOLA
                                         ,TORAD(113.0),TORAD(11.0)//head pan/tilt/conv
#endif//HEAD_LOLA
    };
    //!minimum addmissible joint angle values  (physical/sensor data)
    const double  theta_min_hw[N_JOINTS]={TORAD(-19.4)-joint_limit_relax,TORAD(-15.1)-joint_limit_relax,//pelvis
                                          TORAD(-26.1)-joint_limit_relax,TORAD(-34.5)-joint_limit_relax,TORAD(-92.0)-joint_limit_relax,//right hip
                                          TORAD(-4.5)-joint_limit_relax,//right knee
                                          TORAD(-23.5)-joint_limit_relax,TORAD(-49.0)-joint_limit_relax,//right ankle
                                          TORAD(-49.5)-joint_limit_relax,//right toe
                                          TORAD(-24.3)-joint_limit_relax,TORAD(-31.3)-joint_limit_relax,TORAD(-44.0)-joint_limit_relax,//left hip
                                          TORAD(-119.5)-joint_limit_relax,//left knee
                                          TORAD(-23.0)-joint_limit_relax,TORAD(-44.0)-joint_limit_relax,//left ankle
                                          TORAD(-15.8)-joint_limit_relax,//left toe
                                          TORAD(-170.0)-joint_limit_relax, TORAD(-120.0)-joint_limit_relax, TORAD(-120.0)-joint_limit_relax,//right arm
                                          TORAD(-60.0)-joint_limit_relax,TORAD(-120.0)-joint_limit_relax,TORAD(-5.0)-joint_limit_relax //left arm
#ifdef HEAD_LOLA
                                          ,TORAD(-113.0),TORAD(-51.0)//head pan/tilt/conv
                                          //FIXME: adjust convergence angle limit after correct calibration
#endif//HEAD_LOLA
    };


    //theoretical values (CAD)
    //!maximum addmissible joint angle values (software/controller input)
    const double theta_max_sw[N_JOINTS]={theta_max_hw[0]-joint_limit_safety, theta_max_hw[1]-joint_limit_safety,
                                         theta_max_hw[2]-joint_limit_safety, theta_max_hw[3]-joint_limit_safety, theta_max_hw[4]-joint_limit_safety,
                                         theta_max_hw[5]-joint_limit_safety, 
                                         theta_max_hw[6]-joint_limit_safety, theta_max_hw[7]-joint_limit_safety,
                                         //0.0,//toe angle zero
                                         theta_max_hw[8]-joint_limit_safety,//normal toe angle limit
                                         theta_max_hw[9]-joint_limit_safety, theta_max_hw[10]-joint_limit_safety, theta_max_hw[11]-joint_limit_safety,
                                         theta_max_hw[12]-joint_limit_safety, 
                                         theta_max_hw[13]-joint_limit_safety, theta_max_hw[14]-joint_limit_safety,
                                         theta_max_hw[15]-joint_limit_safety,
                                         theta_max_hw[16]-joint_limit_safety, theta_max_hw[17]-TORAD(2.0), theta_max_hw[18]-joint_limit_safety,
                                         theta_max_hw[19]-joint_limit_safety, theta_max_hw[20]-TORAD(2.0), theta_max_hw[21]-joint_limit_safety
#ifdef HEAD_LOLA
                                         ,theta_max_hw[22]-joint_limit_safety, theta_max_hw[23]-joint_limit_safety
#endif//HEAD_LOLA
    };
    //!minimum addmissible joint angle values  (software/controller input)
    const double  theta_min_sw[N_JOINTS]={theta_min_hw[0]+joint_limit_safety, theta_min_hw[1]+joint_limit_safety,
                                          theta_min_hw[2]+joint_limit_safety, theta_min_hw[3]+joint_limit_safety, theta_min_hw[4]+joint_limit_safety,
                                          theta_min_hw[5]+joint_limit_safety, 
                                          theta_min_hw[6]+joint_limit_safety, theta_min_hw[7]+joint_limit_safety,
                                          theta_min_hw[8]+joint_limit_safety,
                                          theta_min_hw[9]+joint_limit_safety, theta_min_hw[10]+joint_limit_safety, theta_min_hw[11]+joint_limit_safety,
                                          theta_min_hw[12]+joint_limit_safety, 
                                          theta_min_hw[13]+joint_limit_safety, theta_min_hw[14]+joint_limit_safety,
                                          //0.0,//toe angle zero
                                          theta_min_hw[15]+joint_limit_safety,//normal toe angle limit
                                          theta_min_hw[16]+joint_limit_safety, theta_min_hw[17]+joint_limit_safety, theta_min_hw[18]+joint_limit_safety,
                                          theta_min_hw[19]+joint_limit_safety, theta_min_hw[20]+joint_limit_safety, theta_min_hw[21]+joint_limit_safety
#ifdef HEAD_LOLA
                                          ,theta_min_hw[22]+joint_limit_safety, theta_min_hw[23]+joint_limit_safety
#endif//HEAD_LOLA
    };


    //!"zero" pose
    const double  theta_zero[N_JOINTS]={0.0,0.0,//pelvis
					0.0,TORAD(10.0),0.0,//right hip
					0.0,//right knee
					0.0,0.0,//right ankle
					0.0,//right knee
					0.0,TORAD(10.0),0.0,//left hip
					0.0,//left knee
					0.0,0.0,//left ankle
					0.0,//left toe
					0.0,TORAD(-10.0),0.0,//right arm
					0.0,TORAD(-10.0),0.0//left arm
#ifdef HEAD_LOLA
					,0.0,0.0//head pan/tilt/conv
#endif//HEAD_LOLA
    };

    //current limits. preliminary values
    const double I_max[N_JOINTS]= {30.0, 30.0,
				   30.0, 30.0, 50.0, 50.0, 30.0, 30.0, 5.0,
				   30.0, 30.0, 50.0, 50.0, 30.0, 30.0, 5.0,
				   30.0, 30.0, 30.0,
				   30.0, 30.0, 30.0
#ifdef HEAD_LOLA
				   ,9.0,1.7//head pan/tilt/conv
#endif//HEAD_LOLA
    };
    //DC Voltage
    const double U_dc[N_JOINTS]= {80.0, 80.0,
                                  80.0, 80.0, 80.0, 80.0, 80.0, 80.0, 80.0,
                                  80.0, 80.0, 80.0, 80.0, 80.0, 80.0, 80.0,
                                  80.0, 80.0, 80.0,
                                  80.0, 80.0, 80.0
#ifdef HEAD_LOLA
                                  ,24,24//head pan/tilt/conv
#endif//HEAD_LOLA
    };
    //Motor Encoder Counts
static const double mot_enc_counts_[]={11520,11520,
                                       11520,11520,11520,11520,11520,11520,4000,
                                       11520,11520,11520,11520,11520,11520,4000,
                                       11520,11520,11520,
                                       11520,11520,11520
#ifdef HEAD_LOLA
                                       ,2000,2000
#endif//HEAD_LOLA
    };

    //Motor max velocity [count/s]
    static const double joint_enc_counts_[]={131071,131071,
                                             131071,131071,131071,131071,65535,65535,65535,
                                             131071,131071,131071,131071,65535,65535,65535,
                                             131071,131071,131071,
                                             131071,131071,131071
#ifdef HEAD_LOLA
                                             ,0,0
#endif//HEAD_LOLA
    };

  }//lola



#ifdef IFACE_MODEL_LOLA_OLD_TORAD
#undef  TORAD
#define TORAD IFACE_MODEL_LOLA_OLD_TORAD
#endif
}//am2b_iface

#include <end_pack.h>
#endif//__IFACE_MODEL_LOLA_HPP__
