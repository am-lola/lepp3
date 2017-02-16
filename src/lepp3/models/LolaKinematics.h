#ifndef LEPP3_MODELS_LOLA_KINEMATICS_H__
#define LEPP3_MODELS_LOLA_KINEMATICS_H__

namespace lepp {

struct LolaKinematicsParams {
  double t_wr_cl[3];
  double R_wr_cl[3][3];
  double t_stance_odo[3];
  double phi_z_odo;
  double stance;
  int frame_num;
  int stamp;
};

}

#endif
