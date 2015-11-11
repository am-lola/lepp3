#ifndef TRANSFORM_OBSERVER_H_
#define TRANSFORM_OBSERVER_H_

namespace {
/**
 * A struct wrapping the parameters LOLA-provided kinematics parameters that are
 * used to construct the transformation matrices between the camera frame and
 * the world coordinate system as LOLA knows it.
 */
struct LolaKinematicsParams {
  double t_wr_cl[3];
  double R_wr_cl[3][3];
  double t_stance_odo[3];
  double phi_z_odo;
  double stance;
  int frame_num;
  int stamp;
};
} // namespace

/**
 * An ABC (interface) that classes that wish to observe a pose service
 * need to implement.
 */
class TFObserver {
public:
  /**
   * Method that the observers need to implement in order to handle a new
   * incoming pose.  Each pose is represented by its an integer identifier
   * (representing the number of the frame in the sequence of the source)
   * and the LolaKinematicsParams extracted from that pose.
   */
  virtual void notifyNewTF(int idx, const LolaKinematicsParams& params) = 0;
};

#endif //TRANSFORM_OBSERVER_H_
