#ifndef LEPP3_KALMANFILTER_H
#define LEPP3_KALMANFILTER_H

#include <kalman/ExtendedKalmanFilter.hpp>
#include <kalman/SquareRootExtendedKalmanFilter.hpp>
#include <kalman/UnscentedKalmanFilter.hpp>
#include <kalman/SquareRootUnscentedKalmanFilter.hpp>

namespace lepp
{
namespace ObstacleFilter
{

/**
 * 6D state: position and velocity
 */
class State : public Kalman::Vector<float, 6>
{
public:
  KALMAN_VECTOR(State, float, 6)

  State(const Eigen::Vector3f& position, const Eigen::Vector3f& velocity)
  {
    (*this) << position, velocity;
  }

  Eigen::Vector3f position() const { return this->head<3>(); }
  Eigen::Vector3f velocity() const { return this->tail<3>(); }
};

/**
 * The system model is a simple linear random-accelerations model,
 * in which both position and velocity have uncorrelated gaussian noise.
 *
 * State transition matrix:
 * (1  0  0  dt 0  0 )
 * (0  1  0  0  dt 0 )
 * (0  0  1  0  0  dt)
 * (0  0  0  1  0  0 )
 * (0  0  0  0  1  0 )
 * (0  0  0  0  0  1 )
 */
class SystemModel : public Kalman::LinearizedSystemModel<State>
{
public:

  SystemModel(float sigmaPos, float sigmaVel, float dtSeconds)
  {
    // set system noise
    Eigen::Matrix<float, 6, 1> diag;
    diag << sigmaPos, sigmaPos, sigmaPos, sigmaVel, sigmaVel, sigmaVel;
    setCovariance((Kalman::Covariance<State>) diag.asDiagonal());
    // set dt
    _dtSeconds = dtSeconds;
  }

  virtual State f(const State& s, const Control& u) const override
  {
    return F * s;
  }

protected:

  virtual void updateJacobians(const State& x, const Control& u) override
  {
    // System model jacobian
    F.setIdentity();
    for (int i = 0; i < 3; i++)
      F(i, i + 3) = _dtSeconds;

    // System model noise jacobian
    W.setIdentity();
  }

  float _dtSeconds;
};

/**
 * We only measure position, the velocity is unknown.
 */
class Measurement : public Kalman::Vector<float, 3>
{
public:
  KALMAN_VECTOR(Measurement, float, 3)
};

class MeasurementModel : public Kalman::LinearizedMeasurementModel<State, Measurement>
{
public:
  MeasurementModel(float sigma)
  {
    // set observation noise
    setCovariance(Eigen::Matrix3f::Identity() * sigma);
  }

  virtual Measurement h(const State& x) const override
  {
    return x.head<3>();
  }
};

} // namespace ObstacleFilter

// We use the EKF but since our model is completely linear we are actually in the special case of the regular old kalman filter
using ObstacleKalmanFilter = Kalman::ExtendedKalmanFilter<ObstacleFilter::State>;

} // namespace lepp

#endif //LEPP3_KALMANFILTER_H
