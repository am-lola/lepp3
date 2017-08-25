#ifndef LEPP_OBSTACLES_SEGMENTER_GMM_GMMDATA_H
#define LEPP_OBSTACLES_SEGMENTER_GMM_GMMDATA_H

namespace lepp {
namespace GMM {

struct SegmenterParameters {
  float voxelGridResolution;

  // this much state-responsibility is needed for a point to be "hard" assigned to a state
  float hardAssignmentStateResp = 0.9f;
  // states with GMM mixing coefficients (pi) lower than this will be removed
  float statePiRemovalThreshold = 0.01f;
  // minimum number of points in a vcluster needed for a new state to be added
  int minVclusterPoints = 10;
  // prior identity covariance scale for new states
  float newStatePriorCovarSize = 0.01f;
  // mixing of prior identity covariance for new states
  float newStatePriorCovarMix = 0.5f;
  // number of frames a state has be be alive for splitting to be enabled
  int numSplitLifeTimeFrames = 20;
  // number of points a state has to have in its second largest vcluster for it to be split
  int numSplitPoints = 15;
  // number of consecutive frames a state has to have points in two different vclusters until it is split
  int numSplitFrames = 6;
  // only split when the other vcluster has less than this percentage of points assigned to other states
  float splitMaxOtherStatesPercentage = 0.2f;
  // how much of the observation covariance is taken from the previous frame
  float obsCovarRegularization = 0.95f;
  // whether to apply a kalman filter to estimate object positions & velocities
  bool enableKalmanFilter = false;
  // noise assumed for object positions
  float kalman_PositionNoise = 0.01f;
  // noise assumed for object velocities
  float kalman_VelocityNoise = 0.15f;
  // noise assumed from measurement of positions
  float kalman_MeasurementNoise = 0.1f;
};

struct State {
  State(const Eigen::Vector3f& _pos, const Eigen::Matrix3f& _obsCovar) : pos(_pos) {
    setObsCovar(_obsCovar);
  }

  Eigen::Vector3f pos; // observation mean
  Eigen::Matrix3f obsCovar; // 3x3 observation covariance (don't set directly, use setObsCovar!)

  // cached values for pdf calculation
  Eigen::Matrix3f obsCovarInv; // 3x3 inverse observation covariance
  float logpdfConstantSummand; // cached constant for MVN distributed logpdf: -log((2 * pi)^(3/2) * sqrt(det(obsCovar))
  bool validObsCovar; // false if obsCovar is not invertible

  float pi = 0.0f; // GMM cluster mixing coefficient

  int lifeTime = 0; // frame counter
  int splitCounter = 0; // number of consecutive frames the split condition was met
  int resetNonSplitCounter = 0; // number of time this state was "reset" without splitting

  // Set the observation covariance and compute cached inverse+determinant and logpdfConstantSummand
  void setObsCovar(const Eigen::Matrix3f& cov) {
    obsCovar = cov;

    double det;
    Eigen::Matrix3d inverse;
    cov.cast<double>().computeInverseAndDetWithCheck(inverse, det, validObsCovar);

    if (validObsCovar) {
      obsCovarInv = inverse.cast<float>();
      logpdfConstantSummand = static_cast<float>(-log(15.749760995 * sqrt(det))); // 15.749760995 = (2 * pi)^(3/2)
    }
  }
};

class GMMDataObserver {
public:
  /**
  * Virtual deconstructor.
  */
  virtual ~GMMDataObserver() {}

  /**
  * Update observer with new state data.
  */
  virtual void updateState(State& gmmState, size_t idx) = 0;
  virtual void deleteState(State& gmmState, size_t idx) = 0;
};


class GMMDataSubject {
public:
  /**
  * Virtual deconstructor.
  */
  virtual ~GMMDataSubject() {}

  /**
  * Attach new observer to observer list.
  */
  void attachObserver(boost::shared_ptr<GMMDataObserver> observer) {
    observers.push_back(observer);
  }

protected:
  /**
  * Notify all attached observers.
  */
  void notifyObservers_Update(State& gmmState, size_t idx) {
    for (size_t i = 0; i < observers.size(); i++) {
      observers[i]->updateState(gmmState, idx);
    }
  }
  void notifyObservers_Delete(State& gmmState, size_t idx) {
    for (size_t i = 0; i < observers.size(); i++) {
      observers[i]->deleteState(gmmState, idx);
    }
  }

private:
  // vector holding all attached observers of this subject
  std::vector<boost::shared_ptr<GMMDataObserver> > observers;
};

} // namespace GMM
} // namespace lepp

#endif // LEPP_OBSTACLES_SEGMENTER_GMM_GMMDATA_H
