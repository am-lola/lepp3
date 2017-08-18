#include "GmmSegmenter.hpp"

#include <algorithm>
#include <cmath>

#include "deps/easylogging++.h"

namespace {

float log_probability_density_function(const lepp::GMM::State& state, const Eigen::Vector4f& point) {
  const Eigen::Vector3f x = point.head<3>() - state.pos;
  return state.logpdfConstantSummand - 0.5f * x.dot(state.obsCovarInv * x);
}

float probability_density_function(const lepp::GMM::State& state, const Eigen::Vector4f& point) {
  const float lp = log_probability_density_function(state, point);
  const float p = std::exp(lp);

  return std::isnan(p) ? 0.0f : p;
}

}

lepp::GmmSegmenter::GmmSegmenter(const GMM::SegmenterParameters& params) : parameters_(params),
                                                                           voxel_grid_(params.voxelGridResolution),
                                                                           initialized_(false) {
}

void lepp::GmmSegmenter::updateFrame(FrameDataPtr frameData)
{
  if (0 < frameData->cloudMinusSurfaces->size()) {
    auto& cloud = frameData->cloudMinusSurfaces;

    using namespace Eigen;

    if (!initialized_) {
      initialize(cloud.get());
      initialized_ = true;
    }

    voxel_grid_.build(*cloud.get());

    const auto map = cloud->getMatrixXfMap();
    const size_t N = cloud->size(); // number of points

    // remove invalid states first (in reverse order because removeState does swap-with-end)
    for (int k = states_.size() - 1; k >= 0; --k) {
      if (!states_[k].validObsCovar) {
        // shouldn't happen too often
        LINFO << "removing state " << k << " due to invalid (non-invertible) observation covariance\n";
        removeState(k);
      }
    }

    if (N > vcluster_point_table.size())
      vcluster_point_table.resize(N);
    if (states_.size() > state_main_vcluster.size())
      state_main_vcluster.resize(states_.size());

    // R(i,j) = "responsibility" of state j for point i
    MatrixXf R(N, states_.size());
    // C(i,j) = points of state j that are in vcluster i (assuming a hard point-state assignment of sorts)
    MatrixXi C = MatrixXi::Zero(voxel_grid_.numClusters(), states_.size());

    Matrix4Xf VCMeans = Matrix4Xf::Zero(4, voxel_grid_.numClusters()); // vcluster means
    std::vector<Matrix4f> VCSM; // vcluster scatter matrices
    VCSM.resize(voxel_grid_.numClusters(), Matrix4f::Zero());

    VectorXi VCPointCounts = VectorXi::Zero(voxel_grid_.numClusters()); // general vcluster point count

    e_step(cloud.get(), R, C, VCMeans, VCSM, VCPointCounts);

    // total responsibilities for states
    const VectorXf rks = R.colwise().sum();
    // total number of assigned points for clusters
    const VectorXi cks = C.colwise().sum();

    std::vector<GMM::State> newStates; // states that should be added
    std::vector<int> removedStates; // indices of states that should be removed
    // remove states with low mixing coefficients, indicating that they don't hold any significant probability over points
    for (size_t k = 0; k < states_.size(); k++) {
      if (rks(k) / N < parameters_.statePiRemovalThreshold) {
        // need to defer the removal if we want to avoid recomputing all of the above
        removedStates.push_back(k);
      }
    }

    // handle the addition of new states
    {
      const VectorXi vcpoints = C.rowwise().sum(); // sum of known points within the clusters

      for (int i = 0; i < vcpoints.size(); i++) {
        // got points that are already covered by a state or don't have enough points? ignore!
        if (vcpoints[i] > 0 || VCPointCounts[i] < parameters_.minVclusterPoints)
          continue;

        const Vector4f vcmean = VCMeans.col(i);

        // compute actual covariance from scatter matrix
        Matrix4f vccov = VCSM[i] / VCPointCounts[i];
        vccov -= vcmean * vcmean.transpose();

        // combine with prior covariance (I*alpha) to avoid "overfitting" to the vcluster
        const float alpha = parameters_.newStatePriorCovarMix;
        vccov = alpha * Matrix4f::Identity() * parameters_.newStatePriorCovarSize + (1.0f - alpha) * vccov;

        GMM::State s(vcmean.head<3>(), vccov.block<3, 3>(0, 0));
        s.pi = 0.05f; // TODO: this seems arbitrary

        newStates.push_back(s);
      }
    }

    m_step(cloud.get(), R, C, rks, cks, newStates, removedStates);

    // Prepare results
    std::vector<ObjectModelParams> ret;
    for (size_t i = 0; i < states_.size(); ++i) {
      ret.emplace_back(ObjectModelParams(boost::make_shared<PointCloudT>()));
    }

    for (size_t i = 0; i < N; i++) {
      for (size_t k = 0; k < states_.size(); ++k) {
        if (R(i, k) > parameters_.hardAssignmentStateResp) {
          ret[k].obstacleCloud->push_back((*cloud)[i]);
          break;
        }
      }
    }

    ret.erase(
        std::remove_if(
            std::begin(ret),
            std::end(ret),
            [](const ObjectModelParams& p) { return p.obstacleCloud->size() == 0; }),
        std::end(ret));

    // Add new  / remove old states
    // sort in descending order for this to work (removeState does swap-with-end)
    std::sort(removedStates.begin(), removedStates.end(), std::greater<size_t>());
    for (int k : removedStates) {
      removeState(k);
    }

    // add new states
    for (GMM::State& state : newStates) {
      addState(state);
    }

    for (size_t i = 0; i < states_.size(); i++)
    {
      GMM::GMMDataSubject::notifyObservers_Update(states_[i], i);
    }

    frameData->obstacleParams = ret;
  }
  notifyObservers(frameData);
}

std::vector<lepp::ObjectModelParams> lepp::GmmSegmenter::extractObstacleParams(PointCloudConstPtr cloud) {
  using namespace Eigen;

  if (!initialized_) {
    initialize(cloud.get());
    initialized_ = true;
  }

  voxel_grid_.build(*cloud.get());

  const auto map = cloud->getMatrixXfMap();
  const size_t N = cloud->size(); // number of points

  // remove invalid states first (in reverse order because removeState does swap-with-end)
  for (int k = states_.size() - 1; k >= 0; --k) {
    if (!states_[k].validObsCovar) {
      // shouldn't happen too often
      LINFO << "removing state " << k << " due to invalid (non-invertible) observation covariance\n";
      removeState(k);
    }
  }

  if (N > vcluster_point_table.size())
    vcluster_point_table.resize(N);
  if (states_.size() > state_main_vcluster.size())
    state_main_vcluster.resize(states_.size());

  // R(i,j) = "responsibility" of state j for point i
  MatrixXf R(N, states_.size());
  // C(i,j) = points of state j that are in vcluster i (assuming a hard point-state assignment of sorts)
  MatrixXi C = MatrixXi::Zero(voxel_grid_.numClusters(), states_.size());

  Matrix4Xf VCMeans = Matrix4Xf::Zero(4, voxel_grid_.numClusters()); // vcluster means
  std::vector<Matrix4f> VCSM; // vcluster scatter matrices
  VCSM.resize(voxel_grid_.numClusters(), Matrix4f::Zero());

  VectorXi VCPointCounts = VectorXi::Zero(voxel_grid_.numClusters()); // general vcluster point count

  e_step(cloud.get(), R, C, VCMeans, VCSM, VCPointCounts);

  // total responsibilities for states
  const VectorXf rks = R.colwise().sum();
  // total number of assigned points for clusters
  const VectorXi cks = C.colwise().sum();

  std::vector<GMM::State> newStates; // states that should be added
  std::vector<int> removedStates; // indices of states that should be removed
  // remove states with low mixing coefficients, indicating that they don't hold any significant probability over points
  for (size_t k = 0; k < states_.size(); k++) {
    if (rks(k) / N < parameters_.statePiRemovalThreshold) {
      // need to defer the removal if we want to avoid recomputing all of the above
      removedStates.push_back(k);
    }
  }

  // handle the addition of new states
  {
    const VectorXi vcpoints = C.rowwise().sum(); // sum of known points within the clusters

    for (int i = 0; i < vcpoints.size(); i++) {
      // got points that are already covered by a state or don't have enough points? ignore!
      if (vcpoints[i] > 0 || VCPointCounts[i] < parameters_.minVclusterPoints)
        continue;

      const Vector4f vcmean = VCMeans.col(i);

      // compute actual covariance from scatter matrix
      Matrix4f vccov = VCSM[i] / VCPointCounts[i];
      vccov -= vcmean * vcmean.transpose();

      // combine with prior covariance (I*alpha) to avoid "overfitting" to the vcluster
      const float alpha = parameters_.newStatePriorCovarMix;
      vccov = alpha * Matrix4f::Identity() * parameters_.newStatePriorCovarSize + (1.0f - alpha) * vccov;

      GMM::State s(vcmean.head<3>(), vccov.block<3, 3>(0, 0));
      s.pi = 0.05f; // TODO: this seems arbitrary

      newStates.push_back(s);
    }
  }

  m_step(cloud.get(), R, C, rks, cks, newStates, removedStates);

  // Prepare results
  std::vector<ObjectModelParams> ret;
  for (size_t i = 0; i < states_.size(); ++i) {
    ret.emplace_back(boost::make_shared<PointCloudT>());
  }

  for (size_t i = 0; i < N; i++) {
    for (size_t k = 0; k < states_.size(); ++k) {
      if (R(i, k) > parameters_.hardAssignmentStateResp) {
        ret[k].obstacleCloud->push_back((*cloud)[i]);
        break;
      }
    }
  }

  ret.erase(
      std::remove_if(
          std::begin(ret),
          std::end(ret),
          [](const ObjectModelParams& p) { return p.obstacleCloud->size() == 0; }),
      std::end(ret));

  // Add new  / remove old states
  // sort in descending order for this to work (removeState does swap-with-end)
  std::sort(removedStates.begin(), removedStates.end(), std::greater<size_t>());
  for (int k : removedStates) {
    removeState(k);
  }

  // add new states
  for (GMM::State& state : newStates) {
    addState(state);
  }

  for (size_t i = 0; i < states_.size(); i++)
  {
    GMM::GMMDataSubject::notifyObservers_Update(states_[i], i);
  }
  return ret;
}

void lepp::GmmSegmenter::initialize(const PointCloudT* pc) {
  // initialize by fitting a gaussian to the entire point cloud
  const auto map = pc->getMatrixXfMap();
  const Eigen::Vector4f mean = map.rowwise().mean();
  Eigen::Matrix4f covar = Eigen::Matrix4f::Zero();

  const size_t N = pc->size(); // number of points
  for (size_t i = 0; i < N; i++) {
    const Eigen::Vector4f x = map.col(i);
    covar += x * x.transpose();
    covar(3, 3) += 1.0f;
  }

  covar /= static_cast<float>(N);
  covar -= mean * mean.transpose();

  addState(mean.head<3>(), covar.block<3, 3>(0, 0));

  for (auto& state : states_) {
    // initialize mixing coefficients uniformly
    state.pi = 1.0f / states_.size();
  }
}

void lepp::GmmSegmenter::e_step(PointCloudT const* pc, Eigen::MatrixXf& R, Eigen::MatrixXi& C,
                                Eigen::Matrix4Xf& VCMeans, std::vector<Eigen::Matrix4f>& VCSM,
                                Eigen::VectorXi& VCPointCounts) {
  using namespace Eigen;

  const auto map = pc->getMatrixXfMap();
  const size_t N = pc->size(); // number of points

  for (size_t i = 0; i < N; i++) {
    const Vector4f x = map.col(i);

    float normalizer = 0.0f;
    for (size_t k = 0; k < states_.size(); k++) {
      normalizer += states_[k].pi * probability_density_function(states_[k], x);
    }

    const int vcluster = voxel_grid_.clusterForPoint(x);

#ifdef DEBUG
    if (vcluster < 0 || vcluster >= _voxelGrid.numClusters())
    {
      std::cout << vcluster << std::endl;
      throw std::runtime_error("invalid vcluster for point");
    }
#endif

    for (size_t k = 0; k < states_.size(); k++) {
      if (normalizer > 0.001f) {
        R(i, k) = states_[k].pi * probability_density_function(states_[k], x) / normalizer;

        if (R(i, k) > parameters_.hardAssignmentStateResp) {
          // this point likely "belongs" to state k, add contribution of state to vcluster of point
          ++C(vcluster, k);
        }
      } else {
        // this point has not enough probability support from any state - ignore it entirely
        R(i, k) = 0.0f;
      }
    }

    ++VCPointCounts(vcluster);
    // in addition, compute the vcluster mean and scatter matrix
    // (however we only need this data for adding new states right now, so small performance gains cloud be obtained by computing this lazily)
    {
      Vector4f mean = VCMeans.col(vcluster);
      const float c = mean.w() + 1.0f;
      mean = (c - 1.0f) / c * mean + x / c;
      mean.w() = c;
      VCMeans.col(vcluster) = mean;

      VCSM[vcluster] += x * x.transpose();
    }
    // cache vcluster for point
    vcluster_point_table[i] = vcluster;
  }
}

void lepp::GmmSegmenter::m_step(PointCloudT const* pc, Eigen::MatrixXf const& R, Eigen::MatrixXi const& C,
                                Eigen::VectorXf const& rks,
                                Eigen::VectorXi const& cks,
                                std::vector<GMM::State>& newStates,
                                std::vector<int>& removedStates) {

  using namespace Eigen;

  const auto map = pc->getMatrixXfMap();
  const size_t N = pc->size(); // number of points

  for (size_t k = 0; k < states_.size(); k++) {
    if (rks(k) / N < parameters_.statePiRemovalThreshold)
      continue;

    // == splitting ==
    if (C.rows() > 1) // if we have more than two vclusters
    {
      // determine the indices of the two vclusters that contain most of the states points
      VectorXi indices = VectorXi::LinSpaced(C.rows(), 0, C.rows() - 1);
      std::nth_element(indices.data(), indices.data() + 1, indices.data() + indices.size(),
                       [&](int a, int b) { return C(a, k) > C(b, k); });

      // vcluster containing the most number of points
      const int vclusterMain = indices[0];
      // vcluster containing the second most number of points
      const int vclusterOther = indices[1];
      // save the main vcluster for the state
      state_main_vcluster[k] = vclusterMain;
      // number of points in the vcluster containing the second most number of points
      const int splitPoints = C(vclusterOther, k);

      if (states_[k].lifeTime >= parameters_.numSplitLifeTimeFrames && splitPoints >= parameters_.numSplitPoints)
        states_[k].splitCounter++;
      else
        states_[k].splitCounter = 0;

      if (states_[k].splitCounter >= parameters_.numSplitFrames) {
        // fit a gaussian to each part of the vcluster the state "owns"
        Vector4f meanA, meanB;
        Matrix4f covA, covB;
        fitSplittingGaussian(pc, R, k, vclusterMain, vclusterOther, meanA, meanB, covA, covB);

        // percentage of points in vclusterOther assigned to other states
        const float ot = (C.row(vclusterOther).sum() - splitPoints) / float(cks.sum() - cks(k));
        if (std::isnan(ot) || ot < parameters_.splitMaxOtherStatesPercentage || states_[k].resetNonSplitCounter > 1) {
          // split state
          GMM::State a(meanA.head<3>(), covA.block<3, 3>(0, 0));
          GMM::State b(meanB.head<3>(), covB.block<3, 3>(0, 0));

          a.pi = states_[k].pi / 2.0f;
          b.pi = states_[k].pi / 2.0f;

          removedStates.push_back(k);

          newStates.push_back(a);
          newStates.push_back(b);
        } else {
          // avoid splitting if vclusterOther is mostly covered by other states
          // instead, reset the state by fitting to it's main vcluster
          states_[k].pos = meanA.head<3>();
          states_[k].setObsCovar(covA.block<3, 3>(0, 0));
          states_[k].lifeTime = 0;
          states_[k].splitCounter = 0;
          states_[k].resetNonSplitCounter++;
        }
      }
    } else {
      state_main_vcluster[k] = 0;
    }
    // == end splitting ==

    // actual m-step
    Vector4f mean = Vector4f::Zero();
    Matrix4f cov = Matrix4f::Zero();
    for (size_t i = 0; i < N; i++) {
      const Vector4f x = map.col(i);
      mean += R(i, k) * x;
      cov += R(i, k) * x * x.transpose();
    }

    mean /= rks(k);
    cov /= rks(k);

    states_[k].pos = mean.head<3>();

    // really important: regularize with previous covariance as prior (corresponding to a linear interpolation)
    // note that we only use a fixed interpolation alpha which seems to work well enough
    const float alpha = parameters_.obsCovarRegularization;
    Matrix3f obsCovar = alpha * states_[k].obsCovar + (1 - alpha) * (cov - mean * mean.transpose()).block<3, 3>(0, 0);
    states_[k].setObsCovar(obsCovar);

    states_[k].pi = rks(k) / N;
    states_[k].lifeTime += 1;
  }
}

void lepp::GmmSegmenter::fitSplittingGaussian(PointCloudT const* pc, Eigen::MatrixXf const& R, size_t state,
                                              int vclusterA, int vclusterB, Eigen::Vector4f& outMeanA,
                                              Eigen::Vector4f& outMeanB,
                                              Eigen::Matrix4f& outCovA, Eigen::Matrix4f& outCovB) const {
  outCovA.setZero();
  outCovB.setZero();
  outMeanA.setZero();
  outMeanB.setZero();

  const auto map = pc->getMatrixXfMap();
  const size_t N = pc->size(); // number of points
  float numA = 0.0f;
  float numB = 0.0f;

  for (size_t i = 0; i < N; i++) {
    const Eigen::Vector4f x = map.col(i);

    if (R(i, state) > parameters_.hardAssignmentStateResp) {
      const int vcluster = voxel_grid_.clusterForPoint(x);
      if (vcluster == vclusterA) {
        outMeanA += x;
        outCovA += x * x.transpose();
        numA += 1.0f;
      } else {
        outMeanB += x;
        outCovB += x * x.transpose();
        numB += 1.0f;
      }
    }
  }

  outMeanA /= numA;
  outMeanB /= numB;
  outCovA /= numA;
  outCovB /= numB;
  outCovA -= outMeanA * outMeanA.transpose();
  outCovB -= outMeanB * outMeanB.transpose();
}

void lepp::GmmSegmenter::addState(GMM::State& state) {
  states_.push_back(state);
}

void lepp::GmmSegmenter::addState(const Eigen::Vector3f& mean, const Eigen::Matrix3f& covar) {
  states_.emplace_back(mean, covar);
}

void lepp::GmmSegmenter::removeState(size_t index) {
  GMM::GMMDataSubject::notifyObservers_Delete(states_[index], index);

  std::swap(states_[index], states_.back());
  states_.pop_back();
}
