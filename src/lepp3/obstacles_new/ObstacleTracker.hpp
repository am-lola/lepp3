#ifndef LEPP3_OBSTACLETRACKER_H
#define LEPP3_OBSTACLETRACKER_H

#include "lepp3/Typedefs.hpp"
#include "lepp3/Utils.hpp"
#include "lepp3/FrameData.hpp"
#include "lepp3/obstacles_new/ObstacleTrackerState.hpp"
#include "lepp3/obstacles_new/VoxelGrid.h"
#include "lepp3/obstacles_new/ObstacleTrackerVisualizer.hpp"

#include "lepp3/util/PointCloudRecorder.hpp"
// #include "lepp3/util/Timer.hpp"
#include "lepp3/debug/timer.hpp"

#include <algorithm>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>

// #define ENABLE_RECORDER
#ifdef ENABLE_RECORDER
bool g_enableObstacleTrackerRecorder = true;
#else
bool g_enableObstacleTrackerRecorder = false;
#endif

/* TODO
 * - Due to sensor noise, parts of the point cloud may flicker, i.e. disappear in some frames.
 *   Right now the obstacle will be removed instantly and added next frame. Instead, keep the obstacle
 *   around for a while before removing it to increase robustness.
 * - Ensure mixing coefficients add to one when adding/splitting/removing (not important - still works without)
 * - maybe choose a better filter for the SSV radius
 * - ssv/ellipsoids alpha (+color) option
 * - when all the points in a vcluster are not hard assigned to a state, because two or more states exist around the vcluster,
 *   a new state will be created each frame, making the problem even worse
 * - capsule filtering
 * - things to do when obstacles get too big:
 * -- force a capsule, will fit much better
 * -- split the obstacle (although good convergence is not guaranteed, so probably a bad idea)
 * -- invoke some of the old splitting code
 * - !!! clean up SSV hysteresis, expose to config
 */

namespace lepp
{

// === constants ===

/*
 * what resolution pcl::VoxelGrid should downsample the point cloud to
 * be aware that when you change this, other parameters depending on number of points
 * may need to be re-adjusted
*/
constexpr static float DOWNSAMPLE_RESOLUTION = 0.03f;
/*
 * CDF of 3-DoF chi-square distribution up to 95% probability (corresponding to 95% probability mass/confidence interval of an MVN)
 * the square root is needed for visualization and maybe SSV generation
 */
constexpr static float SQRT_CHI_SQUARE = 2.7955f;
// this much state-responsibility is needed for a point to be "hard" assigned to a state
constexpr static float HARD_ASSIGNMENT_STATE_RESP = 0.9f;
// states with GMM mixing coefficients (pi) lower than this will be removed
constexpr static float STATE_PI_REMOVAL_THRESHOLD = 0.01f;
// minimum number of points in a vcluster needed for a new state to be added
constexpr static int MIN_VCLUSTER_POINTS = 10;
// prior identity covariance scale for new states
constexpr static float NEW_STATE_PRIOR_COVAR_SIZE = 0.01f;
// mixing of prior identity covariance for new states
constexpr static float NEW_STATE_PRIOR_COVAR_MIX = 0.5f;
// number of frames a state has be be alive for splitting to be enabled
constexpr static int NUM_SPLIT_LIFE_TIME_FRAMES = 20;
// number of points a state has to have in its second largest vcluster for it to be split
constexpr static int NUM_SPLIT_POINTS = 15;
// number of consecutive frames a state has to have points in two different vclusters until it is split
constexpr static int NUM_SPLIT_FRAMES = 6;
// only split when the other vcluster has less than this percentage of points assigned to other states
constexpr static float SPLIT_MAX_OTHER_STATES_PERCENTAGE = 0.2f;
// how much of the observation covariance is taken from the previous frame
constexpr static float OBS_COVAR_REGULARIZATION = 0.95f;
// SSV radius exponential filter alpha
constexpr static float SSV_EXPFILTER_ALPHA = 0.9f;
// =================

/**
 * Main obstacle tracking class, takes a point cloud and outputs obstacles.
 * NOTE: We often use Vector4f for improved SIMD performance (also note pcl delivers 16-byte aligned point clouds for the same reason)
 * NOTE: Terminology: vcluster = voxel grid cluster! Don't confuse this with a state, which is technically also
 * a cluster, but a probabalistic one (in the gaussian mixture model).
 */
class ObstacleTracker : public FrameDataObserver, public FrameDataSubject
{
public:

  ObstacleTracker(const ObstacleTrackerParameters& parameters)
    : _voxelGrid(parameters.voxelGridResolution)
  {
    _parameters = parameters;
    if (parameters.enableVisualizer)
      _visualizer = new ObstacleTrackerVisualizer(parameters);

#ifdef ENABLE_RECORDER
    if (hasVisualizer())
    {
      _recorder = new PointCloudRecorder(_visualizer->getVisualizer());
      _recorder->attachObserver(boost::shared_ptr<ObstacleTracker>(this));
    }
#endif
  }

  ~ObstacleTracker()
  {
    if (_recorder != nullptr)
      delete _recorder;
    if (_visualizer != nullptr)
      delete _visualizer;
  }

  virtual void updateFrame(FrameDataPtr frameData) override;

private:
  ObstacleTrackerParameters _parameters;

  void initialize(const PointCloudT* pc);
  void emstep(const PointCloudT* pc, int frameNum);
  // fit two gaussians to both parts of the vclusters that are assigned to a state
  void fitSplittingGaussian(const PointCloudT* pc, const Eigen::MatrixXf& R, int state,
                            int vclusterA, int vclusterB, Eigen::Vector4f& outMeanA, Eigen::Vector4f& outMeanB,
                            Eigen::Matrix4f& outCovA, Eigen::Matrix4f& outCovB) const;

  void fitSSVs(int k, const PointCloudT* pc, const Eigen::MatrixXf& R, const Eigen::MatrixXi& C);

  void generateSSVs(State& state, FrameDataPtr frameData) const;

  void addState(State& state);
  void addState(const Eigen::Vector3f& mean, const Eigen::Matrix3f& covar);
  void removeState(size_t index);
  void clearStates();

  Vector<State> _states;
  VoxelGrid _voxelGrid;
  // cached vclusters for points
  Vector<int> _vclusterPointTable;
  Vector<int> _stateMainVCluster;

  bool _enableTightFit;
  int _currentFrame = -1; // own frame counter, different from the one in FrameData
  Timer _frameTimer;

  // optional recorder
  PointCloudRecorder* _recorder = nullptr;
  // optional visualizer stuff
  inline bool hasVisualizer() const { return _visualizer != nullptr; }
  ObstacleTrackerVisualizer* _visualizer = nullptr;
  void fillVisualizerPointCloudColors(const Eigen::MatrixXf& R, const Eigen::MatrixXi& C);

  typedef Eigen::Map<Eigen::MatrixXf,
                     Eigen::Aligned,
                     Eigen::OuterStride<sizeof(pcl::PointXYZRGBA) / sizeof(float)>> VisualizerPCMapT;
  std::vector<pcl::PointXYZRGBA, Eigen::aligned_allocator<pcl::PointXYZRGBA>> _visualizerPointCloud;
};

// log of the gaussian probability density function
static float logpdf(const State& state, const Eigen::Vector4f& point)
{
  const Eigen::Vector3f x = point.head<3>() - state.pos;
  return state.logpdfConstantSummand - 0.5f * x.dot(state.obsCovarInv * x);
}

// gaussian probability density function for a state given a point
static float pdf(const State& state, const Eigen::Vector4f& point)
{
  const float lp = logpdf(state, point);
  const float p = expf(lp);

  return std::isnan(p) ? 0.0f : p;
}

// gaussian MLE fit
static void fitGaussianToPoints(const PointCloudT* pc, Eigen::Vector3f& outMean, Eigen::Matrix3f& outCovar)
{
  const auto map = pc->getMatrixXfMap();
  const Eigen::Vector4f mean = map.rowwise().mean();
  Eigen::Matrix4f covar = Eigen::Matrix4f::Zero();

  const size_t N = pc->size(); // number of points
  for (size_t i = 0; i < N; i++)
  {
    const Eigen::Vector4f x = map.col(i);
    covar += x * x.transpose();
    covar(3,3) += 1.0f;
  }

  covar /= static_cast<float>(N);
  covar -= mean * mean.transpose();

  outMean = mean.head<3>();
  outCovar = covar.block<3,3>(0,0);
}

void ObstacleTracker::initialize(const PointCloudT* pc)
{
  Eigen::Vector3f mean;
  Eigen::Matrix3f covar;
  // initialize by fitting a gaussian to the entire point cloud
  fitGaussianToPoints(pc, mean, covar);

  addState(mean, covar);

  for (auto& state : _states)
  {
    // initialize mixing coefficients uniformly
    state.pi = 1.0f / _states.size();
  }
}

void ObstacleTracker::emstep(const PointCloudT* pc, int frameNum)
{
  using namespace Eigen;

  const auto map = pc->getMatrixXfMap();
  const size_t N = pc->size(); // number of points

  // remove invalid states first (in reverse order because removeState does swap-with-end)
  for (int k = _states.size() - 1; k >= 0; k--)
  {
    if (!_states[k].validObsCovar)
    {
      // shouldn't happen too often
      std::cout << "removing state " << k << " due to invalid (non-invertible) observation covariance\n";
      removeState(k);
    }
  }

  if (N > _vclusterPointTable.size())
    _vclusterPointTable.resize(N);
  if (_states.size() > _stateMainVCluster.size())
    _stateMainVCluster.resize(_states.size());

  // R(i,j) = "responsibility" of state j for point i
  MatrixXf R(N, _states.size());
  // C(i,j) = points of state j that are in vcluster i (assuming a hard point-state assignment of sorts)
  MatrixXi C = MatrixXi::Zero(_voxelGrid.numClusters(), _states.size());

  Matrix4Xf VCMeans = Matrix4Xf::Zero(4, _voxelGrid.numClusters()); // vcluster means
  Vector<Matrix4f> VCSM; // vcluster scatter matrices
  VCSM.resize(_voxelGrid.numClusters(), Matrix4f::Zero());

  VectorXi VCPointCounts = VectorXi::Zero(_voxelGrid.numClusters()); // general vcluster point count

  Vector<State> newStates; // states that should be added
  Vector<int> removedStates; // indices of states that should be removed

  // e-step
  for (size_t i = 0; i < N; i++)
  {
    const Vector4f x = map.col(i);

    float normalizer = 0.0f;
    for (size_t k = 0; k < _states.size(); k++)
    {
      normalizer += _states[k].pi * pdf(_states[k], x);
    }

    const int vcluster = _voxelGrid.clusterForPoint(x);

#ifdef DEBUG
    if (vcluster < 0 || vcluster >= _voxelGrid.numClusters())
    {
      std::cout << vcluster << std::endl;
      throw std::runtime_error("invalid vcluster for point");
    }
#endif

    for (size_t k = 0; k < _states.size(); k++)
    {
      if (normalizer > 0.001f)
      {
        R(i,k) = _states[k].pi * pdf(_states[k], x) / normalizer;

        if (R(i,k) > HARD_ASSIGNMENT_STATE_RESP)
        {
          // this point likely "belongs" to state k, add contribution of state to vcluster of point
          ++C(vcluster, k);
        }
      }
      else
      {
        // this point has not enough probability support from any state - ignore it entirely
        R(i,k) = 0.0f;
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
    _vclusterPointTable[i] = vcluster;
  }

  // total responsibilities for states
  const VectorXf rks = R.colwise().sum();
  // total number of assigned points for clusters
  const VectorXi cks = C.colwise().sum();

  // remove states with low mixing coefficients, indicating that they don't hold any significant probability over points
  for (size_t k = 0; k < _states.size(); k++)
  {
    if (rks(k) / N < STATE_PI_REMOVAL_THRESHOLD)
    {
      // need to defer the removal if we want to avoid recomputing all of the above
      removedStates.push_back(k);
    }
  }

  // handle the addition of new states
  {
    const VectorXi vcpoints = C.rowwise().sum(); // sum of known points within the clusters

    for (int i = 0; i < vcpoints.size(); i++)
    {
      // got points that are already covered by a state or don't have enough points? ignore!
      if (vcpoints[i] > 0 || VCPointCounts[i] < MIN_VCLUSTER_POINTS)
        continue;

      const Vector4f vcmean = VCMeans.col(i);

      // compute actual covariance from scatter matrix
      Matrix4f vccov = VCSM[i] / VCPointCounts[i];
      vccov -= vcmean * vcmean.transpose();

      // combine with prior covariance (I*alpha) to avoid "overfitting" to the vcluster
      const float alpha = NEW_STATE_PRIOR_COVAR_MIX;
      vccov = alpha * Matrix4f::Identity() * NEW_STATE_PRIOR_COVAR_SIZE + (1.0f - alpha) * vccov;

      State s(vcmean.head<3>(), vccov.block<3,3>(0,0));
      s.pi = 0.05f; // TODO: this seems arbitrary

      newStates.push_back(s);
    }
  }

  // m-step
  for (size_t k = 0; k < _states.size(); k++)
  {
    if (rks(k) / N < STATE_PI_REMOVAL_THRESHOLD)
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
      _stateMainVCluster[k] = vclusterMain;
      // number of points in the vcluster containing the second most number of points
      const int splitPoints = C(vclusterOther, k);

      if (_states[k].lifeTime >= NUM_SPLIT_LIFE_TIME_FRAMES && splitPoints >= NUM_SPLIT_POINTS)
        _states[k].splitCounter++;
      else
        _states[k].splitCounter = 0;

      // TODO: else reset split counter?

      if (_states[k].splitCounter >= NUM_SPLIT_FRAMES)
      {
        // fit a gaussian to each part of the vcluster the state "owns"
        Vector4f meanA, meanB;
        Matrix4f covA, covB;
        fitSplittingGaussian(pc, R, k, vclusterMain, vclusterOther, meanA, meanB, covA, covB);

        // percentage of points in vclusterOther assigned to other states
        const float ot = (C.row(vclusterOther).sum() - splitPoints) / float(cks.sum() - cks(k));
        if (std::isnan(ot) || ot < SPLIT_MAX_OTHER_STATES_PERCENTAGE || _states[k].resetNonSplitCounter > 1)
        {
          // split state
          State a(meanA.head<3>(), covA.block<3,3>(0,0));
          State b(meanB.head<3>(), covB.block<3,3>(0,0));

          a.pi = _states[k].pi / 2.0f;
          b.pi = _states[k].pi / 2.0f;

          removedStates.push_back(k);

          newStates.push_back(a);
          newStates.push_back(b);
        }
        else
        {
          // avoid splitting if vclusterOther is mostly covered by other states
          // instead, reset the state by fitting to it's main vcluster
          _states[k].pos = meanA.head<3>();
          _states[k].setObsCovar(covA.block<3,3>(0,0));
          _states[k].lifeTime = 0;
          _states[k].splitCounter = 0;
          _states[k].resetNonSplitCounter++;
          // re-initialize the filter
          _states[k].initKalmanFilter();
        }
      }
    }
    else
    {
      _stateMainVCluster[k] = 0;
    }
    // == end splitting ==

    // actual m-step
    Vector4f mean = Vector4f::Zero();
    Matrix4f cov = Matrix4f::Zero();
    for (size_t i = 0; i < N; i++)
    {
      const Vector4f x = map.col(i);
      mean += R(i,k) * x;
      cov += R(i,k) * x * x.transpose();
    }

    mean /= rks(k);
    cov /= rks(k);

    _states[k].pos = mean.head<3>();

    // really important: regularize with previous covariance as prior (corresponding to a linear interpolation)
    // note that we only use a fixed interpolation alpha which seems to work well enough
    const float alpha = OBS_COVAR_REGULARIZATION;
    Matrix3f obsCovar = alpha * _states[k].obsCovar + (1-alpha) * (cov - mean * mean.transpose()).block<3,3>(0,0);
    _states[k].setObsCovar(obsCovar);

    _states[k].pi = rks(k) / N;
    _states[k].lifeTime += 1;

    _states[k].debugValue = _states[k].pi;

    if (_enableTightFit)
    {
      fitSSVs(k, pc, R, C);
    }
  }

  if (hasVisualizer())
    fillVisualizerPointCloudColors(R, C);

  // sort in descending order for this to work (removeState does swap-with-end)
  std::sort(removedStates.begin(), removedStates.end(), std::greater<int>());
  for (int k : removedStates)
  {
    removeState(k);
  }

  // add new states
  for (State& state : newStates)
  {
    addState(state);
  }
}

void ObstacleTracker::fitSplittingGaussian(const PointCloudT* pc, const Eigen::MatrixXf& R, int state,
                                           int vclusterA, int vclusterB, Eigen::Vector4f& outMeanA, Eigen::Vector4f& outMeanB,
                                           Eigen::Matrix4f& outCovA, Eigen::Matrix4f& outCovB) const
{
  outCovA.setZero();
  outCovB.setZero();
  outMeanA.setZero();
  outMeanB.setZero();

  const auto map = pc->getMatrixXfMap();
  const size_t N = pc->size(); // number of points
  float numA = 0.0f;
  float numB = 0.0f;

  for (size_t i = 0; i < N; i++)
  {
    const Eigen::Vector4f x = map.col(i);

    if (R(i,state) > HARD_ASSIGNMENT_STATE_RESP)
    {
      const int vcluster = _voxelGrid.clusterForPoint(x);
      if (vcluster == vclusterA)
      {
        outMeanA += x;
        outCovA += x * x.transpose();
        numA += 1.0f;
      }
      else
      {
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

void ObstacleTracker::fitSSVs(int k, const PointCloudT* pc, const Eigen::MatrixXf& R, const Eigen::MatrixXi& C)
{
  using namespace Eigen;

  State& state = _states[k];
  ObstacleSSVData& ssvData = state.ssvData;

  const auto map = pc->getMatrixXfMap();
  const size_t N = pc->size(); // number of points
  Vector3f mean = state.pos;

  SelfAdjointEigenSolver<Matrix3f> eigensolver(state.obsCovar);
  const Matrix3f evecs = eigensolver.eigenvectors();

  ssvData.ssvCapsuleMin = -0.01f;
  ssvData.ssvCapsuleMax = 0.01f;

  float radiusSphere = 0.0f;
  float radiusCapsule = 0.0f;

  const int mainVCluster = _stateMainVCluster[k];

  for (size_t i = 0; i < N; i++)
  {
    if (R(i,k) > HARD_ASSIGNMENT_STATE_RESP)
    {
      // if this point is in a vcluster (which is not the main vcluster) for which the state does not have enough points, ignore it
      if (_vclusterPointTable[i] != mainVCluster && C(_vclusterPointTable[i], k) < NUM_SPLIT_POINTS)
        continue;

      const Vector4f x = map.col(i);
      Vector3f a = x.head<3>() - mean;
      radiusSphere = std::max(radiusSphere, a.norm());

      // for the capsule, transform into eigenvector space first
      const Vector3f b = a.transpose() * evecs;

      // project onto the 2D plane spanned by the two smaller eigenvectors
      radiusCapsule = std::max(radiusCapsule, b.head<2>().norm());

      // take min/max along the largest eigenvector
      ssvData.ssvCapsuleMax = std::max(ssvData.ssvCapsuleMax, b.z());
      ssvData.ssvCapsuleMin = std::min(ssvData.ssvCapsuleMin, b.z());
    }
  }

  // clamp to some small value to avoid issues later on
  radiusSphere = std::max(0.01f, radiusSphere);
  radiusCapsule = std::max(0.01f, radiusCapsule);

  if (!ssvData.initialized)
  {
    ssvData.ssvRadiusSphere = radiusSphere;
    ssvData.ssvRadiusCapsule = radiusCapsule;
    ssvData.initialized = true;
  }
  else
  {
    const float alpha = SSV_EXPFILTER_ALPHA;
    ssvData.ssvRadiusSphere = alpha * ssvData.ssvRadiusSphere + (1.0f - alpha) * radiusSphere;
    ssvData.ssvRadiusCapsule = alpha * ssvData.ssvRadiusCapsule + (1.0f - alpha) * radiusCapsule;
  }
}

void ObstacleTracker::generateSSVs(State& state, FrameDataPtr frameData) const
{
  using namespace Eigen;

  auto& ssvData = state.ssvData;
  if (_enableTightFit && !ssvData.initialized)
    return;

  const Vector3f filteredPos = state.kalmanFilter.getState().position();

  SelfAdjointEigenSolver<Matrix3f> eigensolver(state.obsCovar);
  const Vector3f evals = eigensolver.eigenvalues();
  const Matrix3f evecs = eigensolver.eigenvectors();
  if (evals(1)/evals(2) < state.hystSplitVal)
  {
    // capsule
    const float radius = _enableTightFit ? ssvData.ssvRadiusCapsule : sqrtf(evals(1)) * SQRT_CHI_SQUARE;
    const Vector3f extent = _enableTightFit ?
                                   evecs.col(2) * ((ssvData.ssvCapsuleMax - ssvData.ssvCapsuleMin) * 0.5f - radius * 0.5f) :
                                   evecs.col(2) * (sqrtf(evals(2)) * SQRT_CHI_SQUARE - radius);

    Vector3f center = _parameters.filterSSVPositions ? filteredPos : state.pos;
    if (_enableTightFit)
    {
      // the actual center of the capsule is in the middle between the end points
      center = ((center + evecs.col(2) * ssvData.ssvCapsuleMax) +
                (center + evecs.col(2) * ssvData.ssvCapsuleMin)) / 2.0f;
    }

    const Vector3f pointA = center - extent;
    const Vector3f pointB = center + extent;
    frameData->obstacles.emplace_back(new CapsuleModel((double)radius, Coordinate(pointA), Coordinate(pointB)));

    state.hystSplitVal = 0.35f;

    if (hasVisualizer())
    {
      state.visData.isCapsule = true;
      state.visData.ssvRadius = radius;
      state.visData.ssvPointA = pointA.cast<double>();
      state.visData.ssvPointB = pointB.cast<double>();
    }
  }
  else
  {
    // sphere
    const float radius = _enableTightFit ? ssvData.ssvRadiusSphere : sqrtf(evals(2)) * SQRT_CHI_SQUARE;
    const Vector3f center = _parameters.filterSSVPositions ? filteredPos : state.pos;
    frameData->obstacles.emplace_back(new SphereModel(radius, Coordinate(center)));

    state.hystSplitVal = 0.25f;

    if (hasVisualizer())
    {
      state.visData.isCapsule = false;
      state.visData.ssvRadius = radius;
      state.visData.ssvPointA = center.cast<double>();
    }
  }

  const Coordinate velocity(state.kalmanFilter.getState().velocity());
  frameData->obstacles.back()->set_velocity(velocity);

#ifdef DEBUG
  if (state.visData.ssvPointA.hasNaN())
    raise(SIGTRAP);
  if (std::isinf(state.visData.ssvPointA[0]))
    raise(SIGTRAP);
#endif
}

void ObstacleTracker::addState(State& state)
{
  if (hasVisualizer())
    _visualizer->initVisData(state);

  _states.push_back(state);
}

void ObstacleTracker::addState(const Eigen::Vector3f& mean, const Eigen::Matrix3f& covar)
{
  _states.emplace_back(mean, covar);
  if (hasVisualizer())
    _visualizer->initVisData(_states.back());
}

void ObstacleTracker::removeState(size_t index)
{
  if (hasVisualizer())
    _visualizer->deinitVisData(_states[index]);

  // the old "swap with last element and pop_back" trick avoids execessive copies or linked lists
  std::swap(_states[index], _states.back());
  _states.pop_back();
}

void ObstacleTracker::clearStates()
{
  if (hasVisualizer())
  {
    for (size_t k = 0; k < _states.size(); k++)
      _visualizer->deinitVisData(_states[k]);
  }

  _states.clear();
}


void ObstacleTracker::updateFrame(FrameDataPtr frameData)
{
  using namespace Eigen;
#ifdef ENABLE_RECORDER
  if (frameData->frameNum == 0) // restarted the recording
    _currentFrame = -1;
#endif

  if (frameData->cloudMinusSurfaces->size() == 0)
    return;

  if (hasVisualizer())
  {
    using UserOption = ObstacleTrackerVisualizer::UserOption;
    // TODO: still notify observers here?
    if (!_visualizer->getUserOption<bool>(UserOption::EnableTracker))
      return;

    _enableTightFit = _visualizer->getUserOption<bool>(UserOption::EnableTightFit);
    _parameters.filterSSVPositions = _visualizer->getUserOption<bool>(UserOption::FilterSSVPositions);
  }
  else
  {
    // TODO: Make this a parameter in the config file
    _enableTightFit = true;
  }

#ifndef ENABLE_RECORDER
  // wait until we have a few ground removal iterations
  if (frameData->planeCoeffsIteration <= 2)
    return;

  PointCloudPtr cloud_filtered(new PointCloudT());

  // downsample the point cloud
  float dsResolution = DOWNSAMPLE_RESOLUTION;
  if (hasVisualizer())
    dsResolution = _visualizer->getUserOption<float>(ObstacleTrackerVisualizer::UserOption::DownsampleResolution);

  pcl::VoxelGrid<PointT> sor;
  sor.setInputCloud(frameData->cloudMinusSurfaces);
  sor.setLeafSize(dsResolution, dsResolution, dsResolution);
  if (dsResolution >= 0.03f)
    sor.setMinimumPointsNumberPerVoxel(2);
  if (dsResolution > 0.005f)
    sor.filter(*cloud_filtered);
  else
    cloud_filtered = frameData->cloudMinusSurfaces;

  cloud_filtered->is_dense = true;

  // crop if we have a visualizer
  if (hasVisualizer() && _parameters.enableCroppingPointCloudInUI)
  {
    pcl::CropBox<PointT> cropBox;
    Vector4f cropBoxMin;
    Vector4f cropBoxMax;
    _visualizer->getUserOptionCropBounds(cropBoxMin, cropBoxMax);

    cropBox.setInputCloud(cloud_filtered);
    cropBox.setMin(cropBoxMin);
    cropBox.setMax(cropBoxMax);
    cropBox.filter(*cloud_filtered);
  }
#else
  // in recorder mode, just take the the point cloud as it is
  PointCloudPtr cloud_filtered(frameData->cloudMinusSurfaces);
#endif

  const PointCloudT* pc = cloud_filtered.get();

  // prepare the visualizer point cloud, which has additional color values
  if (hasVisualizer())
  {
    const size_t N = pc->size(); // number of points
    if (N > _visualizerPointCloud.size())
      _visualizerPointCloud.resize(N); // realloc only when needed

    VisualizerPCMapT map(reinterpret_cast<float*>(_visualizerPointCloud.data()), 4, N);
    // copy position data to _visualizerPointCloud
    map = pc->getMatrixXfMap();
  }

  _currentFrame++;

  // initialize on the first frame
  bool shouldInit = _currentFrame == 0;

  if (shouldInit)
  {
    clearStates();

    Timer timer;
    timer.start();

    initialize(pc);

    timer.stop();
    std::cout << "init states took " << timer.duration() << "ms\n";
  }

  Timer perfTimer;
  perfTimer.start();

  // first build the voxel grid
  _voxelGrid.build(pc);

  // then do one step of the main algorithm
  emstep(pc, frameData->frameNum);

  // delta frame time in seconds
  double dt;
  if (_currentFrame == 0)
  {
    // assume 30hz for the first frame
    dt = 1/30.0f;
  }
  else
  {
    _frameTimer.stop();
    dt = _frameTimer.duration() / 1000.0f;
  }

  if (hasVisualizer())
    _visualizer->pushStat(ObstacleTrackerVisualizer::Stat::DeltaT, dt * 1000.0f);

  _frameTimer.start();

  // setup the kalman filter models
  ObstacleFilter::SystemModel sys(_parameters.kalman_SystemNoisePosition, _parameters.kalman_SystemNoiseVelocity, dt);
  ObstacleFilter::MeasurementModel mm(_parameters.kalman_MeasurementNoise);

  for (auto& state : _states)
  {
    // update the filtered state
    state.kalmanFilter.predict(sys);
    state.kalmanFilter.update(mm, ObstacleFilter::Measurement(state.pos));

    // add SSVs of obstacles to frameData (and optionally, state.visData)
    generateSSVs(state, frameData);

    // NOTE: to access velocity use state.kalmanFilter.getState().velocity()
  }

  perfTimer.stop();

  // doing this after perfTimer.stop()
  for (auto& state : _states)
  {
    // send latest data to the visualizer
    if (hasVisualizer())
      _visualizer->updateVisData(state);
  }

  if (hasVisualizer())
  {
    _visualizer->visualizePointCloud(_visualizerPointCloud.data(), pc->size()); // need to use size of pc, we don't always resize _visualizerPointCloud
    _visualizer->visualizeVoxelGrid(_voxelGrid);
    _visualizer->pushStat(ObstacleTrackerVisualizer::Stat::MainAlgorithmTime, perfTimer.duration());
  }

  // finally, push everything to our observers
  notifyObservers(frameData);
}

void ObstacleTracker::fillVisualizerPointCloudColors(const Eigen::MatrixXf& R, const Eigen::MatrixXi& C)
{
  using namespace Eigen;
  const size_t N = R.rows();
  const size_t K = R.cols();
  const auto colorMode = _visualizer->getUserOptionColorMode();
  if (colorMode == ObstacleTrackerVisualizer::ColorMode_HardAssignment)
  {
    for (size_t i = 0; i < N; i++)
    {
      pcl::PointXYZRGBA& p = _visualizerPointCloud[i];
      p.r = p.g = p.b = p.a = 0;

      for (size_t k = 0; k < K; k++)
      {
        if (R(i,k) > HARD_ASSIGNMENT_STATE_RESP)
        {
          const int mainVCluster = _stateMainVCluster[k];
          // mark the points that are not in the main vcluster, but in a vcluster for which the state doesn't have enough points to split
          if (_vclusterPointTable[i] != mainVCluster && C(_vclusterPointTable[i], k) < NUM_SPLIT_POINTS)
          {
            p.r = p.g = p.b = p.a = 255;
            continue;
          }

          const Vector3f col = rangeToColor<Vector3f, int>(0, K-1, k);
          p.b = col[0] * 255;
          p.g = col[1] * 255;
          p.r = col[2] * 255;
          p.a = 255;

          break;
        }
      }
    }
  }
  else if (colorMode == ObstacleTrackerVisualizer::ColorMode_SoftAssignment)
  {
    for (size_t i = 0; i < N; i++)
    {
      Vector3f col = Vector3f::Zero();

      for (size_t k = 0; k < K; k++)
        col += R(i,k) * rangeToColor<Vector3f, int>(0, K-1, k);

      pcl::PointXYZRGBA& p = _visualizerPointCloud[i];
      p.b = col[0] * 255;
      p.g = col[1] * 255;
      p.r = col[2] * 255;
      p.a = 255;
    }
  }
  else
  {
    for (size_t i = 0; i < N; i++)
    {
      pcl::PointXYZRGBA& p = _visualizerPointCloud[i];
      p.r = p.g = p.b = p.a = 255;
    }
  }
}

} // namespace lepp

#endif //LEPP3_OBSTACLETRACKER_H
