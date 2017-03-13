#ifndef LEPP_OBSTACLES_SEGMENTER_GMM_SEGMENTER_H
#define LEPP_OBSTACLES_SEGMENTER_GMM_SEGMENTER_H

#include "lepp3/Typedefs.hpp"
#include "lepp3/obstacles/segmenter/Segmenter.hpp"
#include "lepp3/util/VoxelGrid.h"

#include "GmmData.hpp"

#include <chrono>

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>

namespace lepp {

class GmmSegmenter : public ObstacleSegmenter {
  using VisualizerPCMapT = Eigen::Map<Eigen::MatrixXf, Eigen::Aligned, Eigen::OuterStride<
      sizeof(pcl::PointXYZRGBA) / sizeof(float)>>;

public:
  GmmSegmenter(GMM::SegmenterParameters const& params);

private:
  virtual std::vector<PointCloudPtr> extractObstacleClouds(PointCloudConstPtr cloud) override;

  void initialize(PointCloudT const* pc);

  void e_step(PointCloudT const* pc, Eigen::MatrixXf& R, Eigen::MatrixXi& C,
              Eigen::Matrix4Xf& VCMeans, std::vector<Eigen::Matrix4f>& VCSM,
              Eigen::VectorXi& VCPointCounts);

  void m_step(PointCloudT const* pc, Eigen::MatrixXf const& R, Eigen::MatrixXi const& C, Eigen::VectorXf const& rks,
              Eigen::VectorXi const& cks, std::vector<GMM::State>& newStates, std::vector<int>& removedStates);

  //void emstep(const PointCloudT* pc, int frameNum);
  // fit two gaussians to both parts of the vclusters that are assigned to a state
  void fitSplittingGaussian(PointCloudT const* pc, Eigen::MatrixXf const& R, size_t state,
                            int vclusterA, int vclusterB, Eigen::Vector4f& outMeanA, Eigen::Vector4f& outMeanB,
                            Eigen::Matrix4f& outCovA, Eigen::Matrix4f& outCovB) const;

  void addState(GMM::State& state);

  void addState(const Eigen::Vector3f& mean, const Eigen::Matrix3f& covar);

  void removeState(size_t index);


  GMM::SegmenterParameters parameters_;

  std::vector<GMM::State> states_;
  VoxelGrid voxel_grid_;

  // cached vclusters for points
  std::vector<int> vcluster_point_table;
  std::vector<int> state_main_vcluster;

  bool initialized_;
};

}

#endif

/*std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();
    std::cout << "Hello World\n";
    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    std::cout << "Printing took "
              << std::chrono::duration_cast<std::chrono::microseconds>(end - start).count()
              << "us.\n";*/
