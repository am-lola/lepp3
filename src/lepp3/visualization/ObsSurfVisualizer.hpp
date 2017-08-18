#ifndef LEPP3_ARVISUALIZER_H__
#define LEPP3_ARVISUALIZER_H__

#include "lepp3/FrameData.hpp"
#include "lepp3/Typedefs.hpp"
#include "lepp3/visualization/BaseVisualizer.hpp"
#include "lepp3/models/ObjectModel.h"
#include "lepp3/models/SurfaceModel.h"
#include "lepp3/CalibrationAggregator.hpp"

#include <vector>
#include <algorithm>
#include <iostream>

namespace lepp
{
/**
* Visitor class for obstacles.
*/
class ModelDrawer : public ModelVisitor
{
public:
  ModelDrawer(
      std::shared_ptr<ar::ARVisualizer> v,
      std::vector<mesh_handle_t> &visHandles)
    : vis_(v),
      visHandles(visHandles) { }

  double model_radius() const { return model_radius_; }
  double model_ax() const { return model_ax_; }
  double model_ay() const { return model_ay_; }
  double model_az() const { return model_az_; }
  double model_bx() const { return model_bx_; }
  double model_by() const { return model_by_; }
  double model_bz() const { return model_bz_; }

  /**
  * Draw visited sphere.
  */
  virtual void visitSphere(lepp::SphereModel& sphere) override;

  /**
  * Draw visited capsule.
  */
  virtual void visitCapsule(lepp::CapsuleModel& capsule) override;

private:
  /**
  * The instance to which the drawer will draw all models.
  */
  std::shared_ptr<ar::ARVisualizer> vis_;

  /**
  * Vector of all handles that are visualized in this frame.
  */
  std::vector<mesh_handle_t>& visHandles;

  double model_radius_;
  double model_ax_;
  double model_ay_;
  double model_az_;
  double model_bx_;
  double model_by_;
  double model_bz_;
};



/**
* Visitor class for surfaces.
*/
class SurfaceDrawer : public SurfaceVisitor
{
public:
  SurfaceDrawer(std::shared_ptr<ar::ARVisualizer> v,
                std::vector<mesh_handle_t> &visHandles)
      : vis_(v),
        surfaceCount(0),
        visHandles(visHandles) { }

  /**
  * Draw the convex hull of the visited Surface.
  */
  virtual void visitSurface(SurfaceModel &plane) override;

private:
  /**
  * The instance to which the drawer will draw all models.
  */
  std::shared_ptr<ar::ARVisualizer> vis_;

  // predefine colors
  static const ar::Color colors[6];

  // count how many surfaces have been drawn so far to adapt colors.
  int surfaceCount;

  /**
  * Vector of all handles that are visualized in this frame.
  */
  std::vector<mesh_handle_t> &visHandles;
};


/**
 * Parameters for ObsSurfVisualizer
 */
struct ObsSurfVisualizerParameters {
  std::string name = "lepp3";
  bool show_obstacles = false;
  bool show_surfaces = true;
  bool show_grid = true;
  bool show_obstacle_clouds = false;
  int width = 1024;
  int height = 768;
};

/**
 * Wrapper class for ARVisualizer that shows the result of obstacle and surface
 * detection.
 */
class ObsSurfVisualizer : public BaseVisualizer {
public:
  ObsSurfVisualizer() : ObsSurfVisualizer(ObsSurfVisualizerParameters{}) {}
  ObsSurfVisualizer(ObsSurfVisualizerParameters const& params);

  /**
  * Visualize obstacles and surfaces of the given frame.
  */
  virtual void updateFrame(FrameDataPtr frameData) override;

  /**
  * `RGBDataObserver` interface implementation.
  */
  void updateFrame(RGBDataPtr rgbData) override;

private:
  /**
   * Visualize convex hulls of surfaces in given vector with ARVisualizer.
   */
  void drawSurfaces(std::vector<SurfaceModelPtr> surfaces, std::vector<mesh_handle_t> &visHandles);

  /**
   * Visualize obstacles in given vector with ARVisualizer.
   */
  void drawObstacles(std::vector<ObjectModelPtr> obstacles, std::vector<mesh_handle_t> &visHandles);

  /**
   * Output the number of the frame.
   */
  void outputFrameNum(FrameDataPtr frameData);

  /**
  * Remove old obstacles and surfaces that are no longer visualized.
  */
  void removeOldSurfObst(std::vector<mesh_handle_t> &visHandles);

  /**
   * Visualize obstacles in given vector with ARVisualizer.
   */
  void drawObstacleClouds(std::vector<ObjectModelParams> const& clouds);


  ObsSurfVisualizerParameters params_;

  // vector that holds the handles to all obstacles and surfaces that were visualized in the previous frame
  std::vector<mesh_handle_t> oldHandles;

  ar::mesh_handle pointCloudHandle;
  ar::PointCloudData pointCloudData;

//  Coordinate System xyz = rgb, size 0,2m x 0,01m
  double cosy_o[3] = { 0.0, 0.0, 0.0 };
  double cosy_x[3] = { 0.2, 0.0, 0.0 };
  double cosy_y[3] = { 0.0, 0.2, 0.0 };
  double cosy_z[3] = { 0.0, 0.0, 0.2 };
  ar::LineSegment cosyX;
  ar::LineSegment cosyY;
  ar::LineSegment cosyZ;

//  Grid for visualization reference: 4m x 4m, with corners at
//  (0,-2, 0)
//  (0, 2, 0)
//  (4,-2, 0)
//  (4, 2, 0)
  ar::mesh_handle gridHandle;

  float gridThickness = 0.001f;
  size_t gridSize = 19;
  double gridVector[57] = {
          0.0,-2.0, 0.0,
          0.0, 2.0, 0.0,
          1.0, 2.0, 0.0,
          1.0,-2.0, 0.0,
          2.0,-2.0, 0.0,
          2.0, 2.0, 0.0,
          3.0, 2.0, 0.0,
          3.0,-2.0, 0.0,
          4.0,-2.0, 0.0,
          4.0, 2.0, 0.0,
          0.0, 2.0, 0.0,
          0.0, 1.0, 0.0,
          4.0, 1.0, 0.0,
          4.0, 0.0, 0.0,
          0.0, 0.0, 0.0,
          0.0,-1.0, 0.0,
          4.0,-1.0, 0.0,
          4.0,-2.0, 0.0,
          0.0,-2.0, 0.0
  };
  ar::LinePath gridData;
  ar::IUIWindow* optionsWindow;
  ar::ui_element_handle gridCheckBox;
  ar::ui_element_handle surfacesCheckBox;
  ar::ui_element_handle obstaclesCheckBox;
  ar::ui_element_handle obstacleCloudsCheckBox;
  ar::IUIWindow* pccolorWindow;
  ar::ui_element_handle editPCColor;
  ar::ui_element_handle setPCColor;
  ar::ui_element_handle PCColorCheckBox;
  float pccolorvalue[4] { 1.0f, 1.0f, 1.0f, 1.0f };
  int presetpccolor = 2;

  std::vector<ar::mesh_handle> obstacleCloudHandles;
  std::vector<std::unique_ptr<ar::PointCloudData>> obstacleCloudData;
};

} // namespace lepp

#endif // LEPP3_ARVISUALIZER_H__
