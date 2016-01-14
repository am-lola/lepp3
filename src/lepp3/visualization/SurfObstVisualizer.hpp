#ifndef LEPP3_VISUALIZATION_SURFOBST_VISUALIZER_H__
#define LEPP3_VISUALIZATION_SURFOBST_VISUALIZER_H__

#include <sstream>
#include <vector>

#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>

#include "lepp3/Typedefs.hpp"
#include "lepp3/VideoObserver.hpp"
#include "lepp3/ObstacleAggregator.hpp"
#include "lepp3/models/ObjectModel.h"
#include "lepp3/SurfaceAggregator.hpp"
#include "lepp3/ConvexHullDetector.hpp"

namespace lepp {

/**
 * A class that can draw `ObjectModel` instances onto a PCLVisualizer.
 * It is a `ModelVisitor` implementation and it draws each model that it visits.
 *
 * Therefore, in order to draw some model instance, the client needs to have it
 * accept an instance of this drawer.
 *
 * The `ModelDrawer` should be able to represent any `ObjectModel` instance.
 *
 */
class ModelDrawer : public ModelVisitor {
public:
  /**
   * Creates a new `ModelDrawer` instance that will draw any models that it
   * visits onto the given viewer.
   */
  ModelDrawer(pcl::visualization::PCLVisualizer& viewer)
      : viewer_(viewer), total_(0) {}

  /**
   * Implementation of the `ModelVisitor` interface. It will draw the given
   * sphere onto the `PCLVisualizer` to which it holds a reference.
   */
  void visitSphere(lepp::SphereModel& sphere);
  void visitCapsule(lepp::CapsuleModel& capsule);
private:
  /**
   * The instance to which the drawer will draw all models.
   */
  pcl::visualization::PCLVisualizer& viewer_;
  /**
   * Counts how many models have been drawn by the drawer.
   */
  int total_;
};

void ModelDrawer::visitSphere(lepp::SphereModel& sphere) {
  // We count how many object have been drawn in total, in order to make sure
  // we don't use any duplicate names.
  // The drawer relies on there not being any objects with a prefix "obstacle"
  // in their name before its instantiation.
  ++total_;

  Coordinate const& center = sphere.center();
  std::ostringstream ss; ss << "obstacle " << total_;
  std::string const name = ss.str();

  // Add it to the view...
  viewer_.addSphere(
      pcl::PointXYZ(center.x, center.y, center.z),
      sphere.radius(),
      0, .5, .5,    // r, g, b
      name);
  // ...and set opacity to 30%
  viewer_.setShapeRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_OPACITY,
      0.3,
      name);
}

void ModelDrawer::visitCapsule(lepp::CapsuleModel& capsule) {
  ++total_;
  std::ostringstream ss; ss << "obstacle " << total_;
  std::string const name = ss.str();

  pcl::ModelCoefficients cylinder_coeff;
  // We need 7 values for a cylinder.
  cylinder_coeff.values.resize(7);
  cylinder_coeff.values[0] = capsule.first().x;
  cylinder_coeff.values[1] = capsule.first().y;
  cylinder_coeff.values[2] = capsule.first().z;
  cylinder_coeff.values[3] = capsule.second().x - capsule.first().x;
  cylinder_coeff.values[4] = capsule.second().y - capsule.first().y;
  cylinder_coeff.values[5] = capsule.second().z - capsule.first().z;
  cylinder_coeff.values[6] = capsule.radius();

  float const r = .5;
  float const g = 0;
  float const b = .5;
  // First, we add the cylinder.
  viewer_.addCylinder(cylinder_coeff, name);
  viewer_.setShapeRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_OPACITY, 0.3, name);
  viewer_.setShapeRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_COLOR,
      r, g, b,
      name);

  // And now the two spheres at either end of the capsule.
  viewer_.addSphere(
      pcl::PointXYZ(capsule.first().x, capsule.first().y, capsule.first().z),
      capsule.radius(),
      r, g, b,
      name + "s1");
  viewer_.setShapeRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_OPACITY, 0.3, name + "s1");
  viewer_.addSphere(
      pcl::PointXYZ(capsule.second().x, capsule.second().y, capsule.second().z),
      capsule.radius(),
      r, g, b,
      name + "s2");
  viewer_.setShapeRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_OPACITY, 0.2, name + "s2");
}
/**
 * Visualizes the obstacles detected by a particular obstacle detector by
 * overlaying them onto a point cloud feed coming from a video source.
 *
 * Implements the VideoObserver and ObstacleAggregator interfaces.
 */
template<class PointT>
class SurfObstVisualizer : public VideoObserver<PointT>, public ObstacleAggregator, public SurfaceAggregator<PointT> {
public:
  SurfObstVisualizer() : viewer_("SurfObstVisualizer") {}

  /**
   * VideoObserver interface implementation: processes the current point cloud.
   */
  virtual void notifyNewFrame(
      int idx,
      const PointCloundConstPtr& pointCloud) {
    viewer_.showCloud(pointCloud);
  }

  /**
   * ObstacleAggregator interface implementation: processes detected obstacles.
   */
  virtual void updateObstacles(std::vector<ObjectModelPtr> const& obstacles);

  /**
   * SurfaceAggregator interface implementation: processes detected surfaces.
  */
  virtual void updateSurfaces(
      std::vector<PointCloundConstPtr> surfaces,
      PointCloudPtr &cloudMinusSurfaces,
      std::vector<pcl::ModelCoefficients> *&surfaceCoefficients);

  
private:
  /**
   * Used for the visualization of the scene.
   */
  pcl::visualization::CloudViewer viewer_;

  /**
   * Private member function that is used to set up a callback which is executed
   * by the viewer when the detected obstacles should be visualized.
   */
  void drawShapes(std::vector<ObjectModelPtr> obstacles,
                  pcl::visualization::PCLVisualizer& viewer);


  void drawSurfaces(
            std::vector<PointCloundConstPtr> surfaces,
      pcl::visualization::PCLVisualizer& viewer);
};

template<class PointT>
void SurfObstVisualizer<PointT>::drawShapes(
    std::vector<ObjectModelPtr> obstacles,
    pcl::visualization::PCLVisualizer& viewer) {
  // Remove all old shapes...
  viewer.removeAllShapes();

  // ...and draw all the new ones.
  ModelDrawer drawer(viewer);
  size_t const sz = obstacles.size();
  for (size_t i = 0; i < sz; ++i) {
    obstacles[i]->accept(drawer);
  }
}

template<class PointT>
void SurfObstVisualizer<PointT>::updateObstacles(
    std::vector<ObjectModelPtr> const& obstacles) {
  pcl::visualization::CloudViewer::VizCallable obstacle_visualization =
      boost::bind(&SurfObstVisualizer::drawShapes,
                  this, obstacles, _1);
  viewer_.runOnVisualizationThreadOnce(obstacle_visualization);
}


template<class PointT>
void SurfObstVisualizer<PointT>::drawSurfaces(
        std::vector<PointCloundConstPtr> surfaces,
    pcl::visualization::PCLVisualizer& pclViz) {

     pclViz.removePointCloud("SUR1",0);
     pclViz.removePointCloud("SUR2",0);
     pclViz.removePointCloud("SUR3",0);
     pclViz.removePointCloud("SUR4",0);

    size_t const sz = surfaces.size();
  for (size_t i = 0; i < sz; ++i) {

     if (i ==0) {
            if (!pclViz.updatePointCloud(surfaces[i], "SUR1"))
                pclViz.addPointCloud(surfaces[i], "SUR1");                       //RED
      pclViz.setPointCloudRenderingProperties(
          pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0, 0,
                    "SUR1");
    }
    else if (i == 1) {
            if (!pclViz.updatePointCloud(surfaces[i], "SUR2"))                  //GREEN
                pclViz.addPointCloud(surfaces[i], "SUR2");
      pclViz.setPointCloudRenderingProperties(
          pcl::visualization::PCL_VISUALIZER_COLOR, 0, 1.0, 0,
                    "SUR2");
    }
    else if (i == 2) {
            if (!pclViz.updatePointCloud(surfaces[i], "SUR3"))                  //BLUE
                pclViz.addPointCloud(surfaces[i], "SUR3");
      pclViz.setPointCloudRenderingProperties(
          pcl::visualization::PCL_VISUALIZER_COLOR, 0, 0, 1.0,
                    "SUR3");
    }
    else if (i == 3) {
            if (!pclViz.updatePointCloud(surfaces[i], "SUR4"))
                pclViz.addPointCloud(surfaces[i], "SUR4");                       //PURPLE
      pclViz.setPointCloudRenderingProperties(
          pcl::visualization::PCL_VISUALIZER_COLOR, 0.580392, 0,
                    0.827451, "SUR4");
    }
//    else if (i == 5) {
//      if (!pclViz.updatePointCloud(surfaces[i], "SUR5"))
//        pclViz.addPointCloud(surfaces[i], "SUR5");                      //YELLOW
//      pclViz.setPointCloudRenderingProperties(
//          pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0.843137, 0,
//          "SUR5");
//    }
  }

  //std::cout << "exiting stair drawer..." << std::endl;
  //std::cout << "=======================" << std::endl;
}

template<class PointT>
void SurfObstVisualizer<PointT>::updateSurfaces(
        std::vector<PointCloundConstPtr> surfaces,
      PointCloudPtr &cloudMinusSurfaces,
      std::vector<pcl::ModelCoefficients> *&surfaceCoefficients) {
    //std::cout << "entered updateSurfaces" << std::endl;
    pcl::visualization::CloudViewer::VizCallable surface_visualization =
            boost::bind(&SurfObstVisualizer::drawSurfaces, this, surfaces, _1);
  //std::cout << "call runOnVisualizationThreadOnce" << std::endl;
    viewer_.runOnVisualizationThread(surface_visualization);
    //std::cout << "exiting updateSurfaces..." << std::endl;
}


} // namespace lepp
#endif
