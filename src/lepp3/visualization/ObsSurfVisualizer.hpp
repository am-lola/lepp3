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
  virtual ~ModelDrawer() { }

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
  virtual void visitSphere(lepp::SphereModel& sphere)
  {
    Coordinate const& center = sphere.center();
    double centerPoint[3] = {center.x, center.y, center.z};
    double radius = sphere.radius();

    // update drawing variables
    model_radius_ = radius;
    model_ax_ = centerPoint[0];
    model_ay_ = centerPoint[1];
    model_az_ = centerPoint[2];
    model_bx_ = 0;
    model_by_ = 0;
    model_bz_ = 0;

    // ar::Sphere obstacle(centerPoint, radius, ar::Color(0,127,127,0.3));
    ar::Sphere obstacle(centerPoint, radius, ar::Color(0.6, 0.35, 0.2, 0.4));

    // sphere was not drawn before
    if (sphere.get_meshHandle() == -1)
    {
      mesh_handle_t mh = vis_->Add(obstacle);
      sphere.set_meshHandle(mh);
    }
    // update sphere
    else
      vis_->Update(sphere.get_meshHandle(), obstacle);

    // add handle to visHandles
    visHandles.push_back(sphere.get_meshHandle());
  }

  /**
  * Draw visited capsule.
  */
  virtual void visitCapsule(lepp::CapsuleModel& capsule)
  {
    // add capsule
    double center1[3] = {capsule.first().x, capsule.first().y, capsule.first().z};
    double center2[3] = {capsule.second().x, capsule.second().y, capsule.second().z};
    double radius = capsule.radius();

    // update drawing variables
    model_radius_ = radius;
    model_ax_ = center1[0];
    model_ay_ = center1[1];
    model_az_ = center1[2];
    model_bx_ = center2[0];
    model_by_ = center2[1];
    model_bz_ = center2[2];

    // ar::Capsule obstacle(center1, center2, radius, ar::Color(127,0,127,0.3));
    ar::Capsule obstacle(center1, center2, radius, ar::Color(0.0, 0.5, 0.5, 0.4));
    // capsule was not drawn before
    if (capsule.get_meshHandle() == -1)
    {
      mesh_handle_t mh = vis_->Add(obstacle);
      capsule.set_meshHandle(mh);
    }
    // update capsule
    else
      vis_->Update(capsule.get_meshHandle(), obstacle);

    // add handle to visHandles
    visHandles.push_back(capsule.get_meshHandle());
  }

private:
  /**
  * The instance to which the drawer will draw all models.
  */
  std::shared_ptr<ar::ARVisualizer> vis_;

  /**
  * Vector of all handles that are visualized in this frame.
  */
  std::vector<mesh_handle_t> &visHandles;

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
   * Destructor
   */
  virtual ~SurfaceDrawer() { }

  /**
  * Draw the convex hull of the visited Surface.
  */
  virtual void visitSurface(SurfaceModel &plane)
  {
    PointCloudConstPtr hull = plane.get_hull();
    int numPoints = static_cast<unsigned int>(hull->size());
    double points[numPoints * 3];
    for (int i = 0; i < numPoints; i++)
    {
      const PointT &p = hull->at(i);
      points[3*i] = p.x;
      points[3*i+1] = p.y;
      points[3*i+2] = p.z;
    }
    int colorID = surfaceCount % 6;
    surfaceCount++;
    ar::Polygon surfPoly(points, numPoints, ar::Color(r[colorID],g[colorID],b[colorID],1));

    // plane was not drawn before
    if (plane.get_meshHandle() == -1)
    {
      mesh_handle_t mh = vis_->Add(surfPoly);
      plane.set_meshHandle(mh);
    }
    // update plane
    else
      vis_->Update(plane.get_meshHandle(), surfPoly);

    // add handle to visHandles
    visHandles.push_back(plane.get_meshHandle());
  }
private:
  /**
  * The instance to which the drawer will draw all models.
  */
  std::shared_ptr<ar::ARVisualizer> vis_;
  // predefine colors
  static const int r[6];
  static const int b[6];
  static const int g[6];

  // count how many surfaces have been drawn so far to adapt colors.
  int surfaceCount;

  /**
  * Vector of all handles that are visualized in this frame.
  */
  std::vector<mesh_handle_t> &visHandles;
};

const int SurfaceDrawer::r[6] = {255,   0,   0, 255, 255,   0};
const int SurfaceDrawer::b[6] = {  0, 255,   0, 255,   0, 255};
const int SurfaceDrawer::g[6] = {  0,   0, 255,   0, 255, 255};


/**
 * Wrapper class for ARVisualizer that shows the result of obstacle and surface
 * detection.
 */

// TODO: [Sahand] Decide whether this should be a template-based class.
//       [Sahand] (My opinion): It can't be. ARVisualizer has its own point
//                type defined, and it'd be tricky to compare it with PCL point
//                types.
class ObsSurfVisualizer : public BaseVisualizer {

public:
	ObsSurfVisualizer(std::string const& name = "lepp3",
                    bool visualizeObstacles = false,
                    bool visualizeSurfaces = true,
                    int width = 1024, int height = 768)
      : BaseVisualizer(name, width, height),
        show_surfaces_(visualizeSurfaces),
        show_obstacles_(visualizeObstacles),
        pointCloudData(ar::PCL_PointXYZ),
        gridData(gridVector, gridSize, gridThickness, ar::Color( 0.5, 0.5, 0.5, 0.5 )),
        cosyX(cosy_o, cosy_x, 0.01f, ar::Color( 1.0, 0.0, 0.0 )),
        cosyY(cosy_o, cosy_y, 0.01f, ar::Color( 0.0, 1.0, 0.0 )),
        cosyZ(cosy_o, cosy_z, 0.01f, ar::Color( 0.0, 0.0, 1.0 )){
    // Updates the camera parameters used for rendering.
    // @position Position of the camera in world-coordinates
    // @forward  Vector pointing in the direction the camera is facing
    // @up       Orthogonal to Forward, defines the vertical axis for the camera
    //double position[3] = {0,0,0};
    //double forward[3] = {1,0,0};
    //double up[3] = {0,0,1};

    //arvis->SetCameraPose(position, forward, up);
    pointCloudHandle = arvis_->Add(pointCloudData);
    arvis_->Add(cosyX);
    arvis_->Add(cosyY);
    arvis_->Add(cosyZ);
    gridHandle = arvis_->Add(gridData);
    gridWindow = arvis_->AddUIWindow("Grid");
    gridCheckBox = gridWindow->AddCheckBox("Draw", true);
    pccolorWindow = arvis_->AddUIWindow("Point Cloud");
    editPCColor = pccolorWindow->AddColorEdit4("Color", pccolorvalue);
    }

  /**
   * Destructor
   */
	~ObsSurfVisualizer() { }

  /**
   * Blocks the calling thread until the visualization window is closed.
   */
  //  TODO: decide where to put this one... (BaseVisualizer, is it even necessary?)
  // void waitForClose()
  // {
  //   arvis->WaitForClose();
  // }

  /**
  * Visualize obstacles and surfaces of the given frame.
  */
  virtual void updateFrame(FrameDataPtr frameData);
  // ar::ARVisualizer* getVisualizer() const { return arvis; };
  /**
* `RGBDataObserver` interface implementation.
*/
  void updateFrame(RGBDataPtr rgbData);

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

  // visulize obstacles and surfaces only if options were chosen in config file
  bool show_surfaces_;
  bool show_obstacles_;

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
  ar::IUIWindow* gridWindow;
  ar::ui_element_handle gridCheckBox;
  ar::IUIWindow* pccolorWindow;
  ar::ui_element_handle editPCColor;
  float pccolorvalue[4] { 1.0f, 1.0f, 1.0f, 1.0f };
};


void ObsSurfVisualizer::drawSurfaces(std::vector<SurfaceModelPtr> surfaces, std::vector<mesh_handle_t> &visHandles)
{
  // create surface drawer object
  SurfaceDrawer sd(arvis_, visHandles);

  // draw surfaces
  for (size_t i = 0; i < surfaces.size(); i++)
    surfaces[i]->accept(sd);
}


void ObsSurfVisualizer::drawObstacles(std::vector<ObjectModelPtr> obstacles, std::vector<mesh_handle_t> &visHandles)
{
  // create model drawer object
  ModelDrawer md(arvis_, visHandles);

  // draw obstacles
  for (size_t i = 0; i < obstacles.size(); i++)
    obstacles[i]->accept(md);
}


void ObsSurfVisualizer::outputFrameNum(FrameDataPtr frameData)
{
  /*std::cout << "Frame " << frameData->frameNum << "    "
    << "Ransac " << frameData->planeCoeffsIteration << "    "
    << "RansacRef " << frameData->planeCoeffsReferenceFrameNum;
  if (show_surfaces_)
    std::cout << "    ";
  else
    std::cout << std::endl;


  if (show_surfaces_)
  {
    std::cout << "Surfaces " << frameData->surfaceDetectionIteration << "    "
      << "SurfacesRef " << frameData->surfaceReferenceFrameNum << std::endl;
  }*/
}


void ObsSurfVisualizer::removeOldSurfObst(std::vector<mesh_handle_t> &visHandles)
{
  // compare the newly visualized handles with the old ones. Remove all handles that appear
  // in the old handle list but not in the new one.
  std::sort(visHandles.begin(), visHandles.end());
  for (mesh_handle_t &mh : oldHandles)
  {
    // if mesh handle is not contained in newly visualized handles, remove it from visualizer
    if (!std::binary_search(visHandles.begin(), visHandles.end(), mh))
      arvis_->Remove(mh);
  }
  oldHandles = visHandles;
}


void ObsSurfVisualizer::updateFrame(FrameDataPtr frameData)
{
  // visualize all obstacles and surfaces and store their handles
  std::vector<mesh_handle_t> visHandles;
  if (show_obstacles_)
    drawObstacles(frameData->obstacles, visHandles);

  if (show_surfaces_)
    drawSurfaces(frameData->surfaces, visHandles);

  // Remove old obstacles and surfaces that are no longer visualized
  removeOldSurfObst(visHandles);

  // output frame num and surface frame num
  outputFrameNum(frameData);

  // visualize the point cloud
  pointCloudData.pointData = reinterpret_cast<const void*>(&(frameData->cloud->points[0]));
  pointCloudData.numPoints = frameData->cloud->size();
  pccolorWindow->GetColorValues4(editPCColor, pccolorvalue);
  pointCloudData.color = ar::Color(pccolorvalue[0], pccolorvalue[1], pccolorvalue[2], pccolorvalue[3]);
  arvis_->Update(pointCloudHandle, pointCloudData);
  arvis_->SetVisibility(gridHandle, (bool)gridWindow->GetCheckBoxState(gridCheckBox));
}

void ObsSurfVisualizer::updateFrame(RGBDataPtr rgbData) {
  return;
};

} // namespace lepp

#endif // LEPP3_ARVISUALIZER_H__
