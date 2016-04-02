#ifndef LEPP3_ARVISUALIZER_H__
#define LEPP3_ARVISUALIZER_H__

#include "lepp3/FrameData.hpp"
#include "lepp3/Typedefs.hpp"
#include "lepp3/models/ObjectModel.h"
#include "lepp3/models/SurfaceModel.h"

#include <am2b-arvis/ARVisualizer.hpp>
#include <vector>
#include <algorithm>

namespace lepp 
{
/**
* Visitor class for obstacles.
*/
class ModelDrawer : public ModelVisitor 
{
public:
  ModelDrawer(ar::ARVisualizer *arvis) : arvis(arvis) {}
  virtual ~ModelDrawer() {}

  /**
  * Draw visited sphere.
  */
  virtual void visitSphere(lepp::SphereModel& sphere)
  {
    Coordinate const& center = sphere.center();
    double centerPoint[3] = {center.x, center.y, center.z};
    double radius = sphere.radius();
    ar::Sphere obstacle(centerPoint, radius, ar::Color(0,127,127,0.7));
    arvis->Add(obstacle);
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
    ar::Capsule cap(center1, center2, radius, ar::Color(127,0,127,0.7));
    arvis->Add(cap);
  }

private:
  /**
  * The instance to which the drawer will draw all models.
  */
  ar::ARVisualizer *arvis;
};



/**
* Visitor class for surfaces.
*/
class SurfaceDrawer : public SurfaceVisitor
{
public:
  SurfaceDrawer(ar::ARVisualizer *arvis) : arvis(arvis), surfaceCount(0) {}
  virtual ~SurfaceDrawer() {}

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
    arvis->Add(surfPoly);
  }
private:
  /**
  * The instance to which the drawer will draw all models.
  */
  ar::ARVisualizer *arvis;

  // predefine colors
  static const int r[6];
  static const int b[6];
  static const int g[6];

  // count how many surfaces have been drawn so far to adapt colors.
  int surfaceCount;
};

const int SurfaceDrawer::r[6] = {255,   0,   0, 255, 255,   0};
const int SurfaceDrawer::b[6] = {  0, 255,   0, 255,   0, 255};
const int SurfaceDrawer::g[6] = {  0,   0, 255,   0, 255, 255};





/**
* Wrapper class for ARVisualizer. Does communication with library visualizer.
*/
class ARVisualizer : public FrameDataObserver  
{
public:
	ARVisualizer(bool visualizeSurfaces, bool visualizeObstacles) : 
    arvis(new ar::ARVisualizer()),
    visualizeSurfaces(visualizeSurfaces),
    visualizeObstacles(visualizeObstacles)
	{  
    // Updates the camera parameters used for rendering.
    // @position Position of the camera in world-coordinates
    // @forward  Vector pointing in the direction the camera is facing
    // @up       Orthogonal to Forward, defines the vertical axis for the camera
    //double position[3] = {0,0,0};
    //double forward[3] = {1,0,0};
    //double up[3] = {0,0,1};
    
		arvis->Start();
    //arvis->SetCameraPose(position, forward, up);
    
	}
	~ARVisualizer()
	{
		arvis->Stop();
		delete arvis;
	}

  /**
  * Visualize obstacles and surfaces of the given frame.
  */
  virtual void updateFrame(FrameDataPtr frameData);

private:
  // instance of ARVisualizer
	ar::ARVisualizer *arvis;

  /**
  * Visualize convex hulls of surfaces in given vector with ARVisualizer.
  */
  void drawSurfaces(std::vector<SurfaceModelPtr> surfaces);

  /**
  * Visualize obstacles in given vector with ARVisualizer.
  */
  void drawObstacles(std::vector<ObjectModelPtr> obstacles);

  // visulize obstacles and surfaces only if options were chosen in config file
  bool visualizeSurfaces;
  bool visualizeObstacles;
};


void ARVisualizer::drawSurfaces(std::vector<SurfaceModelPtr> surfaces)
{
  // create surface drawer object
  SurfaceDrawer sd(arvis);

  // draw surfaces
  for (size_t i = 0; i < surfaces.size(); i++)
    surfaces[i]->accept(sd);
}


void ARVisualizer::drawObstacles(std::vector<ObjectModelPtr> obstacles)
{
  // create model drawer object
  ModelDrawer md(arvis);

  // draw obstacles
  for (size_t i = 0; i < obstacles.size(); i++)
    obstacles[i]->accept(md);
}


void ARVisualizer::updateFrame(FrameDataPtr frameData)
{
  // remove all objects drawn with the visualizer on the previous frame.
	arvis->RemoveAll();
  
  //draw surfaces
  if (visualizeSurfaces)
    drawSurfaces(frameData->surfaces);

  // draw obstacles
  if (visualizeObstacles)
    drawObstacles(frameData->obstacles);
}


} // namespace lepp
#endif