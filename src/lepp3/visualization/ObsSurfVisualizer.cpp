#include "ObsSurfVisualizer.hpp"

namespace {

/**
 * clamps a value to the given bounds
 */
template<typename T>
T clamp(const T& value, T low, T high) {
  assert(low < high);

  if (value < low) {
    return low;
  } else if (value > high) {
    return high;
  } else {
    return value;
  }
}


/**
 * Converts an HSV color to an RGB color
 * @param h hue, [0; 360)
 * @param s saturation [0; 1]
 * @param v value [0; 1]
 * @return
 */
ar::Color HsvToRgbColor(float h, float s, float v) {
  h = clamp(h, 0.0f, 360.0f);
  s = clamp(s, 0.0f, 1.0f);
  v = clamp(v, 0.0f, 1.0f);

  if (360.0f == h) {
    h = 0.0f;
  }

  int hi = static_cast<int>(h / 60.0f);
  float f = h / 60.0f - hi;

  float p = v * (1- s);
  float q = v * (1 - s * f);
  float t = v * (1 - s * (1 - f));

  switch (hi) {
  case 0:
    return ar::Color(v, t, p);

  case 1:
    return ar::Color(q, v, p);

  case 2:
    return ar::Color(p, v, t);

  case 3:
    return ar::Color(p, q, v);

  case 4:
    return ar::Color(t, p, v);

  case 5:
    return ar::Color(v, p, q);

  default:
    assert(false);
    // Shouldn't happen, return white
    return ar::Color(1.0f, 1.0f, 1.0f);
  }
}

}


void lepp::ModelDrawer::visitSphere(lepp::SphereModel& sphere)
{
  Coordinate const& center = sphere.center();
  Coordinate const& sphereVelocity = sphere.velocity();
  double centerPoint[3] = {center.x, center.y, center.z};
  Eigen::Vector3d velocity_step = 0.2*Eigen::Vector3d(sphereVelocity.x, sphereVelocity.y, sphereVelocity.z) + Eigen::Vector3d(center.x, center.y, center.z);
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
  ar::LineSegment obstacle_vel(centerPoint, velocity_step.data(), 0.005f);

  auto sphereData = visData.find(sphere.id());
  // sphere was not drawn before
  if (sphereData == visData.end())
  {
    ObstacleVisualizationData vd;
    vd.mh = vis_->Add(obstacle);
    vd.vh = vis_->Add(obstacle_vel);
    vd.lp = new ar::BufferedLinePath(10, 0.003f, ar::Color(1,1,1,1));
    vd.lp->addPoint(centerPoint);
    vd.th = vis_->Add(*vd.lp);
    visData.insert(std::make_pair(sphere.id(), vd));
  }
  // update sphere
  else
  {
    vis_->Update(sphereData->second.mh, obstacle);
    vis_->Update(sphereData->second.vh, obstacle_vel);
    sphereData->second.lp->addPoint(centerPoint);
    vis_->Update(sphereData->second.th, *(sphereData->second.lp));
  }

  // add handle to visHandles
  seenObstacles.push_back(sphere.id());
}

void lepp::ModelDrawer::visitCapsule(lepp::CapsuleModel& capsule)
{
  // add capsule
  double center1[3] = {capsule.first().x, capsule.first().y, capsule.first().z};
  double center2[3] = {capsule.second().x, capsule.second().y, capsule.second().z};
  double centerPoint[3] = {capsule.center_point().x, capsule.center_point().y, capsule.center_point().z};
  Eigen::Vector3d velocity_step = 0.2*Eigen::Vector3d(capsule.velocity().x, capsule.velocity().y, capsule.velocity().z)
                                  + Eigen::Vector3d(centerPoint[0], centerPoint[1], centerPoint[2]);
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
  ar::LineSegment obstacle_vel(centerPoint, velocity_step.data(), 0.005f);

  auto capsuleData = visData.find(capsule.id());
  // capsule was not drawn before
  if (capsuleData == visData.end())
  {
    ObstacleVisualizationData vd;
    vd.mh = vis_->Add(obstacle);
    vd.vh = vis_->Add(obstacle_vel);
    vd.lp = new ar::BufferedLinePath(10, 0.003f, ar::Color(1,1,1,1));
    vd.lp->addPoint(centerPoint);
    vd.th = vis_->Add(*vd.lp);
    visData.insert(std::make_pair(capsule.id(), vd));
  }
  // update capsule
  else
  {
    vis_->Update(capsuleData->second.mh, obstacle);
    vis_->Update(capsuleData->second.vh, obstacle_vel);
    capsuleData->second.lp->addPoint(centerPoint);
    vis_->Update(capsuleData->second.th, *(capsuleData->second.lp));
  }

  // add handle to visHandles
  seenObstacles.push_back(capsule.id());
}


const ar::Color lepp::SurfaceDrawer::colors[6] = {
  { 1.0f, 0.0f, 0.0f },
  { 0.0f, 1.0f, 0.0f },
  { 0.0f, 0.0f, 1.0f },
  { 1.0f, 1.0f, 0.0f },
  { 1.0f, 0.0f, 1.0f },
  { 0.0f, 1.0f, 1.0f },
};

void lepp::SurfaceDrawer::visitSurface(SurfaceModel &plane)
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
  ar::Polygon surfPoly(points, numPoints, colors[colorID]);

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


lepp::ObsSurfVisualizer::ObsSurfVisualizer(ObsSurfVisualizerParameters const& params = {})
  : BaseVisualizer(params.name, params.width, params.height),
    params_(params),
    pointCloudData(ar::PCL_PointXYZ),
    gridData(gridVector, gridSize, gridThickness, ar::Color( 0.5, 0.5, 0.5, 0.5 )),
    cosyX(cosy_o, cosy_x, 0.01f, ar::Color( 1.0, 0.0, 0.0 )),
    cosyY(cosy_o, cosy_y, 0.01f, ar::Color( 0.0, 1.0, 0.0 )),
    cosyZ(cosy_o, cosy_z, 0.01f, ar::Color( 0.0, 0.0, 1.0 )) {
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

  optionsWindow = arvis_->AddUIWindow("Options");
  gridCheckBox = optionsWindow->AddCheckBox("Draw grid", params.show_grid);
  obstaclesCheckBox = optionsWindow->AddCheckBox("Draw obstacles", params.show_obstacles);
  velocitiesCheckBox = optionsWindow->AddCheckBox("Draw velocities", params.show_obstacle_velocities);
  trajectoriesCheckBox = optionsWindow->AddCheckBox("Draw trajectories", params.show_obstacle_trajectories);
  surfacesCheckBox = optionsWindow->AddCheckBox("Draw surfaces", params.show_surfaces);
  obstacleCloudsCheckBox = optionsWindow->AddCheckBox("Draw obstacle clouds", params.show_obstacle_clouds);

  pccolorWindow = arvis_->AddUIWindow("Point Cloud");
  const char* colorOptions[4] = {"Grey", "Black", "White", "Skyblue"};
  setPCColor = pccolorWindow->AddComboBox("Preset", colorOptions, 4, 2);
  PCColorCheckBox = pccolorWindow->AddCheckBox("Edit", false);
  editPCColor = pccolorWindow->AddColorEdit4("Color", pccolorvalue);
}

void lepp::ObsSurfVisualizer::drawSurfaces(std::vector<SurfaceModelPtr> surfaces, std::vector<mesh_handle_t> &visHandles) {
  // create surface drawer object
  SurfaceDrawer sd(arvis_, visHandles);

  // draw surfaces
  for (size_t i = 0; i < surfaces.size(); i++)
    surfaces[i]->accept(sd);
}

void lepp::ObsSurfVisualizer::drawObstacles(std::vector<ObjectModelPtr> obstacles, std::map<int, ObstacleVisualizationData> &obsVisData) {
  // create model drawer object
  ModelDrawer md(arvis_, obsVisData);

  // draw obstacles
  for (size_t i = 0; i < obstacles.size(); i++)
    obstacles[i]->accept(md);
}

void lepp::ObsSurfVisualizer::outputFrameNum(FrameDataPtr frameData) {
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

void lepp::ObsSurfVisualizer::removeOldSurfObst(std::vector<mesh_handle_t> &visHandles) {
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

void lepp::ObsSurfVisualizer::updateFrame(FrameDataPtr frameData) {
  // visualize all obstacles and surfaces and store their handles
  std::vector<mesh_handle_t> visHandlesSurfaces;
  drawObstacles(frameData->obstacles, obsVisData);
  drawSurfaces(frameData->surfaces, visHandlesSurfaces);

  const bool show_obstacles = optionsWindow->GetCheckBoxState(obstaclesCheckBox);
  const bool show_velocities = optionsWindow->GetCheckBoxState(velocitiesCheckBox);
  const bool show_trajectories = optionsWindow->GetCheckBoxState(trajectoriesCheckBox);
  for (auto& o : obsVisData) {
    arvis_->SetVisibility(o.second.mh, show_obstacles);
    arvis_->SetVisibility(o.second.vh, show_velocities);
    arvis_->SetVisibility(o.second.th, show_trajectories);
  }


  const bool show_surfaces = optionsWindow->GetCheckBoxState(surfacesCheckBox);
  for (mesh_handle_t h : visHandlesSurfaces) {
    arvis_->SetVisibility(h, show_surfaces);
  }

  // Remove old obstacles and surfaces that are no longer visualized
  removeOldSurfObst(visHandlesSurfaces);

  // output frame num and surface frame num
  outputFrameNum(frameData);

  drawObstacleClouds(frameData->obstacleParams);

  // visualize the point cloud
  pointCloudData.pointData = static_cast<const void*>(&(frameData->cloud->points[0]));
  pointCloudData.numPoints = frameData->cloud->size();
  if (pccolorWindow->GetCheckBoxState(PCColorCheckBox)) {
    pccolorWindow->GetColorValues4(editPCColor, pccolorvalue);
  } else  {
    if (presetpccolor != pccolorWindow->GetSelectedComboBoxItem(setPCColor)) {
      presetpccolor = pccolorWindow->GetSelectedComboBoxItem(setPCColor);
      switch(presetpccolor) {
        case 0:
          pccolorvalue[0] = 0.197;
          pccolorvalue[1] = 0.197;
          pccolorvalue[2] = 0.197;
          break;
        case 1:
          pccolorvalue[0] = 0.0;
          pccolorvalue[1] = 0.0;
          pccolorvalue[2] = 0.0;
          break;
        case 2:
          pccolorvalue[0] = 1.0;
          pccolorvalue[1] = 1.0;
          pccolorvalue[2] = 1.0;
          break;
        case 3:
          pccolorvalue[0] = 0.529;
          pccolorvalue[1] = 0.808;
          pccolorvalue[2] = 0.980;
          break;
      }
    }
  }
  pointCloudData.color = ar::Color(pccolorvalue[0], pccolorvalue[1], pccolorvalue[2], pccolorvalue[3]);
  arvis_->Update(pointCloudHandle, pointCloudData);
  arvis_->SetVisibility(gridHandle, optionsWindow->GetCheckBoxState(gridCheckBox));
}

void lepp::ObsSurfVisualizer::updateFrame(RGBDataPtr rgbData) {
  return;
}

void lepp::ObsSurfVisualizer::drawObstacleClouds(std::vector<ObjectModelParams> const& obstacles) {
  const bool show_clouds = optionsWindow->GetCheckBoxState(obstacleCloudsCheckBox);

  if (!show_clouds) {
    for (auto h : obstacleCloudHandles) {
      arvis_->Remove(h);
    }
    obstacleCloudHandles.clear();
    obstacleCloudData.clear();
    return;
  }

  assert(obstacleCloudHandles.size() == obstacleCloudData.size());

  for (size_t idx = 0; idx < obstacles.size(); ++idx) {
    assert(obstacleCloudHandles.size() >= idx);
    if (obstacleCloudHandles.size() == idx) {
      // We derive the color from the HSV color space (in 30° steps)
      // This should lead to easily distinguishable colors
      float h = (idx * 30) % 360;
      ar::Color color = HsvToRgbColor(h, 0.7f, 0.5f);

      obstacleCloudData.emplace_back(new ar::PointCloudData(ar::PCL_PointXYZ, color));
      obstacleCloudHandles.push_back(arvis_->Add(*obstacleCloudData.back()));
    }

    auto& cloudData = *obstacleCloudData[idx];
    cloudData.pointData = &(obstacles[idx].obstacleCloud->points[0]);
    cloudData.numPoints = obstacles[idx].obstacleCloud->size();
    arvis_->Update(obstacleCloudHandles[idx], cloudData);
  }

  // remove unused
  if (obstacles.size() < obstacleCloudHandles.size()) {
    for (size_t i = obstacles.size(); i < obstacleCloudHandles.size(); ++i) {
      arvis_->Remove(obstacleCloudHandles[i]);
    }
    obstacleCloudHandles.erase(std::begin(obstacleCloudHandles) + obstacles.size(), std::end(obstacleCloudHandles));
    obstacleCloudData.erase(std::begin(obstacleCloudData) + obstacles.size(), std::end(obstacleCloudData));
  }
}
