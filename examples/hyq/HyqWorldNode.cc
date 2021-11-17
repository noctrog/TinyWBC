#include <HyqWorldNode.hpp>

#include <osgShadow/ShadowMap>

//==============================================================================
HyqWorldNode::HyqWorldNode(
    const dart::simulation::WorldPtr& world,
    const dart::dynamics::SkeletonPtr& hyq)
  : dart::gui::osg::RealTimeWorldNode(world),
  mExternalForce(Eigen::Vector3d::Zero()),
  mForceDuration(0.0)
{
  assert(world);
  assert(hyq);

  // Add the desired center of mass ball visual to the world
  desired_com_pos_ = Eigen::Vector3d(0.0, 0.0,
      -0.1 + 0.05 * std::cos(dart::common::Timer::getWallTime()));
  visual_desired_com_ = std::make_shared<VisualBall>(world, 0.05,
      desired_com_pos_);

  mController.reset(new Controller(hyq));
}

//==============================================================================
  std::shared_ptr<Controller>
HyqWorldNode::getController()
{
  return mController;
}

//==============================================================================
void HyqWorldNode::customPreStep()
{
  // Apply external force
  auto base = mController->getRobotSkeleton()->getBodyNode("trunk");
  base->addExtForce(mExternalForce);

  // Update desired center of mass
  desired_com_pos_ = Eigen::Vector3d(0.0, 0.0,
      -0.1 + 0.05 * std::cos(dart::common::Timer::getWallTime()));
  visual_desired_com_->setPosition(desired_com_pos_);
  mController->setDesiredCom(desired_com_pos_);

  // Update desired base frame rotation
  Eigen::Vector3d base_rot_des = Eigen::Vector3d(std::cos(dart::common::Timer::getWallTime()), 0.0, 0.0);
  mController->setDesiredBaseRot(base_rot_des, {0.0, 0.0, 0.0});

  // TODO: Get the robot's feet collisions with the floor
  auto collision_engine =
    mWorld->getConstraintSolver()->getCollisionDetector();

  mController->update();
}

//==============================================================================
void HyqWorldNode::reset()
{
  mExternalForce.setZero();
  mController->resetRobot();
}

//==============================================================================
void HyqWorldNode::pushForwardHyq(double force, int frames)
{
  mExternalForce.x() = force;
  mForceDuration = frames;
}

//==============================================================================
void HyqWorldNode::pushBackwardHyq(double force, int frames)
{
  mExternalForce.x() = -force;
  mForceDuration = frames;
}

//==============================================================================
void HyqWorldNode::pushLeftHyq(double force, int frames)
{
  mExternalForce.z() = force;
  mForceDuration = frames;
}

//==============================================================================
void HyqWorldNode::pushRightHyq(double force, int frames)
{
  mExternalForce.z() = -force;
  mForceDuration = frames;
}

//==============================================================================
void HyqWorldNode::showShadow()
{
  auto shadow
    = dart::gui::osg::WorldNode::createDefaultShadowTechnique(mViewer);
  if (auto sm = dynamic_cast<::osgShadow::ShadowMap*>(shadow.get()))
  {
    auto mapResolution = static_cast<short>(std::pow(2, 12));
    sm->setTextureSize(::osg::Vec2s(mapResolution, mapResolution));
  }

  setShadowTechnique(shadow);
}

//==============================================================================
void HyqWorldNode::hideShadow()
{
  setShadowTechnique(nullptr);
}

//==============================================================================
//========================= Controller Widget ==================================
//==============================================================================
ControllerWidget::ControllerWidget(
    dart::gui::osg::ImGuiViewer* viewer, dart::simulation::WorldPtr world,
    std::shared_ptr<Controller> controller)
  : mViewer(viewer),
  mWorld(std::move(world)),
  mController(controller),
  mGuiGravity(true),
  mGuiHeadlights(true),
  mGravity(true),
  mbEquationOfMotion(true),
  mbFixedContactCondition(true),
  mbContactStability(true),
  mbActuationLimits(true),
  mbPostureTask(true),
  mbComTask(false),
  mbOrientationTask(false),
  mPostureWeight(0.3),
  mComWeight(0.4),
  mOrientationWeight(0.3),
  mPostureConstant(40000.0),
  mComConstant(30000.0),
  mOrientationConstant(20000.0)
{

}

//==============================================================================
  void
ControllerWidget::render()
{
  ImGui::SetNextWindowPos(ImVec2(10, 20));
  ImGui::SetWindowSize(ImVec2(240, 520));
  ImGui::SetNextWindowBgAlpha(0.5);

  // Create an Imgui Window
  if (!ImGui::Begin("ToyWidget Control",
        nullptr,
        ImGuiWindowFlags_NoResize | ImGuiWindowFlags_MenuBar |
        ImGuiWindowFlags_HorizontalScrollbar))
  {
    ImGui::End();
    return;
  }

  // Menu
  if (ImGui::BeginMenuBar()) {
    if (ImGui::BeginMenu("Menu")) {
      if (ImGui::MenuItem("Exit")) {
        mViewer->setDone(true);
      }
      ImGui::EndMenu();
    }
    if (ImGui::BeginMenu("Help")) {
      if (ImGui::MenuItem("About Dart")) {
        mViewer->showAbout();
      }
      ImGui::EndMenu();
    }
    ImGui::EndMainMenuBar();
  }

  if (ImGui::CollapsingHeader("Simulation"))
  {
    int e = mViewer->isSimulating() ? 0 : 1;
    if (mViewer->isAllowingSimulation())
    {
      if (ImGui::RadioButton("Play", &e, 0) && !mViewer->isSimulating())
        mViewer->simulate(true);
      ImGui::SameLine();
      if (ImGui::RadioButton("Pause", &e, 1) && mViewer->isSimulating())
        mViewer->simulate(false);
    }

    ImGui::Text("Time: %.3f", mWorld->getTime());
  }

  // Control the controller's parameters
  if (ImGui::CollapsingHeader("Controller", ImGuiTreeNodeFlags_DefaultOpen)) {
    // Enable or disable constraints
    ImGui::Text("Constraints");
    ImGui::Checkbox("Equation of motion", &mbEquationOfMotion);
    ImGui::Checkbox("Fixed contact condition", &mbFixedContactCondition);
    ImGui::Checkbox("Actuation limits", &mbActuationLimits);
    ImGui::Checkbox("Contact stability", &mbContactStability);
    // Enable or disable tasks
    ImGui::Text("Tasks");
    ImGui::Checkbox("Posture task", &mbPostureTask);
    ImGui::Checkbox("Com task", &mbComTask);
    ImGui::Checkbox("Orientation task", &mbOrientationTask);
    ImGui::Spacing();
    // Choose tasks weights
    ImGui::Text("Task Weight");
    ImGui::SliderFloat("Posture W", &mPostureWeight, 0.0, 1.0);
    ImGui::SliderFloat("Com W", &mComWeight, 0.0, 1.0);
    ImGui::SliderFloat("Orientation W", &mOrientationWeight, 0.0, 1.0);
    ImGui::Spacing();
    // Choose tasks dynamic constants
    ImGui::Text("Task Dynamics");
    ImGui::SliderFloat("Posture K", &mPostureConstant, 0.0, 100000.0);
    ImGui::SliderFloat("Com K", &mComConstant, 0.0, 100000.0);
    ImGui::SliderFloat("Orientation K", &mOrientationConstant, 0.0, 100000.0);
    ImGui::Spacing();
    // Update Controller
    setControllerState();
  }

  if (ImGui::CollapsingHeader(
        "World Options"))
  {
    // Gravity
    ImGui::Checkbox("Gravity On/Off", &mGuiGravity);
    setGravity(mGuiGravity);

    ImGui::Spacing();

    // Headlights
    mGuiHeadlights = mViewer->checkHeadlights();
    ImGui::Checkbox("Headlights On/Off", &mGuiHeadlights);
    mViewer->switchHeadlights(mGuiHeadlights);
  }

  if (ImGui::CollapsingHeader("View"))
  {
    osg::Vec3d eye;
    osg::Vec3d center;
    osg::Vec3d up;
    mViewer->getCamera()->getViewMatrixAsLookAt(eye, center, up);

    ImGui::Text("Eye   : (%.2f, %.2f, %.2f)", eye.x(), eye.y(), eye.z());
    ImGui::Text(
        "Center: (%.2f, %.2f, %.2f)", center.x(), center.y(), center.z());
    ImGui::Text("Up    : (%.2f, %.2f, %.2f)", up.x(), up.y(), up.z());
  }

  if (ImGui::CollapsingHeader("Help"))
  {
    ImGui::PushTextWrapPos(ImGui::GetCursorPos().x + 320);
    ImGui::Text("User Guide:\n");
    ImGui::Text("%s", mViewer->getInstructions().c_str());
    ImGui::PopTextWrapPos();
  }

  ImGui::End();

}

//==============================================================================
  void 
ControllerWidget::setGravity(bool gravity)
{
  if (mGravity == gravity)
    return;

  mGravity = gravity;

  if (mGravity)
    mWorld->setGravity(-9.81 * Eigen::Vector3d::UnitZ());
  else
    mWorld->setGravity(Eigen::Vector3d::Zero());
}

//==============================================================================
  void
ControllerWidget::setControllerState()
{
  mController->setEquationOfMotionConstraint(mbEquationOfMotion);
  mController->setFixedContactConditionConstraint(mbFixedContactCondition);
  mController->setActuationLimitsConstraint(mbActuationLimits);
  mController->setContactStabilityConstraint(mbContactStability);

  mController->setPostureTaskActive(mbPostureTask);
  mController->setComTaskActive(mbComTask);
  mController->setOrientationTaskActive(mbOrientationTask);

  mController->setPostureTaskConstant(mPostureConstant);
  mController->setComTaskConstant(mComConstant);
  mController->setOrientationTaskConstant(mOrientationConstant);

  mController->setPostureTaskWeight(mPostureWeight);
  mController->setComTaskWeight(mComWeight);
  mController->setOrientationTaskWeight(mOrientationWeight);
}

//==============================================================================
//============================== Visual Ball ===================================
//==============================================================================
VisualBall::VisualBall(dart::simulation::WorldPtr world, 
    double radius, const Eigen::Vector3d& pos,
    Eigen::Vector4d color)
{
  // Create skeleton
  skel = dart::dynamics::Skeleton::create();
  // Link skeleton with respect to the world and retrieve the ball body
  const auto ball_and_world 
    = skel->createJointAndBodyNodePair<dart::dynamics::WeldJoint>();
  ball_body = ball_and_world.second;
  // Set the sphere shape to the ball body
  const auto ball_shape = std::make_shared<dart::dynamics::SphereShape>(radius);
  ball_body->createShapeNodeWith<dart::dynamics::VisualAspect>(ball_shape);

  // Set the ball position
  setPosition(pos);

  // Set the ball color
  ball_body->getShapeNode(0)->getVisualAspect()->setColor(color);

  // Add the skeleton to the world
  world->addSkeleton(skel);
}

//==============================================================================
VisualBall::~VisualBall()
{

}

//==============================================================================
void 
VisualBall::setPosition(const Eigen::Vector3d& pos)
{
  Eigen::Isometry3d tf(Eigen::Isometry3d::Identity());
  tf.translation() = pos;
  ball_body->getParentJoint()->setTransformFromParentBodyNode(tf);
}
