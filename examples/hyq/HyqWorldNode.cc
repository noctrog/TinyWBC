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

	// TODO: Update the controller tasks

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

	ImGui::Text("An empty OSG example with ImGui");
	ImGui::Spacing();

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
