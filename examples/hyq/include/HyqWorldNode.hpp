#ifndef HYQWORLDNODE_H
#define HYQWORLDNODE_H

#include <dart/dart.hpp>
#include <dart/external/imgui/imgui.h>
#include <dart/gui/osg/osg.hpp>
#include <dart/utils/utils.hpp>

#include <Controller.hpp>

class HyqWorldNode : public dart::gui::osg::RealTimeWorldNode
{
public:
  /// Constructor
  HyqWorldNode(
      const dart::simulation::WorldPtr& world,
      const dart::dynamics::SkeletonPtr& hyq);

  // Documentation inherited
  void customPreStep() override;

  void reset();

  void pushForwardHyq(double force = 500, int frames = 100);
  void pushBackwardHyq(double force = 500, int frames = 100);
  void pushLeftHyq(double force = 500, int frames = 100);
  void pushRightHyq(double force = 500, int frames = 100);

  void showShadow();
  void hideShadow();

	std::shared_ptr<Controller> getController(void);

protected:
  std::shared_ptr<Controller> mController;
  Eigen::Vector3d mExternalForce;
  int mForceDuration;
};

class ControllerWidget : public dart::gui::osg::ImGuiWidget
{
	public:
		ControllerWidget(
				dart::gui::osg::ImGuiViewer* viewer, dart::simulation::WorldPtr world,
				std::shared_ptr<Controller> controller);

		void render(void) override;

	private:
		void setGravity(bool gravity);
		void setControllerState(void);

		// Simulation
		osg::ref_ptr<dart::gui::osg::ImGuiViewer> mViewer;
		dart::simulation::WorldPtr mWorld;
		bool mGuiGravity;
		bool mGravity;
		bool mGuiHeadlights;

		// Controller
		std::shared_ptr<Controller> mController;
		// Controller Tasks
		bool mbPostureTask;
		bool mbComTask;
		bool mbOrientationTask;
		float mPostureConstant;
		float mComConstant;
		float mOrientationConstant;
		float mPostureWeight;
		float mComWeight;
		float mOrientationWeight;
};

#endif /* HYQWORLDNODE_H */
