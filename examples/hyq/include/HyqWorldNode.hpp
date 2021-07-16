#ifndef HYQWORLDNODE_H
#define HYQWORLDNODE_H

#include <dart/dart.hpp>
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

protected:
  std::unique_ptr<Controller> mController;
  Eigen::Vector3d mExternalForce;
  int mForceDuration;
};

#endif /* HYQWORLDNODE_H */
