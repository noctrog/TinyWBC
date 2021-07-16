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
void HyqWorldNode::customPreStep()
{
  auto base = mController->getRobotSkeleton()->getBodyNode("trunk");
  base->addExtForce(mExternalForce);

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
