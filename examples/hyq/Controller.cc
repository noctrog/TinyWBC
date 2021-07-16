#include <Controller.hpp>
#include <dart/dynamics/SmartPointer.hpp>

//==============================================================================
Controller::Controller(dart::dynamics::SkeletonPtr _robot)
  : mRobot(_robot)
{
  
}

//==============================================================================
Controller::~Controller()
{
  
}

//==============================================================================
void
Controller::update()
{
  // TODO: Compute control force

}

//==============================================================================
dart::dynamics::SkeletonPtr
Controller::getRobotSkeleton()
{
  return mRobot;
}

//==============================================================================
void
Controller::resetRobot()
{
  mRobot->setConfiguration(mInitialState);
}
