#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <Eigen/Dense>

#include <dart/dart.hpp>
#include <dart/dynamics/SmartPointer.hpp>

class Controller
{
public:
  /// \brief Constructor
  Controller(dart::dynamics::SkeletonPtr _robot);
  
  /// \brief Destructor
  virtual ~Controller(void);

  /// \brief Called before every simulation step in MyWindow class.
  /// Computes the control forces and applies them to the robot.
  virtual void update(void);

  /// \brief Returns the robot skeleton
  dart::dynamics::SkeletonPtr getRobotSkeleton(void);

  /// \brief Resets the robot
  void resetRobot(void);

protected:
  /// \brief The robot skeleton
  dart::dynamics::SkeletonPtr mRobot;

private:
  /// \brief Initial state of the robot
  dart::dynamics::Skeleton::Configuration mInitialState;
};

#endif /* CONTROLLER_H */
