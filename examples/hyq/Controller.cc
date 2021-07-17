#include <Controller.hpp>
#include <dart/dynamics/SmartPointer.hpp>

#include <tiny_wbc.hpp>

//==============================================================================
Controller::Controller(dart::dynamics::SkeletonPtr _robot)
  : mRobot(_robot)
{
  // TODO: more elegant solution
  const std::string urdf_path("/home/ramon/Documents/programming/tiny_wbc/examples/hyq/hyq_urdf/urdf/hyq.urdf");

  // Load the robot model to the whole body controller
  mWBC.reset(new TinyWBC(urdf_path));

  // Activate the tasks
  mWBC->PushTask(TinyWBC::TaskName::FOLLOW_JOINT);
  mWBC->PushTask(TinyWBC::TaskName::FOLLOW_COM);

  // Set the tasks priorities
  mWBC->SetTaskWeight(TinyWBC::TaskName::FOLLOW_JOINT, 0.4);
  mWBC->SetTaskWeight(TinyWBC::TaskName::FOLLOW_COM,   0.6);

  // Set the tasks dynamic properties
  mWBC->SetTaskDynamics(TinyWBC::TaskName::FOLLOW_JOINT, 20000.0, 200.0);
  mWBC->SetTaskDynamics(TinyWBC::TaskName::FOLLOW_COM,   30000.0, 300.0);

  // Enable all the constraints
  mWBC->PushConstraint(TinyWBC::ConstraintName::EQUATION_OF_MOTION);
  mWBC->PushConstraint(TinyWBC::ConstraintName::FIXED_CONTACT_CONDITION);
  mWBC->PushConstraint(TinyWBC::ConstraintName::ACTUATION_LIMITS);
  mWBC->PushConstraint(TinyWBC::ConstraintName::CONTACT_STABILITY);
}

//==============================================================================
Controller::~Controller()
{
  
}

//==============================================================================
void
Controller::update()
{
  const int nJoints = mRobot->getNumJoints();
  
  // Retrieve the robot state
  // TODO: Retrieve contacts
  const Eigen::Isometry3d robot_base_tf = mRobot->getRootBodyNode()->getTransform();
  const Eigen::Quaterniond robot_base_quat(robot_base_tf.rotation());
  const Eigen::VectorXd robot_spatial_pos = mRobot->getPositions();
  const Eigen::Vector6d robot_spatial_vel = mRobot->getRootBodyNode()->getSpatialVelocity();
  const Eigen::Vector3d robot_com_pos = mRobot->getCOM();
  const Eigen::Vector3d robot_com_vel = mRobot->getCOMLinearVelocity();

  // Send the current state to the controller
  // TODO: Send contacts
  Eigen::VectorXd pin_spatial_pos(7);
  Eigen::Vector6d pin_spatial_vel;
  pin_spatial_pos << robot_spatial_pos.segment(3, 3), robot_base_quat.x(),
    robot_base_quat.y(), robot_base_quat.z(), robot_base_quat.w();
  pin_spatial_vel << robot_spatial_vel.segment(3, 3), robot_spatial_vel.head(3);

  mWBC->SetRobotState(pin_spatial_pos,
		      pin_spatial_vel,
		      robot_spatial_pos.tail(12),
		      robot_spatial_vel.tail(12),
		      {});

  // TODO: Send the desired state to the controller

  // TODO: Build and solve problem

  // TODO: Apply the obtained torques
  
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
