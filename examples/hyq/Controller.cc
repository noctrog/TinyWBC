#include <boost/dll/runtime_symbol_info.hpp>
#include <boost/filesystem.hpp>
#include <boost/dll.hpp>

#include <Controller.hpp>
#include <boost/filesystem/directory.hpp>
#include <dart/dynamics/SmartPointer.hpp>

#include <tiny_wbc.hpp>

#include <configure.h>

//==============================================================================
Controller::Controller(dart::dynamics::SkeletonPtr _robot)
  : mRobot(_robot)
{
  // Load the robot model
  std::string urdf_path;
  if (boost::filesystem::exists(EXAMPLE_RESOURCE_DIR "/urdf/hyq.urdf"))
    urdf_path = EXAMPLE_RESOURCE_DIR "/urdf/hyq.urdf";
  else if (boost::filesystem::exists(EXAMPLE_RESOURCE_INSTALL_DIR "/urdf/hyq.urdf"))
    urdf_path = EXAMPLE_RESOURCE_INSTALL_DIR "/urdf/hyq.urdf";
  else
    urdf_path = boost::dll::program_location().parent_path().string() + "/urdf/hyq.urdf";

  // Load the robot model to the whole body controller
  mWBC.reset(new TinyWBC(urdf_path));

  // Activate the tasks
  mWBC->PushTask(TinyWBC::TaskName::FOLLOW_JOINT);
  // mWBC->PushTask(TinyWBC::TaskName::FOLLOW_COM);

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

  // Set the actuation limits
  mWBC->SetActuationLimits(Eigen::VectorXd::Constant(12, 1000.0));

  // Set the friction coefficient
  mWBC->SetFrictionCoefficient(0.4);

  // Initialize and save the robot initial state
  mInitialState = mRobot->getConfiguration(dart::dynamics::Skeleton::CONFIG_POSITIONS |
					   dart::dynamics::Skeleton::CONFIG_VELOCITIES);
  mInitialState.mPositions << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    -0.14213842558668383, 0.8222834967484725, -1.446773610100507,
    -0.14213842558668383, -0.8222834967484725, 1.446773610100507,
    -0.14213842558668383, 0.8222834967484725, -1.446773610100507,
    -0.14213842558668383, -0.8222834967484725, 1.446773610100507;

  resetRobot();
}

//==============================================================================
Controller::~Controller()
{
  
}

//==============================================================================
void
Controller::update()
{
  const int nJoints = mRobot->getNumJoints() - 6;

  // Retrieve the robot state
  // TODO: Retrieve contacts
  const Eigen::Isometry3d robot_base_tf = mRobot->getRootBodyNode()->getTransform();
  const Eigen::Quaterniond robot_base_quat(robot_base_tf.rotation());
  const Eigen::VectorXd robot_spatial_pos = mRobot->getPositions();
  const Eigen::VectorXd robot_spatial_vel = mRobot->getVelocities();

  // Send the current state to the controller
  // TODO: Send contacts
  Eigen::VectorXd pin_spatial_pos(7);
  Eigen::Vector6d pin_spatial_vel;
  pin_spatial_pos << robot_spatial_pos.segment(3, 3), robot_base_quat.x(),
    robot_base_quat.y(), robot_base_quat.z(), robot_base_quat.w();
  pin_spatial_vel << robot_spatial_vel.segment(3, 3), robot_spatial_vel.head(3);

  mWBC->SetRobotState(pin_spatial_pos,
		      pin_spatial_vel,
		      robot_spatial_pos.tail(nJoints),
		      robot_spatial_vel.tail(nJoints),
		      {"lf_foot", "rf_foot", "lh_foot", "rh_foot"});

  // Send the desired state to the controller
  mWBC->SetDesiredPosture(mInitialState.mPositions.tail(nJoints),
			  Eigen::VectorXd::Constant(nJoints, 0.0),
			  Eigen::VectorXd::Constant(nJoints, 0.0));
  mWBC->SetDesiredCoM(Eigen::Vector3d(0.0, 0.0, 0.4));

  // Build and solve problem
  mWBC->BuildProblem();
  mWBC->SolveProblem();
  const Eigen::VectorXd sol = mWBC->GetSolution();

  // Apply the obtained torques
  Eigen::VectorXd forces(mRobot->getNumJoints());
  forces << Eigen::VectorXd::Constant(6, 0.0), sol.tail(nJoints);
  mRobot->setForces(forces);
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
