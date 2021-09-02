# Introduction

TinyWBC is a simle implementation of task inverse dynamics. It uses 
[Pinocchio](https://github.com/stack-of-tasks/pinocchio) for the computation of
the kinematic and dynamic magnitudes, and the optimization is done by the
[OSQP](https://github.com/osqp/osqp) solver (using the 
[osqp-eigen](https://github.com/robotology/osqp-eigen) interface).

All the tasks and constraints are modular and can be activated and de-activaded
in real time. Due to the simplicity of the code, it is very straight forward to
add new tasks and constraints.

# Installation

You have to install all the required dependencies:

 - [Pinocchio](https://stack-of-tasks.github.io/pinocchio/download.html)
 - [OSQP](https://osqp.org/docs/installation/index.html)
 - [osqp-eigen](https://github.com/robotology/osqp-eigen#%EF%B8%8F-build-from-source-advanced)

The included example makes use of [Dart](https://dartsim.github.io/) for the
simulation. In order to build the example you have to install it.

# Usage

You can find an example of usage in `examples/hyq/Controller.cc`. In summary:

## Instantiate a controller

```cpp
std::string urdf_path("/path/to/urdf");
TinyWBC wbc(urdf_path);
```

## Constraints

### Activate constraints

```cpp
wbc.SetConstraint(TinyWBC::ConstraintName::EQUATION_OF_MOTION);
wbc.SetConstraint(TinyWBC::ConstraintName::FIXED_CONTACT_CONDITION);
wbc.SetConstraint(TinyWBC::ConstraintName::ACTUATION_LIMITS);
wbc.SetConstraint(TinyWBC::ConstraintName::CONTACT_STABILITY);
```

### Deactivate constraints

```cpp
wbc.EraseConstraint(TinyWBC::ConstraintName::EQUATION_OF_MOTION);
wbc.EraseConstraint(TinyWBC::ConstraintName::FIXED_CONTACT_CONDITION);
wbc.EraseConstraint(TinyWBC::ConstraintName::ACTUATION_LIMITS);
wbc.EraseConstraint(TinyWBC::ConstraintName::CONTACT_STABILITY);
```

### Deactivate all constraints

```cpp
wbc.ClearConstraints();
```

## Tasks

### Activate each task

```cpp
wbc.SetTask(TinyWBC::TaskName::FOLLOW_JOINT);
wbc.SetTask(TinyWBC::TaskName::FOLLOW_COM);
wbc.SetTask(TinyWBC::TaskName::FOLLOW_ORIENTATION);
```

### Deactivate a task

```cpp
wbc.EraseTask(TinyWBC::TaskName::FOLLOW_JOINT);
wbc.EraseTask(TinyWBC::TaskName::FOLLOW_COM);
wbc.EraseTask(TinyWBC::TaskName::FOLLOW_ORIENTATION);
```

### Deactivate all tasks

```cpp
wbc.ClearTasks();
```

### Set the tasks weights and dynamics

Sets the second order dynamic constants for each task.

```cpp
wbc.SetTaskDynamics(TinyWBC::TaskName::FOLLOW_JOINT, 40000.0, 200.0);
wbc.SetTaskDynamics(TinyWBC::TaskName::FOLLOW_COM,   20000.0, 200.0);
wbc.SetTaskDynamics(TinyWBC::TaskName::FOLLOW_ORIENTATION, 3000.0, 50.0);
```

## Set the current robot state

```cpp
std::vector<std::string> frames_in_contact =
                  {"lf_foot", "rf_foot", "lh_foot", "rh_foot"};

wbc.SetRobotState(base_pos, base_vel,
                  spatial_pos, spatial_vel,
		  frames_in_contact);
```

Where `base_pos` and `base_vel` are the base's position and velocity in the
WORLD frame of reference, and `spatial_pos` and `spatial_vel` represent the
position and velocity of the rest degree's of freedom.

## Foot friction

```cpp
double mu = 0.4;
wbc.SetFrictionCoefficient(mu);
```

## Actuation limits

Receives a vector indicating the maximum torque of each actuable degree of
freedom.

```cpp
wbc.SetActuationLimits(Eigen::VectorXd::Constant(12, 1000.0));
```

## Set the desired state

### Set the desired pose

```cpp
wbc.SetDesiredPosture(Eigen::VectorXd::Constant(nJoints, 0.0),
			Eigen::VectorXd::Constant(nJoints, 0.0),
			Eigen::VectorXd::Constant(nJoints, 0.0));
```

### Set the desired CoM

```cpp
wbc.SetDesiredCoM(Eigen::Vector3d(0.0, 0.0, 0.0));
```

### Set the desired frame orientation

```cpp
wbc.SetDesiredFrameOrientation("frame_name",
		{0.0, 0.0, 0.0},
		{0.0, 0.0, 0.0});
```

## Build and solve the problem

The solution to each solved problem is a vector with each DoF acceleration,
external contact forces and each DoF torques, in that order.

```cpp
mWBC->BuildProblem();
mWBC->SolveProblem();
const Eigen::VectorXd sol = wbc.GetSolution();
```
