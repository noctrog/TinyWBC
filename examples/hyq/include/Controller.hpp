#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <Eigen/Dense>

#include <dart/dart.hpp>
#include <dart/dynamics/SmartPointer.hpp>

class TinyWBC;
using TinyWBCPtr = std::shared_ptr<TinyWBC>;

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

	void setPostureTaskActive(bool);
	void setComTaskActive(bool);
	void setOrientationTaskActive(bool);
	void setPostureTaskConstant(float);
	void setComTaskConstant(float);
	void setOrientationTaskConstant(float);
	void setPostureTaskWeight(float);
	void setComTaskWeight(float);
	void setOrientationTaskWeight(float);

protected:
  /// \brief The robot skeleton
  dart::dynamics::SkeletonPtr mRobot;

private:
  /// \brief Initial state of the robot
  dart::dynamics::Skeleton::Configuration mInitialState;

  /// \brief TinyWBC instance
  TinyWBCPtr mWBC; 

	/// \brief Active tasks
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

#endif /* CONTROLLER_H */
