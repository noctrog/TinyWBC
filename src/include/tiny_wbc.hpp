#ifndef TINY_WBC_H
#define TINY_WBC_H

#include <dart/math/MathTypes.hpp>
#include <vector>
#include <set>
#include <array>
#include <memory>

#include <OsqpEigen/Solver.hpp>

#include <pinocchio/fwd.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>

class TinyWBC;
using TinyWBCPtr = std::shared_ptr<TinyWBC>;

class TinyWBC
{
  public:

    enum class TaskName {
      FOLLOW_JOINT = 0,
      FOLLOW_COM,
      FOLLOW_ORIENTATION,
      TOTAL_TASKS
    };

    enum class ConstraintName {
      EQUATION_OF_MOTION = 0,
      FIXED_CONTACT_CONDITION,
      ACTUATION_LIMITS,
      CONTACT_STABILITY,
      TOTAL_CONSTRAINTS
    };

    // QP formulation cost weight
    typedef double Weight;
    typedef std::array<Weight, static_cast<size_t>(TaskName::TOTAL_TASKS)> WeightList;
    // Tasks and constraints
    typedef std::set<TaskName> TaskList;
    struct TaskDynamics {
      double Kp, Kv;
    };
    typedef std::array<TaskDynamics, static_cast<size_t>(TaskName::TOTAL_TASKS)> TaskDynamicsList;
    typedef std::set<ConstraintName> ConstraintList;
    // Robot state
    typedef Eigen::VectorXd SpatialPos;
    typedef Eigen::Vector6d SpatialVel;
    typedef Eigen::VectorXd JointPos;
    typedef Eigen::VectorXd JointVel;
    typedef Eigen::VectorXd JointAcc;
    typedef Eigen::VectorXd JointForce;
    typedef Eigen::Vector3d ComPos;
    typedef Eigen::Vector3d ComVel;
    typedef Eigen::Vector3d ComAcc;
    typedef Eigen::Vector3d FrameRot;
    typedef Eigen::Vector3d FrameAngVel;
    // Robot feedback
    typedef Eigen::VectorXd PosErrors;
    typedef Eigen::VectorXd VelErrors;
    // Contact names and jacobians
    typedef std::vector<Eigen::MatrixXd, Eigen::aligned_allocator<Eigen::MatrixXd>> ContactJacobians;
    // Robot representation
    typedef pinocchio::Model Model;
    typedef pinocchio::Data Data;
    typedef std::shared_ptr<Model> ModelPtr;
    typedef std::shared_ptr<Data> DataPtr;
    typedef std::string FrameName;
    typedef std::vector<FrameName> FrameNameList;
    // Contacts
    typedef FrameName ContactName;
    typedef FrameNameList ContactNameList;
    typedef Eigen::Matrix3d ContactOrientation;
    typedef std::vector<ContactOrientation> ContactOrientationList;
    struct ContactFamily {
      ContactName family_name;
      ContactNameList contact_names;

      ContactFamily(const ContactName& fname,
          const ContactNameList& cn) {
        family_name = fname;
        contact_names = cn;
      }
    };
    typedef std::vector<ContactFamily> ContactFamilyList;
    struct Contact {
      ContactName contact_name;
      ContactOrientation contact_orientation;
      std::vector<int> contact_frames_ids;
    };
    typedef std::vector<Contact> ContactList;

    typedef int FrameId;
    struct OrientationState {
      Eigen::Vector3d orientation;
      Eigen::Vector3d angular_vel;
    };
    // typedef std::map<FrameId, OrientationState, std::less<FrameId>,
    // 		   Eigen::aligned_allocator<std::pair<const FrameId, OrientationState>>> DesiredOrientations;
    typedef std::map<FrameId, OrientationState> DesiredOrientations;

    TinyWBC(const std::string& urdf_path);
    virtual ~TinyWBC() = default;

    /**
     * Updates the robot state. It automatically computes the inertia,
     * nonlinear, stacked contact jacobian and stacked contact jacobian
     * time derivative.
     *
     * @param base_pos: position of the base link with respect to
     * WORLD. 7 dimensions: 3 for linear position and 4 for orientation
     * (quaternion).
     *
     * @param base_vel: velocity of the base link with respect to
     * WORLD. 6 dimensions: 3 for linear velocity and 3 for angular
     * velocity.
     *
     * @param q: joint positions.
     *
     * @param qd: joint velocities.
     *
     * @param contact_names: list of the names of the links or contact
     * families (see @ref SetContactFamily) that are currently active.
     *
     * @param contact_orientations: list of 3x3 rotation matrices with
     * respect to WORLD that represent the orientation of each
     * unilateral contact. X and Y cols define the contact plane and Z
     * corresponds to the normal of the plane. X and Y can be any vector
     * as long as they are orthogonal and are inside the contact
     * plane. The default is a XY plane.
     *
     */
    void SetRobotState(const SpatialPos& base_pos, const SpatialVel& base_vel,
        const JointPos& q, const JointVel& qd,
        const ContactNameList& contact_names,
        const ContactOrientationList& contact_orientations = {});

    /**
     * @brief Sets the limits of all of the actuators.
     *
     * @param limits: vector of absolute maximum forces for each motor.
     *
     */
    void SetActuationLimits(const JointForce&);

    /**
     * @brief Defines a contact family. The newly contact family
     * persists.
     *
     * @param family_name: Name that will be used to activate the
     * contact family.
     *
     * @param contact_link_names: The names of the links to include in
     * the contact. This names must be defined in the previously loaded
     * URDF model of the robot.
     *
     * Set a contact family. With a contact family, the corresponding
     * set of contacts will become active automatically when the contact
     * family is activated. This is useful to model contacts that are
     * surfaces, e.g feet, as feet can be modelled by its
     * vertices. (This can lead to some physical inconsistencies but we
     * assume they are not relevant).
     */
    void SetContactFamily(const ContactName& family_name,
        const ContactNameList& contact_link_names);

    /**
     * @brief Deletes all contact families.
     */
    void ClearContactFamily(void);

    /**
     * @brief Set the desired posture.
     *
     * @ref SetRobotState must be called first, since the current
     * posture is used to calculate the position error.
     */
    void SetDesiredPosture(const JointPos&);
    void SetDesiredPosture(const JointPos&,
        const JointVel&);
    void SetDesiredPosture(const JointPos&,
        const JointVel&,
        const JointAcc&);

    /**
     * Sets the reference for the CoM
     */
    void SetDesiredCoM(const ComPos& com_pos);
    void SetDesiredCoM(const ComPos& com_pos, const ComVel& com_vel);
    void SetDesiredCoM(const ComPos& com_pos, const ComVel& com_vel,
        const ComAcc& com_acc);

    /**
     * @brief Sets the desired orientation for @ref frame_name.
     *
     * The desired orientation is persistant. It can be erased with @ref
     * EraseDesiredFrameOrientation.
     */
    void SetDesiredFrameOrientation(const std::string& frame_name, const FrameRot& frame_rot,
        const FrameAngVel& frame_ang_vel);

    /**
     * @brief Erases a desired frame orientation previously added with
     * @ref SetDesiredFrameOrientation.
     */
    void EraseDesiredFrameOrientation(const std::string& frame_name);

    /**
     * @brief Sets the dynamic constants of the specified task.
     */
    void SetTaskDynamics(const TaskName task, const double kp, const double kv);

    /**
     * @brief Retrieves the dynamics of a given task.
     */
    void GetTaskDynamics(const TaskName, double& kp, double& kv);

    /**
     * @brief Set the value of a task weight.
     */
    void SetTaskWeight(const TaskName task, const Weight w);

    /**
     * @brief Retrieve the weight assigned to a task.
     */
    Weight GetTaskWeight(const TaskName task);

    /**
     * @brief Sets the friction coefficient for all contacts for all contacts.
     */
    void SetFrictionCoefficient(double mu);

    /**
     * @brief Builds all the matrices for the QP program.
     * 
     * You need to call @ref SetRobotState, @ref SetPositionErrors,
     * @ref SetVelocityErrors and @ref SetReferenceAccelerations first.
     *
     */
    void BuildProblem(void);

    /**
     * @brief Solves the QP problem. 
     *
     * You need to call @ref BuildProblem before.
     *
     */
    void SolveProblem(void);

    /**
     * @brief Resets the warm start
     */
    void ResetWarmStart(void);

    /**
     * @brief Returns the last solution. You need to call @ref SolveProblem first.
     */
    Eigen::VectorXd GetSolution(void);

    /**
     * @brief Removes all active tasks.
     */
    void ClearTasks(void);

    /** 
     * @brief Removes all active contraints.
     */
    void ClearConstraints(void);

    /**
     * @brief Inserts a task.
     */
    void SetTask(const TaskName task);

    /**
     * @brief Insert a task and update its dynamics parameters.
     */
    void SetTask(const TaskName task, const double Kp, const double Kv);

    /**
     * @brief Erase an active task
     */
    void EraseTask(const TaskName task);

    /**
     * @brief Inserts a constraint.
     */
    void SetConstraint(const ConstraintName constraint);

    /**
     * @brief Deactivates a single constraint
     */
    void EraseConstraint(const ConstraintName constraint);

    /**
     * @brief Returns the number of variables for the current formulation.
     */
    int GetNumVariables(void) const;

    /**
     * @brief Returns the number of constraints rows for the current formulation.
     */
    int GetNumConstraints(void) const;

    /**
     * @brief Returns the center of mass. @ref SetRobotState needs to be called first.
     */
    Eigen::Vector3d GetCenterOfMass(void) const;

    /**
     * @brief Returns the center of mass velocity. @ref SetRobotState needs to be called first.
     */
    Eigen::Vector3d GetCenterOfMassVelocity(void) const;

  private:

    /**
     * @brief Given a list of contact names and orientations, saves a copy of them in the class.
     */
    void SaveContacts(const ContactNameList& contact_names,
        const ContactOrientationList& contact_orientations);

    // Dynamics algorithms
    /** 
     * @brief Computes the term dJ * qd for the current robot configuration
     */
    Eigen::MatrixXd ComputedJqd(void) const;

    /**
     * @brief Computes the center of mass of the robot with respect to WORLD.
     *
     * You need to call @ref SetRobotState first.
     */
    Eigen::VectorXd ComputeCoM(void) const;

    /**
     * @brief Compute and returns the center of mass jacobian with respect to WORLD.
     *
     * You need to call @ref SetRobotState first.
     */
    Eigen::MatrixXd ComputeCoMJacobian(void) const;

    /**
     * @brief Compute and returns the center of mass time jacobian with respect to WORLD.
     *
     * You need to call @ref SetRobotState and @ref ComputeCoM first, respectively.
     */
    Eigen::MatrixXd ComputeCoMJacobianTimeVariation(void) const;

    // QP Formulation
    /** 
     * @brief Initializes the solver parameters
     */
    void SetSolverParameters(void);

    /**
     * @brief Updates the @ref P_ matrix according to the robot state
     */
    void UpdateHessianMatrix(void);
    /**
     *  Update the @ref g_ matrix according to the robot state.
     *  You must call @ref SetPositionErrors, @ref SetVelocityErrors and 
     *  @ref SetReferenceAccelerations first.
     */
    void UpdateGradientMatrix(void);
    /**
     *  @brief Update the @ref l_ and @ref u_ matrices according to the robot state.
     */
    void UpdateBounds(void);
    /**
     *  @brief Update the @ref A_ matrix according to the robot state.
     */
    void UpdateLinearConstraints(void);

    /**
     * @brief Returns the number of rows for each constraint.
     * 
     * You must call @ref SetRobotState, since we need to know how many
     * contacts the robot is currently making.
     *
     */
    int GetNumConstraintRows(const ConstraintName constraint) const;

    /// QP Solver instance
    OsqpEigen::Solver solver_;
    /// Used to decide if warm start the problem with the previous solution
    bool bWarmStart_;
    /// QP Hessian matrix
    Eigen::SparseMatrix<double> P_;
    /// Gradient matrix
    Eigen::VectorXd g_;
    /// Linear constraint matrix
    Eigen::SparseMatrix<double> A_;
    /// Lower bound matrix
    Eigen::VectorXd l_;
    /// Upper bound matrix
    Eigen::VectorXd u_;

    // List of currently active tasks and constraints
    TaskList active_tasks_;
    TaskDynamicsList task_dynamics_;
    ConstraintList active_constraints_;

    // Joint state task
    WeightList task_weight_;
    PosErrors ep_;
    VelErrors ev_;
    JointAcc qrdd_;

    // The current contact jacobians
    ContactJacobians contact_jacobians_;
    ContactList contacts_;
    double mu_; /// Friction coeficient

    // Frame orientation task
    DesiredOrientations desired_orientations_;

    // Selection matrix;
    Eigen::MatrixXd S_;

    // Robot internal representation
    ModelPtr model_;
    DataPtr data_;

    // Joint states
    Eigen::VectorXd q_, qd_;
    // Desired CoM pos and vel
    Eigen::Vector3d des_com_pos_, des_com_vel_, des_com_acc_;
    bool b_com_vel_specified_, b_com_acc_specified_;

    // Joint actuator limits
    Eigen::VectorXd u_max_;

    // Saves the last solution
    Eigen::VectorXd solution_;

    // Saves the number of constraints in the most recent solved problem
    int last_num_constraints_;

    // Contact families
    ContactFamilyList contact_families_;
};

#endif /* TINY_WBC_H */
