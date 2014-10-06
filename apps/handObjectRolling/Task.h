#ifndef TASK_H
#define TASK_H

#include <iostream>
#include <Eigen/Dense>

namespace dart {
namespace dynamics {
class Skeleton;
class BodyNode;
}  // namespace dynamics
}  // namespace dart

using namespace Eigen;

namespace tasks {

#define MAX_TASK_NAME 128
#define DISTANCE_CLOSE_THRESHOLD_ARM 0.03
#define DISTANCE_CLOSE_THRESHOLD_HAND_STRICT 0.01
#define DISTANCE_CLOSE_THRESHOLD_HAND_LOOSE 0.03
#define ORIENTATION_CLOSE_THRESHOLD 0.2

enum taskType
{
  EE,
  POSE,
  MAINTAIN,
  ORI,
  TRANSPORT
};

/// class Task
class Task
{
public:
  /// Constructor
  explicit Task(dart::dynamics::Skeleton* _model);

  /// Destructor
  virtual ~Task();

public:
  ///
  Eigen::VectorXd getTorque() const;

  ///
  virtual void evalTorque();

  ///
  virtual Eigen::MatrixXd getNullSpace() const;

  ///
  virtual Eigen::MatrixXd getTaskSpace() const;

  ///
  inline void setDependTask(Task* _task);

  ///
  inline void setPriority(int _priority);

  ///
  virtual void evalTaskFinish();

  ///
  void setName(const std::string& _name);

  ///
  inline const std::string& getName() const;

public:
  ///
  dart::dynamics::Skeleton* mModel;

  /// Task that this task is depend on
  Task* mDependTask;

  /// Name
  std::string mName;

  /// Task type
  taskType mTaskType;

  /// Priority of the task
  int mPriority;

  /// Flag that this task is finished
  bool mFinish;

  /// Torque to fulfill this task
  Eigen::VectorXd mTorque;

  /// Current dof of the model
  Eigen::VectorXd mDofs;

  /// Current dof velocity of the model
  Eigen::VectorXd mDofVels;

  ///
  Eigen::MatrixXd mOmega;

  /// Jacobian matrix for this task, wrt all dofs of the model
  Eigen::MatrixXd mJ;

  /// time derivative of Jacobian matrix for this task, wrt all dofs of the model
  Eigen::MatrixXd mJDot;

  /// null space of matrix Omega
  Eigen::MatrixXd mNullSpace;

  // 	private:
  // 		Task(const Task& _copy) {}
};

} // namespace tasks

#endif // #ifndef TASK_H

