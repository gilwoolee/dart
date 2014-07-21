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

enum taskType {
  EE,
  POSE,
  MAINTAIN,
  ORI,
  TRANSPORT
};

class Task {
public:
  explicit Task(dart::dynamics::Skeleton* _model);
  virtual ~Task() {}

public:
  dart::dynamics::Skeleton* mModel;
  Task* mDependTask;        // the task that this task is depend on

  char mName[MAX_TASK_NAME];
  taskType mTaskType;
  int mPriority;            // the priority of the task
  bool mFinish;             // the flag that this task is finished

  Eigen::VectorXd mTorque;  // the torque to fulfill this task
  Eigen::VectorXd mDofs;    // the current dof of the model
  Eigen::VectorXd mDofVels; // the current dof velocity of the model
  Eigen::MatrixXd mOmega;
  Eigen::MatrixXd mJ;       // Jacobian matrix for this task, wrt all dofs of the model
  Eigen::MatrixXd mJDot;    // time derivative of Jacobian matrix for this task, wrt all dofs of the model
  Eigen::MatrixXd mNullSpace; // null space of matrix Omega

  inline Eigen::VectorXd getTorque() const { return mTorque; }
  virtual void evalTorque() {}
  virtual Eigen::MatrixXd getNullSpace() const { return MatrixXd::Zero(1,1); }
  virtual Eigen::MatrixXd getTaskSpace() const { return MatrixXd::Zero(1,1); }
  inline void setDependTask(Task* _task) { mDependTask = _task; }
  inline void setPriority(int _priority) { mPriority = _priority; }
  virtual void evalTaskFinish() {}

  void setName(char* _n) { strcpy(mName, _n); }
  inline char* getName() { return mName; }

  // 	private:
  // 		Task(const Task& _copy) {}
};
} // namespace tasks

#endif // #ifndef TASK_H

