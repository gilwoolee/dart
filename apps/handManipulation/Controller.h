#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <Eigen/Dense>
#include <iostream>
#include <vector>
#include <string>
#include <cstring>
#include "Task.h"
#include "MaintainTask.h"
#include "TrackOriTask.h"
#include "TrackPoseTask.h"
#include "TrackEETask.h"

extern double angleX;
extern double angleY;
extern double angleZ;
extern double handPosX;
extern double handPosY;
extern double handPosZ;
extern double objPosX;
extern double objPosY;
extern double objPosZ;

namespace dart {
namespace dynamics {
class SkeletonDynamics;
}  // namespace dynamics
namespace optimizer {
class ObjectiveBox;
class Var;
}  // namespace optimizer
}  // namespace dart

namespace tasks {
class Task;
class MaintainTask;
class TrackOriTask;
class TrackPoseTask;
class TrackEETask;
}  // namespace tasks

class Controller
{
public:
  /// Constructor
  Controller(dart::dynamics::Skeleton* _skel,
             double _t,
             std::vector<tasks::Task*>& _tasks,
             int _fingerNum,
             std::vector<std::string>& _fingerRootNames, std::vector<std::string>& _fingerTipNames,
             //std::vector<int>& _fingerTipIndices,
             std::string _palmName);

  /// Destructor
  virtual ~Controller();

//  void resetController(std::vector<tasks::Task*>& _tasks);

  ///
  Eigen::VectorXd getTorques();

//  double getTorque(int _index) { return mTorques[_index]; }

  ///
  void setDesiredDof(int _index, double _val);

  /// Compute torques
  /// \param[in] _dof
  /// \param[in] _dofVel
  /// \param[in] _dofAcc
  /// \param[in] _objDof
  /// \param[in] _objVel
  /// \param[in] _contactPoints
  /// \param[in] _contactForces
  /// \param[in] _contactBodyNames
  /// \param[in] _targetOri
  /// \param[in] _objOri
  void computeTorques(const Eigen::VectorXd& _dof,
                      const Eigen::VectorXd& _dofVel,
                      const Eigen::VectorXd& _dofAcc,
                      const Eigen::VectorXd& _objDof,
                      const Eigen::VectorXd& _objVel,
                      std::vector<Eigen::Vector3d>& _contactPoints,
                      std::vector<Eigen::Vector3d>& _contactForces,
                      std::vector<std::string>& _contactBodyNames,
                      const Eigen::Vector3d& _targetOri,
                      const Eigen::Vector3d& _objOri);

//  dart::dynamics::Skeleton*  getSkel() { return mSkel; }
//  Eigen::VectorXd getDesiredDofs() { return mDesiredDofs; }
//  Eigen::MatrixXd getKp() {return mKp; }
//  void evalTargetOri(const Eigen::VectorXd& _dof, const Eigen::VectorXd& _dofVel, const Eigen::VectorXd& _objDof, const Eigen::VectorXd& _objVel, const Eigen::Vector3d& _offset, const Eigen::Vector3d& _targetOri, const Eigen::Vector3d& _objOri);

  ///
  void evalTargetPose(const Eigen::VectorXd& _dof,
                      const Eigen::VectorXd& _dofVel,
                      const Eigen::VectorXd& _objDof,
                      const Eigen::VectorXd& _objVel,
                      const Eigen::Vector3d& _offset,
                      const Eigen::Vector3d& _targetOri,
                      const Eigen::Vector3d& _objOri);

//  Eigen::MatrixXd evalTaskNullSpace();

  /// Compute gravity compensation force and store it to
  /// mGravityCompensationForce
  void evalGravityCompensationForce();

  /// ... and store it to mObjControlForce
  // void evalObjControlForce(const std::vector<Eigen::Vector3d>& _contactPoints,
  //                          const std::vector<Eigen::Vector3d>& _contactForces,
  //                          const std::vector<std::string>& _contactBodyNames);

  ///
  void evalTrackForce(const Eigen::VectorXd& _dof,
                      const Eigen::VectorXd& _dofVel);

  /// Compute damping force
  void evalDampForce(const Eigen::VectorXd& _dof);

  /// Compute ...
  void evalOriForce(const Eigen::VectorXd& _dof,
                    const Eigen::VectorXd& _dofVel);

//  void evalMaintainForce(const Eigen::VectorXd& _dof, const Eigen::VectorXd& _dofVel);

  ///
  // void evalTaskForce();

public:
  dart::dynamics::Skeleton *mSkel;
  Eigen::VectorXd mTorques;
  Eigen::VectorXd mGravityCompensationForce;
  Eigen::VectorXd mObjControlForce;
  Eigen::VectorXd mTrackForce;
  Eigen::VectorXd mDampForce;
  Eigen::VectorXd mOriForce;
  Eigen::VectorXd mMaintainForce;
  Eigen::VectorXd mTaskForce;
  Eigen::VectorXd mConstraintForce;
  Eigen::VectorXd mDesiredDofs;
  Eigen::VectorXd mFingerEndPose;
  Eigen::VectorXd mFingerRestPose;
  Eigen::VectorXd mFingerInterceptPose;
  Eigen::MatrixXd mKp;
  Eigen::MatrixXd mKd;
  double mTimestep;
  int mFrame;
  int mFingerNum;

  std::vector<tasks::Task*> mTasks;
  std::vector<Eigen::MatrixXd> mJVec;
  std::vector<Eigen::MatrixXd> mInvJVec;
  Eigen::MatrixXd mCombinedJ;
  Eigen::MatrixXd mCombinedJTrans;

  std::vector<int> mActiveNumFrame;
  std::vector<int> mActiveSimFrame;
  std::vector<int> mTrackNumFrame;
  std::vector<int> mTrackSimFrame;
  std::vector<int> mInContactFrame;
  std::vector<bool> mActiveFingers;
  std::vector<bool> mContactFingers;
  std::vector<bool> mInContactFingers;
  std::vector<bool> mRestFingers;
  std::vector<bool> mTrackFingers;
  std::vector<VectorXi> mFingerDofs;
  Eigen::VectorXi mOriDofs;
  std::vector<std::string> mFingerRootNames;
  std::vector<std::string> mFingerTipNames;
  std::string mPalmName;

  // Excluded node number when doing virtual force control
  int mExtNodeNum;

  // Excluded dof number when doing virtual force control
  int mExtDofNum;

  // Plan related
  bool mOriFlag;
  int mOriSimFrame;
  int mOriNumFrame;
  bool mMaintainFlag;

  ///
  bool mControlFlag;

  ///
  bool mTaskFlag;

  ///
  bool mOnPalmFlag;

  Eigen::Vector3d mPreOriTarget;
  Eigen::Vector3d mAccumulateOriError;

};



#endif // #CONTROLLER_H
