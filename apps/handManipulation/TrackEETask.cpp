#include "TrackEETask.h"

#include "dart/dynamics/Skeleton.h"
#include "dart/dynamics/BodyNode.h"
#include "dart/dynamics/Joint.h"
#include "dart/dynamics/BodyNode.h"

using namespace Eigen;

namespace tasks {

TrackEETask::TrackEETask(dart::dynamics::Skeleton* _model, dart::dynamics::Skeleton* _obj, int _eeIndex, char *_name, EETraj* _traj)
  : Task(_model)
{
  mDependTask = NULL;
  strcpy(mName, _name);
  mTaskType = tasks::EE;
  mPriority = 3;
  mFinish = false;

  int numDofs = mModel->getNumDofs();
  mDofs.resize(numDofs);
  mDofVels.resize(numDofs);
  mTorque.resize(numDofs);
  mOmega.resize(3,numDofs);
  mJ.resize(3,numDofs);
  mJDot.resize(3,numDofs);
  mEEIndex = _eeIndex;
  // 		mPGain = -50.0;
  // 		mVGain = -30.0;
  // 		mPGain = -70.0;
  // 		mVGain = -15.0;
  // 		mPGain = -40.0;
  // 		mVGain = -15.0;
  // 		mPGainTraj = -70.0;
  // 		mVGainTraj = -100.0;
  mPGainTraj = -70.0;
  mVGainTraj = -70.0;
  mPGain = -70.0;
  mVGain = -30.0;

  mOtherForce = VectorXd::Zero(mModel->getNumDofs());

  setObj(_obj);
}

void TrackEETask::evalTorque()
{
  // TODO(JS): Just commented out
//  int nodeJacobianIndex = 0; // index of column in Jacobian matrix of a node
//  int modelJacobianIndex = 0; // index of column in Jacobian matrix of a model
//  for (modelJacobianIndex = 0; modelJacobianIndex < mModel->getNumDofs(); ++modelJacobianIndex)
//  {
//    if (mModel->getBodyNode(mEEIndex)->dependsOn(modelJacobianIndex))
//    {
//      mJ.col(modelJacobianIndex) = mModel->getBodyNode(mEEIndex)->getJacobianLinear().col(nodeJacobianIndex);
//      nodeJacobianIndex++;
//    }
//    else
//    {
//      mJ.col(modelJacobianIndex) = VectorXd::Zero(3);
//    }
//  }

//  mOmega = mJ * (mModel->getMassMatrix().inverse());
//  FullPivLU<MatrixXd> lu_decomp(mOmega);
//  mNullSpace = lu_decomp.kernel();

//  dynamics::BodyNodeDynamics *nodel = static_cast<dynamics::BodyNodeDynamics*>(mModel->getBodyNode(mEEIndex));
//  VectorXd commandF;

//  commandF = mPGainTraj*(mModel->getBodyNode(mEEIndex)->getWorldCOM()-mTarget) + mVGainTraj*(mJ*mDofVels-mTargetVel)/*+ mVGainTraj*mJ*mDofVels*/;

//  nodeJacobianIndex = 0;
//  modelJacobianIndex = 0;
//  for (modelJacobianIndex = 0; modelJacobianIndex < mModel->getNumDofs(); ++modelJacobianIndex)
//  {
//    if (mModel->getBodyNode(mEEIndex)->dependsOn(modelJacobianIndex))
//    {
//      mJDot.col(modelJacobianIndex) = nodel->mJvDot.col(nodeJacobianIndex);
//      nodeJacobianIndex++;
//    }
//    else
//    {
//      mJDot.col(modelJacobianIndex) = VectorXd::Zero(3);
//    }
//  }
//  mTorque = mOmega.transpose()*(mOmega*mOmega.transpose()).inverse()*(commandF+mOmega*(mModel->getCombinedVector() - mOtherForce)-mJDot*mDofVels);
}

void TrackEETask::evalTaskFinish()
{
  if ((mModel->getBodyNode(mEEIndex)->getWorldCOM()-mFinalTarget).norm() < DISTANCE_CLOSE_THRESHOLD_HAND_STRICT)
    mFinish = true;
}

bool TrackEETask::evalCloseToTarget()
{
  if ((mModel->getBodyNode(mEEIndex)->getWorldCOM()-mFinalTarget).norm() < DISTANCE_CLOSE_THRESHOLD_HAND_LOOSE)
    return true;
  else
    return false;
}

void TrackEETask::updateTask(Eigen::VectorXd _state, Eigen::Vector3d _target, Eigen::VectorXd _otherForce) {
  mDofVels = _state.tail(mDofVels.size());
  mDofs = _state.head(mDofs.size());
  mTarget = _target;
  mOtherForce = _otherForce;
}

Eigen::MatrixXd TrackEETask::getNullSpace() const
{
  // TODO(JS): Just commented out
//  int numDofs = mModel->getNumDofs();
//  MatrixXd omega = MatrixXd::Zero(3,numDofs);
//  MatrixXd jv = MatrixXd::Zero(3,numDofs);
//  int nodeJacobianIndex = 0; // index of column in Jacobian matrix of a node
//  int modelJacobianIndex = 0; // index of column in Jacobian matrix of a model
//  for (modelJacobianIndex = 0; modelJacobianIndex < mModel->getNumDofs(); ++modelJacobianIndex)
//  {
//    if (mModel->getBodyNode(mEEIndex)->dependsOn(modelJacobianIndex))
//    {
//      jv.col(modelJacobianIndex) = mModel->getBodyNode(mEEIndex)->getJacobianLinear().col(nodeJacobianIndex);
//      nodeJacobianIndex++;
//    }
//    else
//    {
//      jv.col(modelJacobianIndex) = VectorXd::Zero(3);
//    }
//  }

//  omega = jv * (mModel->getMassMatrix().inverse());
//  FullPivLU<MatrixXd> lu_decomp(omega);
//  MatrixXd nullSpace = lu_decomp.kernel();

//  return nullSpace;
  return Eigen::MatrixXd::Zero(0, 0);
}

Eigen::MatrixXd TrackEETask::getTaskSpace() const
{
  // TODO(JS): Just commented out
//  int numDofs = mModel->getNumDofs();
//  MatrixXd omega = MatrixXd::Zero(3,numDofs);
//  MatrixXd jv = MatrixXd::Zero(3,numDofs);
//  int nodeJacobianIndex = 0; // index of column in Jacobian matrix of a node
//  int modelJacobianIndex = 0; // index of column in Jacobian matrix of a model
//  for (modelJacobianIndex = 0; modelJacobianIndex < mModel->getNumDofs(); ++modelJacobianIndex)
//  {
//    if (mModel->getBodyNode(mEEIndex)->dependsOn(modelJacobianIndex))
//    {
//      jv.col(modelJacobianIndex) = mModel->getBodyNode(mEEIndex)->getJacobianLinear().col(nodeJacobianIndex);
//      nodeJacobianIndex++;
//    }
//    else
//    {
//      jv.col(modelJacobianIndex) = VectorXd::Zero(3);
//    }
//  }

//  omega = jv * (mModel->getMassMatrix().inverse());
//  return omega;
  return Eigen::MatrixXd::Zero(0, 0);
}

} // namespace tasks
