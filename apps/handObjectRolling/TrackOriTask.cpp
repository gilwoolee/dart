/*
 * Copyright (c) 2014, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Yunfei Bai <ybai30@mail.gatech.edu>,
 *            Jeongseok Lee <jslee02@gmail.com>
 *
 * Georgia Tech Graphics Lab and Humanoid Robotics Lab
 *
 * Directed by Prof. C. Karen Liu and Prof. Mike Stilman
 * <karenliu@cc.gatech.edu> <mstilman@cc.gatech.edu>
 *
 * This file is provided under the following "BSD-style" License:
 *   Redistribution and use in source and binary forms, with or
 *   without modification, are permitted provided that the following
 *   conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 *   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 */

#include "TrackOriTask.h"

#include "dart/dynamics/Skeleton.h"
#include "dart/dynamics/BodyNode.h"
#include "dart/dynamics/Joint.h"
#include "dart/dynamics/BodyNode.h"
#include "dart/math/Helpers.h"

using namespace Eigen;

namespace tasks {

//==============================================================================
TrackOriTask::TrackOriTask(dart::dynamics::Skeleton* _model,
                           const std::string& _eeName,
                           char *_name)
  : Task(_model),
    mEndEffectorName(_eeName)
{
  mDependTask = NULL;
  strcpy(mName, _name);
  mTaskType = tasks::ORI;
  mPriority = 2;
  mFinish = false;

  int numDofs = mModel->getNumDofs();
  mDofs.resize(numDofs);
  mDofVels.resize(numDofs);
  mTorque.resize(numDofs);
  mOmega.resize(3,numDofs);
  mJ.resize(3,numDofs);
  mJDot.resize(3,numDofs);
  mPGain = 100.0;
  mVGain = 5.0/*0.0*/;
  mIGain = 10.0/*0.0*/;
  mOtherForce = VectorXd::Zero(mModel->getNumDofs());
  mAccumulateError = Vector3d::Zero();
}

//==============================================================================
TrackOriTask::~TrackOriTask()
{
}

void TrackOriTask::evalTorque()
{
  // TODO(JS): Just commented out
//  mTorque = VectorXd::Zero(mModel->getNumDofs());

//  // use one DoF at the wrist to control rolling motion

//  int nodeJacobianIndex = 0; // index of column in Jacobian matrix of a node
//  int modelJacobianIndex = 0; // index of column in Jacobian matrix of a model
//  for (modelJacobianIndex = 0; modelJacobianIndex < mModel->getNumDofs(); ++modelJacobianIndex)
//  {
//    if (mModel->getBodyNode(mEEIndex)->dependsOn(modelJacobianIndex))
//    {
//      mJ.col(modelJacobianIndex) = mModel->getBodyNode(mEEIndex)->getJacobianAngular().col(nodeJacobianIndex);
//      nodeJacobianIndex++;
//    }
//    else
//    {
//      mJ.col(modelJacobianIndex) = VectorXd::Zero(3);
//    }
//  }
//  Eigen::Matrix4d transformMatrix = mModel->getBodyNode(mEEIndex)->getWorldTransform();
//  Eigen::Matrix3d rotationMatrix = transformMatrix.topLeftCorner(3,3);
//  Eigen::Vector3d orientationVector = rotationMatrix*Vector3d(0.0,-1.0,0.0);
//  double angle = acos(orientationVector.dot(mTarget)/(orientationVector.norm()*mTarget.norm()));
//  Vector3d omega = mJ*mDofVels; // angular velocity
//  VectorXd commandF;
//  if (dart_math::isZero((orientationVector.cross(mTarget)).norm())) {
//    commandF = Vector3d::Zero();
//  }
//  else commandF = mPGain*angle*((orientationVector.cross(mTarget)).normalized())-mVGain*omega;

//  // use integrate term
//  commandF = commandF + mIGain*mAccumulateError;

//  // the dof index related to the rotation of the wrist is 2, we care about the rotation about x axis whose index is 0, the minus is because local coordinate and world coordinate is opposite
//  mTorque(2) = -commandF(0);


//  // use three DoF at the wrist to control rolling motion
//  /*
//    int nodeJacobianIndex = 0; // index of column in Jacobian matrix of a node
//    int modelJacobianIndex = 0; // index of column in Jacobian matrix of a model
//    for (modelJacobianIndex = 0; modelJacobianIndex < mModel->getNumDofs(); ++modelJacobianIndex)
//    {
//      if (mModel->getBodyNode(mEEIndex)->dependsOn(modelJacobianIndex))
//      {
//        mJ.col(modelJacobianIndex) = mModel->getBodyNode(mEEIndex)->getJacobianAngular().col(nodeJacobianIndex);
//        nodeJacobianIndex++;
//      }
//      else
//      {
//        mJ.col(modelJacobianIndex) = VectorXd::Zero(3);
//      }
//    }

//    mOmega = mJ * (mModel->getMassMatrix().inverse());
//    FullPivLU<MatrixXd> lu_decomp(mOmega);
//    mNullSpace = lu_decomp.kernel();
//    dynamics::BodyNodeDynamics *nodel = static_cast<dynamics::BodyNodeDynamics*>(mModel->getBodyNode(mEEIndex));
//    Eigen::Matrix4d transformMatrix = mModel->getBodyNode(mEEIndex)->getWorldTransform();
//    Eigen::Matrix3d rotationMatrix = transformMatrix.topLeftCorner(3,3);
//    Eigen::Vector3d orientationVector = rotationMatrix*Vector3d(0.0,-1.0,0.0);
//    double angle = acos(orientationVector.dot(mTarget)/(orientationVector.norm()*mTarget.norm()));

//    Vector3d omega = mJ*mDofVels; // angular velocity
//    Matrix3d omegaStar = dart_math::makeSkewSymmetric(omega);
//    Vector3d orientationVectorVel = omegaStar*orientationVector;

//    VectorXd commandF;
//    if (dart_math::isZero((orientationVector.cross(mTarget)).norm())) {
//      commandF = Vector3d::Zero();
//    }
//    else commandF = mPGain*angle*((orientationVector.cross(mTarget)).normalized())-mVGain*omega;
//    nodeJacobianIndex = 0;
//    modelJacobianIndex = 0;
//    for (modelJacobianIndex = 0; modelJacobianIndex < mModel->getNumDofs(); ++modelJacobianIndex)
//    {
//      if (mModel->getBodyNode(mEEIndex)->dependsOn(modelJacobianIndex))
//      {
//        mJDot.col(modelJacobianIndex) = nodel->mJwDot.col(nodeJacobianIndex);
//        nodeJacobianIndex++;
//      }
//      else
//      {
//        mJDot.col(modelJacobianIndex) = VectorXd::Zero(3);
//      }
//    }

//    if (dart_math::isEqual((mOmega*mOmega.transpose()).determinant(),0.0)) {
//      cout << "not invertible" << endl;
//    }
//    mTorque = mOmega.transpose()*(mOmega*mOmega.transpose()).inverse()*(commandF+mOmega*(mModel->getCombinedVector() - mOtherForce)-mJDot*mDofVels);

//    for (int i = 3; i < mModel->getNumDofs(); ++i) {
//      mTorque(i) = 0.0;
//    }
//    */
}

void TrackOriTask::evalTaskFinish()
{
  // TODO(JS): Just commented out
//  Eigen::Matrix4d transformMatrix = mModel->getBodyNode(mEEIndex)->getWorldTransform();
//  Eigen::Matrix3d rotationMatrix = transformMatrix.topLeftCorner(3,3);
//  Eigen::Vector3d orientationVector = rotationMatrix*Vector3d(0.0,-1.0,0.0);
//  double angle = acos(orientationVector.dot(mTarget)/(orientationVector.norm()*mTarget.norm()));

//  if (angle < ORIENTATION_CLOSE_THRESHOLD)
//    mFinish = true;
}

void TrackOriTask::updateTask(Eigen::VectorXd _state, Eigen::Vector3d _target, Eigen::VectorXd _otherForce) {
  mDofVels = _state.tail(mDofVels.size());
  mDofs = _state.head(mDofs.size());
  mTarget = _target;
  mOtherForce = _otherForce;
}

Eigen::MatrixXd TrackOriTask::getNullSpace() const
{
  // TODO(JS): Just commented out
//  int numDofs = mModel->getNumDofs();
//  MatrixXd omega = MatrixXd::Zero(3,numDofs);
//  MatrixXd jw = MatrixXd::Zero(3,numDofs);
//  int nodeJacobianIndex = 0; // index of column in Jacobian matrix of a node
//  int modelJacobianIndex = 0; // index of column in Jacobian matrix of a model
//  for (modelJacobianIndex = 0; modelJacobianIndex < mModel->getNumDofs(); ++modelJacobianIndex)
//  {
//    if (mModel->getBodyNode(mEEIndex)->dependsOn(modelJacobianIndex))
//    {
//      jw.col(modelJacobianIndex) = mModel->getBodyNode(mEEIndex)->getJacobianAngular().col(nodeJacobianIndex);
//      nodeJacobianIndex++;
//    }
//    else
//    {
//      jw.col(modelJacobianIndex) = VectorXd::Zero(3);
//    }
//  }

//  omega = jw * (mModel->getMassMatrix().inverse());
//  FullPivLU<MatrixXd> lu_decomp(omega);
//  MatrixXd nullSpace = lu_decomp.kernel();
//  return nullSpace;
}

Eigen::MatrixXd TrackOriTask::getTaskSpace() const
{
  // TODO(JS): Just commented out
//  int numDofs = mModel->getNumDofs();
//  MatrixXd omega = MatrixXd::Zero(3,numDofs);
//  MatrixXd jw = MatrixXd::Zero(3,numDofs);
//  int nodeJacobianIndex = 0; // index of column in Jacobian matrix of a node
//  int modelJacobianIndex = 0; // index of column in Jacobian matrix of a model
//  for (modelJacobianIndex = 0; modelJacobianIndex < mModel->getNumDofs(); ++modelJacobianIndex)
//  {
//    if (mModel->getBodyNode(mEEIndex)->dependsOn(modelJacobianIndex))
//    {
//      jw.col(modelJacobianIndex) = mModel->getBodyNode(mEEIndex)->getJacobianAngular().col(nodeJacobianIndex);
//      nodeJacobianIndex++;
//    }
//    else
//    {
//      jw.col(modelJacobianIndex) = VectorXd::Zero(3);
//    }
//  }

//  omega = jw * (mModel->getMassMatrix().inverse());
//  return omega;
}

//==============================================================================
Vector3d TrackOriTask::getTarget()
{
  return mTarget;
}

//==============================================================================
const std::string& TrackOriTask::getEEName() const
{
  return mEndEffectorName;
}

//==============================================================================
void TrackOriTask::setTarget(Vector3d _target)
{
  mTarget = _target;
}

//==============================================================================
Eigen::Vector3d TrackOriTask::evalTaskError()
{
  // TODO(JS): Just commented out
//  Eigen::Matrix4d transformMatrix = mModel->getBodyNode(mEEIndex)->getWorldTransform();
//  Eigen::Matrix3d rotationMatrix = transformMatrix.topLeftCorner(3,3);
//  Eigen::Vector3d orientationVector = rotationMatrix*Vector3d(0.0,-1.0,0.0);
//  double angle = acos(orientationVector.dot(mTarget)/(orientationVector.norm()*mTarget.norm()));
//  return angle*((orientationVector.cross(mTarget)).normalized());
}

//==============================================================================
void TrackOriTask::setAccumulateError(Vector3d _accumulateError)
{
  mAccumulateError = _accumulateError;
}

} // namespace tasks
