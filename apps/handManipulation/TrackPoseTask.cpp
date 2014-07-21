#include <math.h>
#include "TrackPoseTask.h"
#include "dynamics/SkeletonDynamics.h"
#include "dynamics/BodyNodeDynamics.h"
#include "kinematics/dof.h"
#include "kinematics/Joint.h"
#include "kinematics/BodyNode.h"

using namespace Eigen;

namespace tasks {
TrackPoseTask::TrackPoseTask(dart::dynamics::Skeleton* _model, const std::vector<int>& _dofIndices, char *_name)
  : Task(_model)
{
  mDependTask = NULL;
  strcpy(mName, _name);
  mTaskType = tasks::POSE;
  mPriority = 1;
  mFinish = false;

  mDamp = false;
  mSmallGain = false;
  mSigmoidVGain = false;

  int numDofs = mModel->getNumDofs();
  mDofs.resize(numDofs);
  mDofVels.resize(numDofs);
  mTorque.resize(numDofs);
  mOmega.resize(3,numDofs);
  mJ.resize(numDofs,numDofs);
  mJDot.resize(numDofs,numDofs);
  mTargets.resize(_dofIndices.size());
  mDofIndices.resize(_dofIndices.size());
  mDofIndices = _dofIndices;
  mPGains.resize(numDofs);
  mVGains.resize(numDofs);
  mSmallPGains.resize(numDofs);
  mSmallVGains.resize(numDofs);
  // initialize mPGains and mVGains based on mass
  for(unsigned int i=0; i<numDofs; i++){ // can assign some initial value here
    double mass = 0.0;
    dynamics::BodyNode *mBodyNode = mModel->getDof(i)->getJoint()->getChildNode();
    while(mBodyNode)
    {
      mass += mBodyNode->getMass();
      if (mBodyNode->getNumChildJoints() >= 1)
      {
        mBodyNode = mBodyNode->getChildNode(0);
      }
      else
        break;
    }
    mPGains.at(i) = mass;
    mVGains.at(i) = mass;
    mPGains.at(i) *= -500.0;
    mVGains.at(i) *= 10.0;
    mSmallPGains.at(i) = mPGains.at(i);
    mSmallVGains.at(i) = mVGains.at(i);
  }

  mPGains.at(0) = mPGains.at(1) = -50.0;
  mVGains.at(0) = mVGains.at(1) = 5.0;
  for (int i = 2; i < numDofs; i++) {
    mPGains.at(i) = -15.0;
    mVGains.at(i) = 2.0;
  }
}

void TrackPoseTask::evalTorque() {
  int numDofs = mModel->getNumDofs();
  mJ = MatrixXd::Zero(numDofs,numDofs);
  mJDot = MatrixXd::Zero(numDofs,numDofs);
  bool flag = false;
  std::vector<double> sigmoidVGains;
  sigmoidVGains.resize(numDofs);
  for (int i = 0; i < numDofs; ++i) {
    for (int j = 0; j < mDofIndices.size(); ++j) {
      if (i == mDofIndices.at(j)) {
        if (mDamp) {
          mTorque(i) = -mVGains.at(i)*mDofVels(i);
        }
        else {
          mTorque(i) = mPGains.at(i)*(mDofs(i)-mTargets.at(mDofIndices.at(j))) - mVGains.at(i)*mDofVels(i);
        }
        mJ(i,i) = 1.0;
        flag = true;
      }
    }
    if (flag == false) {
      mTorque(i) = 0.0;
    }
    flag = false;
  }
  mOmega = mJ;
  FullPivLU<MatrixXd> lu_decomp(mOmega);
  mNullSpace = lu_decomp.kernel();
}

void TrackPoseTask::evalTaskFinish() {

}

void TrackPoseTask::updateTask(Eigen::VectorXd _state, const std::vector<double>& _targets, const std::vector<int>& _dofIndices) {
  mDofVels = _state.tail(mDofVels.size());
  mDofs = _state.head(mDofs.size());
  mTargets = _targets;
  mDofIndices = _dofIndices;
}

void TrackPoseTask::updateState(Eigen::VectorXd _state) {
  mDofVels = _state.tail(mDofVels.size());
  mDofs = _state.head(mDofs.size());
}

Eigen::MatrixXd TrackPoseTask::getNullSpace() const {
  int numDofs = mModel->getNumDofs();
  MatrixXd omega = MatrixXd::Zero(numDofs,numDofs);
  MatrixXd jv = MatrixXd::Zero(numDofs,numDofs);
  for (int i = 0; i < numDofs; ++i) {
    for (int j = 0; j < mDofIndices.size(); ++j) {
      if (i == mDofIndices.at(j)) {
        jv(i,i) = 1.0;
      }
    }
  }
  omega = jv;
  FullPivLU<MatrixXd> lu_decomp(omega);
  MatrixXd nullSpace = lu_decomp.kernel();
  return nullSpace;
}

Eigen::MatrixXd TrackPoseTask::getTaskSpace() const {
  int numDofs = mModel->getNumDofs();
  MatrixXd omega = MatrixXd::Zero(numDofs,numDofs);
  MatrixXd jv = MatrixXd::Zero(numDofs,numDofs);
  for (int i = 0; i < numDofs; ++i) {
    for (int j = 0; j < mDofIndices.size(); ++j) {
      if (i == mDofIndices.at(j)) {
        jv(i,i) = 1.0;
      }
    }
  }
  omega = jv;
  return omega;
}

void TrackPoseTask::setDamp(bool _damp) {
  mDamp = _damp;
}

void TrackPoseTask::setSigmoid(bool _sigmoid) {
  mSigmoidVGain = _sigmoid;
}
} // namespace tasks
