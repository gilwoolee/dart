/*
 * Copyright (c) 2011-2014, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Karen Liu <karenliu@cc.gatech.edu>,
 *            Jeongseok Lee <jslee02@gmail.com>
 *
 * Geoorgia Tech Graphics Lab and Humanoid Robotics Lab
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

#include "dart/constraint/ImpulseBasedBallJointConstraint.h"

#include "dart/math/Helpers.h"
#include "dart/dynamics/BodyNode.h"
#include "dart/dynamics/Skeleton.h"

using namespace dart;
using namespace math;

namespace dart {
namespace constraint {

ImpulseBasedBallJointConstraint::ImpulseBasedBallJointConstraint(dynamics::BodyNode *_body1, dynamics::BodyNode *_body2, Eigen::Vector3d _offset1, Eigen::Vector3d _offset2) {
  mBodyNode1 = _body1;
  mBodyNode2 = _body2;
  mOffset1 = _offset1;
  mOffset2 = _offset2;
  mNumRows = 3;
  mJ1 = Eigen::MatrixXd::Zero(mNumRows, mBodyNode1->getSkeleton()->getNumGenCoords());
  mJ2 = Eigen::MatrixXd::Zero(mNumRows, mBodyNode2->getSkeleton()->getNumGenCoords());
}

ImpulseBasedBallJointConstraint::ImpulseBasedBallJointConstraint(dynamics::BodyNode *_body1, dynamics::BodyNode *_body2, Eigen::Vector3d _jointPosition) {
  mBodyNode1 = _body1;
  mBodyNode2 = _body2;
  mOffset1 = mBodyNode1->getWorldTransform().inverse() * _jointPosition;
  mOffset2 = mBodyNode2->getWorldTransform().inverse() * _jointPosition;
  mNumRows = 3;
  mJ1 = Eigen::MatrixXd::Zero(mNumRows, mBodyNode1->getSkeleton()->getNumGenCoords());
  mJ2 = Eigen::MatrixXd::Zero(mNumRows, mBodyNode2->getSkeleton()->getNumGenCoords());
}

ImpulseBasedBallJointConstraint::ImpulseBasedBallJointConstraint(dynamics::BodyNode *_body1, Eigen::Vector3d _offset1, Eigen::Vector3d _target) {
  mBodyNode1 = _body1;
  mBodyNode2 = NULL;
  mOffset1 = _offset1;
  mOffset2 = _target;
  mNumRows = 3;
  mJ1 = Eigen::MatrixXd::Zero(mNumRows, mBodyNode1->getSkeleton()->getNumGenCoords());
}

ImpulseBasedBallJointConstraint::~ImpulseBasedBallJointConstraint() {
}

void ImpulseBasedBallJointConstraint::updateDynamics(Eigen::MatrixXd & _J1, Eigen::VectorXd & _C, Eigen::VectorXd & _CDot, int _rowIndex) {
  getJacobian();
  _J1.block(_rowIndex, 0, 3, mBodyNode1->getSkeleton()->getNumGenCoords()) = mJ1;
  Eigen::Vector3d worldP1 = mBodyNode1->getWorldTransform() * mOffset1;
  Eigen::VectorXd qDot1 = mBodyNode1->getSkeleton()->get_dq();
  _C.segment(_rowIndex, 3) = worldP1 - mOffset2;
  _CDot.segment(_rowIndex, 3) = mJ1 * qDot1;
}

void ImpulseBasedBallJointConstraint::updateDynamics(Eigen::MatrixXd & _J1, Eigen::MatrixXd & _J2, Eigen::VectorXd & _C, Eigen::VectorXd & _CDot, int _rowIndex) {
  getJacobian();
  _J2.block(_rowIndex, 0, 3, mBodyNode2->getSkeleton()->getNumGenCoords()).setZero();
  _J1.block(_rowIndex, 0, 3, mBodyNode1->getSkeleton()->getNumGenCoords()) = mJ1;
  _J2.block(_rowIndex, 0, 3, mBodyNode2->getSkeleton()->getNumGenCoords()) += mJ2;

  Eigen::Vector3d worldP1 = mBodyNode1->getWorldTransform() * mOffset1;
  Eigen::VectorXd qDot1 = ((dynamics::Skeleton*)mBodyNode1->getSkeleton())->get_dq();
  Eigen::VectorXd worldP2 = mBodyNode2->getWorldTransform() * mOffset2;
  Eigen::VectorXd qDot2 = ((dynamics::Skeleton*)mBodyNode2->getSkeleton())->get_dq();

  _C.segment(_rowIndex, 3) = worldP1 - worldP2;
  _CDot.segment(_rowIndex, 3) = mJ1 * qDot1 + mJ2 * qDot2;
}

void ImpulseBasedBallJointConstraint::getJacobian() {
  Eigen::Vector3d offsetWorld = mBodyNode1->getWorldTransform().rotation() * mOffset1;
  Eigen::MatrixXd JBody1 = mBodyNode1->getWorldJacobian(offsetWorld).bottomRows<3>();
  for(int i = 0; i < mBodyNode1->getNumDependentGenCoords(); i++) {
    int dofIndex = mBodyNode1->getDependentGenCoordIndex(i);
    mJ1.col(dofIndex) = JBody1.col(i);
  }
  if (mBodyNode2) {
    offsetWorld = mBodyNode2->getWorldTransform().rotation() * mOffset2;
    Eigen::MatrixXd JBody2 = mBodyNode2->getWorldJacobian(offsetWorld).bottomRows<3>();
    for(int i = 0; i < mBodyNode2->getNumDependentGenCoords(); i++) {
      int dofIndex = mBodyNode2->getDependentGenCoordIndex(i);
      mJ2.col(dofIndex) = -JBody2.col(i);
    }
  }
}

} // namespace constraint
} // namespace dart
