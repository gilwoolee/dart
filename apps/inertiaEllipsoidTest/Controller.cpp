/*
 * Copyright (c) 2014, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Jeongseok Lee <jslee02@gmail.com>
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

#include "apps/inertiaEllipsoidTest/Controller.h"

#include "dart/math/Helpers.h"
#include "dart/dynamics/Skeleton.h"
#include "dart/dynamics/BodyNode.h"
#include "dart/dynamics/Shape.h"
#include "dart/collision/CollisionDetector.h"

using namespace dart;
using namespace dynamics;
using namespace math;

Controller::Controller(dynamics::Skeleton* _skel,
                       constraint::ConstraintSolver* _collisionSolver,
                       double _t)
{
  mSkel = _skel;
  mCollisionHandle = _collisionSolver;
  mTimestep = _t;
  mFrame = 0;
  int nDof = mSkel->getNumDofs();
  mKp = Eigen::MatrixXd::Identity(nDof, nDof);
  mKd = Eigen::MatrixXd::Identity(nDof, nDof);
  mConstrForces = Eigen::VectorXd::Zero(nDof);

  mTorques.resize(nDof);
  mDesiredDofs.resize(nDof);
  for (int i = 0; i < nDof; i++)
  {
    mTorques[i] = 0.0;
    mDesiredDofs[i] = mSkel->getPosition(i);
  }

  mRotorX = mSkel->getJoint(1);
  mRotorY = mSkel->getJoint(2);
  mRotorZ = mSkel->getJoint(3);
}

void Controller::computeTorques(const Eigen::VectorXd& _dof,
                                const Eigen::VectorXd& _dofVel)
{
  mFrame++;
}

void Controller::setTorqueRotorX(double _torque)
{
  mRotorX->setForce(0, _torque);
}

void Controller::setTorqueRotorY(double _torque)
{
  mRotorY->setForce(0, _torque);
}

void Controller::setTorqueRotorZ(double _torque)
{
  mRotorZ->setForce(0, _torque);
}

