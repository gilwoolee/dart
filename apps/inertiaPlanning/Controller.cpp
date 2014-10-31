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

#include "apps/inertiaPlanning/Controller.h"

#include "dart/math/Helpers.h"
#include "dart/dynamics/Skeleton.h"
#include "dart/dynamics/BodyNode.h"
#include "dart/dynamics/Shape.h"
#include "dart/collision/CollisionDetector.h"

using namespace Eigen;

using namespace dart;
using namespace dynamics;
using namespace math;

Controller::Controller(dynamics::Skeleton* _skel)
{
  assert(_skel);
  mSkel = _skel;
  int dof = mSkel->getNumDofs();

  mTorques = Eigen::VectorXd::Zero(dof);

  mOff = false;
}

void Controller::setInitState(double q0, double q1, double w0, double w1)
{
  mQ0_i = q0;
  mQ1_i = q1;

  mW0_i = w0;
  mW1_i = w1;
}

void Controller::setFinalState(double q0, double q1, double w0, double w1)
{
  mQ0_f = q0;
  mQ1_f = q1;

  mW0_f = w0;
  mW1_f = w1;
}

void Controller::setDesiredRotation(double desiredRotation)
{
  mDesiredRotation = desiredRotation;
}

void Controller::setDuration(double duration)
{
  mDuration = duration;
}

void Controller::init()
{
  mSkel->getJoint(0)->setPosition(0, mQ0_i);
  mSkel->getJoint(1)->setPosition(0, mQ1_i);

  mSkel->getJoint(0)->setVelocity(0, mW0_i);
  mSkel->getJoint(1)->setVelocity(0, mW1_i);

  mSkel->computeForwardKinematics(true, true, false);

  double Izz = mSkel->getTotalSpatialInertiaTensorRoot()(2,2);
  double w = mSkel->getBodyNode(0)->getWorldAngularVelocity()[2];
  mH = Izz * w;

  std::cout << "mH: " << mH << std::endl;

  mW0_i;

  mDesiredAngularVelocity = mDesiredRotation / (mDuration + 2.0);

  std::cout << "Current W: " << w << std::endl;
  std::cout << "Desired W: " << mDesiredAngularVelocity << std::endl;


  double q   = mSkel->getJoint(1)->getPosition(0);
  double dq  = mSkel->getJoint(1)->getVelocity(0);

  double mDesiredIzz = mH / mDesiredAngularVelocity;
  double mDesiredQ   = mSkel->setDesiredIzz(mDesiredIzz);
  double tau  = -20.0*(q - mDesiredQ) - 1.0*dq;

  // TODO(JS):
//  mDesiredAngularVelocity *= 1.3;


  std::cout << "Current Izz: " << Izz << std::endl;
  std::cout << "Desired Izz: " << mDesiredIzz << std::endl;
}

void Controller::update(double time)
{
  if (mOff)
    return;

  if (time < 1.0)
  {
    double q   = mSkel->getJoint(1)->getPosition(0);
    double dq  = mSkel->getJoint(1)->getVelocity(0);

    double mDesiredIzz = mH / mDesiredAngularVelocity;
    double mDesiredQ   = mSkel->setDesiredIzz(mDesiredIzz);
    double tau  = -5.0*(q - DART_RADIAN*30) - 1.0*dq;

    mSkel->getJoint(1)->setForce(0, tau);

    double w = mSkel->getBodyNode(0)->getWorldAngularVelocity()[2];
    double Izz = mSkel->getTotalSpatialInertiaTensorRoot()(2,2);

    std::cout << "Delta Q: " << q - mDesiredQ << std::endl;
//    std::cout << "Current W: " << w << std::endl;
//    std::cout << "Current Izz: " << Izz << std::endl;
  }
  else if (mDuration - 1.0 < time && time < mDuration)
  {
    double q   = mSkel->getJoint(1)->getPosition(0);
    double dq  = mSkel->getJoint(1)->getVelocity(0);

    double mDesiredIzz = mH / mDesiredAngularVelocity;
    double mDesiredQ   = mSkel->setDesiredIzz(mDesiredIzz);
    double tau  = -5.0*(q - mQ1_i) - 1.0*dq;

    mSkel->getJoint(1)->setForce(0, tau);
  }
}

VectorXd Controller::linspace(double _val1, double _val2, int _numPoints)
{
  assert(_numPoints > 1);

  VectorXd res = VectorXd::Zero(_numPoints);

  double delta = (_val2 - _val1) / static_cast<double>(_numPoints);

  res[0] = _val1;
  for (int i = 1; i < _numPoints - 1; ++i)
    res[i] = res[i - 1] + delta;
  res[_numPoints - 1] = _val2;

  return res;
}

VectorXd Controller::diff(const VectorXd& _val)
{
  assert(_val.size() > 1);

  VectorXd res = VectorXd::Zero(_val.size() - 1);

  for (int i = 0; i < _val.size() - 1; ++i)
    res[i] = _val[i + 1] - _val[i];

  return res;
}

