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

#include "ManualController.h"

#include <iostream>

using namespace std;

using namespace Eigen;

using namespace dart;
using namespace dynamics;
using namespace simulation;

ManualController::ManualController(Skeleton* _skel,
    dart::simulation::World* _world)
  : Controller(_skel, _world)
{
  int nDof = _skel->getNumDofs();

  mTorques.resize(mSkel->getNumDofs());
  mTorques.fill(0);
  mDesiredDofs.resize(nDof);
  mKp = MatrixXd::Zero(nDof, nDof);
  mKd = MatrixXd::Zero(nDof, nDof);
  for (int i = 0; i < nDof; i++)
  {
    mDesiredDofs[i] = 0.0;
    mKp(i, i) = 0.5;
    mKd(i, i) = 0.1;
  }
  setHomeDesiredDofs();
}

ManualController::~ManualController()
{
}

void ManualController::prestep(double _currentTime)
{
  Controller::prestep(_currentTime);
}

void ManualController::activate(double _currentTime)
{
  // Set current joint angle as desired angle.
  VectorXd q = mSkel->getPositions();
  double q1 = q[3];
  double q2 = q[4];

  setDesiredDof(0, q1);
  setDesiredDof(1, q2);
}

void ManualController::deactivate(double _currentTime)
{

}

void ManualController::update(double _time)
{
  Controller::update(_time);

  evalTorques();

  mSkel->clearExternalForces();
  mSkel->setForces(VectorXd::Zero(mSkel->getNumDofs()));
  mSkel->clearConstraintImpulses();

//  std::cout << "torque: " << getTorques().transpose() << std::endl;

  mSkel->setForces(getTorques());
}

const VectorXd& ManualController::getTorques() const
{
  return mTorques;
}

double ManualController::getTorque(int _index) const
{
  return mTorques[_index];
}

void ManualController::keyboard(unsigned char _key)
{
  Controller::keyboard(_key);

  const double pi = DART_PI;
  switch(_key)
  {
    case 's': // center, linkage extended
      setHomeDesiredDofs();
      break;
    case 'e': // top-right,
      setHomeDesiredDofs();
      setDesiredDof(6, -0.5*pi); // l_hip
      setDesiredDof(8, -0.5*pi); // r_hip
      setDesiredDof(11, 0.5*pi); // l_arm
      setDesiredDof(13, 0.5*pi); // r_arm
      break;
    case 'w': // top
      setHomeDesiredDofs();
      setDesiredDof(6, -0.5*pi); // l_hip
      setDesiredDof(8, -0.5*pi); // r_hip
      setDesiredDof(11, 0.0*pi); // l_arm
      setDesiredDof(13, 0.0*pi); // r_arm
      break;
    case 'q': // top-left, linkage collapsed (inverted)
      setHomeDesiredDofs();
      setDesiredDof(6, -0.5*pi); // l_hip
      setDesiredDof(8, -0.5*pi); // r_hip
      setDesiredDof(11, -0.5*pi); // l_arm
      setDesiredDof(13, -0.5*pi); // r_arm
      break;
    case 'a': // left
      setHomeDesiredDofs();
      setDesiredDof(6, 0.0*pi); // l_hip
      setDesiredDof(8, 0.0*pi); // r_hip
      setDesiredDof(11, -0.5*pi); // l_arm
      setDesiredDof(13, -0.5*pi); // r_arm
      break;
    case 'z': // bottom-left
      setHomeDesiredDofs();
      setDesiredDof(6, 0.5*pi); // l_hip
      setDesiredDof(8, 0.5*pi); // r_hip
      setDesiredDof(11, -0.5*pi); // l_arm
      setDesiredDof(13, -0.5*pi); // r_arm
      break;
    case 'x': // bottom, left link extended
      setHomeDesiredDofs();
      setDesiredDof(6, 0.5*pi); // l_hip
      setDesiredDof(8, 0.5*pi); // r_hip
      setDesiredDof(11, 0.0*pi); // l_arm
      setDesiredDof(13, 0.0*pi); // r_arm
      break;
    case 'c': // bottom-right, linkage collapsed
      setHomeDesiredDofs();
      setDesiredDof(6, 0.5*pi); // l_hip
      setDesiredDof(8, 0.5*pi); // r_hip
      setDesiredDof(11, 0.5*pi); // l_arm
      setDesiredDof(13, 0.5*pi); // r_arm
      break;
    case 'd': // right, right link extended
      setHomeDesiredDofs();
      setDesiredDof(6, 0.0*pi); // l_hip
      setDesiredDof(8, 0.0*pi); // r_hip
      setDesiredDof(11, 0.5*pi); // l_arm
      setDesiredDof(13, 0.5*pi); // r_arm
      break;
    default:
      break;
  }
}

void ManualController::setZeroDesiredDofs()
{
  mDesiredDofs.setZero();
}

void ManualController::setHomeDesiredDofs()
{
  mDesiredDofs.setZero();

  const double pi = DART_PI;

  setDesiredDof(7, 1.0*pi);
  setDesiredDof(9, 1.0*pi);

  setDesiredDof(15, -0.5*pi);
  setDesiredDof(17, -0.5*pi);
}

void ManualController::setDesiredDof(int _index, double _val)
{
  mDesiredDofs[_index] = _val;
}

const VectorXd& ManualController::getDesiredDofs() const
{
  return mDesiredDofs;
}

const VectorXd& ManualController::getKp() const
{
  return mKp;
}

const VectorXd& ManualController::getKd() const
{
  return mKd;
}

void ManualController::evalTorques()
{ 
  VectorXd _q  = mSkel->getPositions();
  VectorXd _dq = mSkel->getVelocities();

  // Solve for the appropriate joint torques
//  mTorques[6] = -mKp[0]*(_q[6]-mDesiredDofs[0]) - mKd[0]*_dq[6];
//  mTorques[7] = -mKp[1]*(_q[7]-mDesiredDofs[1]) - mKd[1]*_dq[7];
//  mTorques[3] = -mKp[0]*(_q[3]-mDesiredDofs[0]) - mKd[0]*_dq[3];
//  mTorques[4] = -mKp[1]*(_q[4]-mDesiredDofs[1]) - mKd[1]*_dq[4];
  //std::cout << _dof.transpose() << mDesiredDofs.transpose() << std::endl;
  //std::cout << mTorques.transpose() << std::endl;

  mTorques = -mKp * (_q - mDesiredDofs) - mKd * _dq;

  mTorques.head(6) = Eigen::Vector6d::Zero();
}

