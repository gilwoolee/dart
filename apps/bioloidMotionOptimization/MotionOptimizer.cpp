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

#include "MotionOptimizer.h"

#include <iostream>

using namespace std;

using namespace Eigen;

using namespace dart;
using namespace constraint;
using namespace dynamics;
using namespace simulation;
using namespace optimizer;

//==============================================================================
MotionOptimizer::MotionOptimizer(Skeleton* _robot)
  : mSkel(_robot),
    mMotion(new BezierCurveMotion(_robot))
{
  assert(_robot != nullptr);
}

//==============================================================================
MotionOptimizer::~MotionOptimizer()
{
  delete mMotion;
}

//==============================================================================
void MotionOptimizer::optimize()
{
  Problem prob(16);

  Eigen::Matrix<double, 16, 1> lb;
  Eigen::Matrix<double, 16, 1> ub;
//  lb << -DART_PI, -DART_PI, -DART_PI, -DART_PI;
//  ub << +DART_PI, +DART_PI, +DART_PI, +DART_PI;
  for (int i = 0; i < 16; ++i)
  {
    lb[i] = -DART_PI*0.5;
    ub[i] = +DART_PI*0.5;
  }

  prob.setLowerBounds(lb);
  prob.setUpperBounds(ub);

  ObjFunc obj(this);
  prob.setObjective(&obj);

  NloptSolver solver(&prob, NLOPT_LN_COBYLA);
  solver.solve();

  double minFunc = prob.getOptimumValue();
  Eigen::VectorXd optX = prob.getOptimalSolution();
  assert(optX.size() == 16);

  std::cout << "minFunc: " << minFunc << std::endl;
}

//==============================================================================
void MotionOptimizer::resetMotion()
{
  mTime = 0.0;
}

//==============================================================================
void MotionOptimizer::playback()
{
  if (mTime > 1.0)
    return;

  int dof = mSkel->getNumDofs();
  Eigen::VectorXd q = Eigen::VectorXd::Zero(dof);

  q[6] = mMotion->getPoint(0, mTime);
  q[8] = mMotion->getPoint(1, mTime);
  q[11] = mMotion->getPoint(2, mTime);
  q[13] = mMotion->getPoint(3, mTime);

  mSkel->setPositions(q);

  mTime += 0.001;
}

//==============================================================================
MotionOptimizer::ObjFunc::ObjFunc(MotionOptimizer* _motionOptimizer)
  : Function(),
    mMotionOptimizer(_motionOptimizer)
{
  assert(mMotionOptimizer != nullptr);
}

//==============================================================================
MotionOptimizer::ObjFunc::~ObjFunc()
{}

//==============================================================================
double MotionOptimizer::ObjFunc::eval(Eigen::Map<const VectorXd>& _x)
{
  double eval = 0.0;

  // Generate motion given _x
  BezierCurveMotion* motion = mMotionOptimizer->getMotion();

  for (int i = 0; i < 4; ++i)
  {
    for (int j = 0; j < 4; ++j)
    {
      motion->setControlPoint(i, j, _x[i*4 + j]);
    }
  }

  double time = 0.0;

  // Simulate the motion evaluating the total effort
  for (int i = 0; i < 1000; ++i)
  {
    dart::dynamics::Skeleton* skel = mMotionOptimizer->getSkeleton();

    int dof = skel->getNumDofs();
    Eigen::VectorXd q = Eigen::VectorXd::Zero(dof);

    q[6] = motion->getPoint(0, time);
    q[8] = motion->getPoint(1, time);
    q[11] = motion->getPoint(2, time);
    q[13] = motion->getPoint(3, time);

    skel->setPositions(q);

    skel->computeForwardDynamics();
    skel->integrateVelocities(skel->getTimeStep());
    skel->integratePositions(skel->getTimeStep());

    Eigen::VectorXd tau = skel->getForces();

    eval += tau.norm();

    time += 0.001;
  }

  // Evaluate cost with the total effort

  return eval;
}

//==============================================================================
void MotionOptimizer::ObjFunc::evalGradient(Eigen::Map<const VectorXd>& _x,
                                            Eigen::Map<VectorXd> _grad)
{
}

//==============================================================================
MotionOptimizer::ConstFunc::ConstFunc(MotionOptimizer* _motionOptimizer)
  : Function(),
    mMotionOptimizer(_motionOptimizer)
{
  assert(mMotionOptimizer != nullptr);
}

//==============================================================================
MotionOptimizer::ConstFunc::~ConstFunc()
{
}

//==============================================================================
double MotionOptimizer::ConstFunc::eval(Eigen::Map<const VectorXd>& _x)
{
  return 0;
}

//==============================================================================
void MotionOptimizer::ConstFunc::evalGradient(Eigen::Map<const VectorXd>& _x,
                                              Eigen::Map<VectorXd> _grad)
{
}
