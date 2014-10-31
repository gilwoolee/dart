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

#ifndef APPS_INERTIAPLANNING_CONTROLLER_H_
#define APPS_INERTIAPLANNING_CONTROLLER_H_

#include <Eigen/Dense>
#include <vector>

#include "dart/dynamics/BodyNode.h"
#include "dart/dynamics/Joint.h"
#include "dart/dynamics/Skeleton.h"
#include "dart/constraint/ConstraintSolver.h"

class Controller
{
public:
  Controller(dart::dynamics::Skeleton*_skel);
  virtual ~Controller() {}

  void setInitState(double q0, double q1, double w0, double w1);

  void setFinalState(double q0, double q1, double w0, double w1);

  void setDesiredRotation(double desiredRotation);

  void setDuration(double duration);

  double getDuration() const { return mDuration; }

  void init();

  void off() { mOff = true; }

  void update(double time);

  Eigen::VectorXd getTorques() { return mTorques; }

  double getTorque(int _index) { return mTorques[_index]; }

  dart::dynamics::Skeleton* getSkel() { return mSkel; }

  /// \brief Compute linearly spaced vectors
  static Eigen::VectorXd linspace(double _val1, double _val2, int _numPoints);

  /// \brief Compute difference
  static Eigen::VectorXd diff(const Eigen::VectorXd& _val);

protected:
  ///
  dart::dynamics::Skeleton* mSkel;

  Eigen::VectorXd mTorques;

  double mQ0_i;
  double mQ1_i;

  double mW0_i;
  double mW1_i;

  double mQ0_f;
  double mQ1_f;

  double mW0_f;
  double mW1_f;

  double mDesiredRotation;

  double mDuration;

  double mDesiredAngularVelocity;

  double mH;

  bool mOff;
};

#endif  // APPS_INERTIAPLANNING_CONTROLLER_H_
