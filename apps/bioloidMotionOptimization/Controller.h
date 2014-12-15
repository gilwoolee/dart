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

#ifndef APPS_BIOLOIDMOTIONOPTIMIZATION_CONTROLLER_H_
#define APPS_BIOLOIDMOTIONOPTIMIZATION_CONTROLLER_H_

#include <Eigen/Dense>

#include "dart/dart.h"

#include "apps/bioloidMotionOptimization/Motion.h"

/*
rootJoint(6)

l_hip(1)
l_thigh(1)
l_shin(1)
l_heel(1)
l_foot(1)

r_hip(1)
r_thigh(1)
r_shin(1)
r_heel(1)
r_foot(1)

l_shoulder(1)
l_arm(1)
l_hand(1)

r_shoulder(1)
r_arm(1)
r_hand(1)
 */

/// \brief Base c
class Controller
{
public:
  /// \brief Constructor
  Controller(dart::dynamics::Skeleton* _skel,
             dart::simulation::World* _world);

  /// \brief Destructor
  virtual ~Controller();

  /// \brief Called once before the simulation.
  virtual void prestep(double _currentTime);

  /// \brief
  virtual void activate(double _currentTime);

  /// \brief
  virtual void deactivate(double _currentTime);

  /// \brief Called before every simulation time step in MyWindow class.
  virtual void update(double _time);

  /// \brief
  virtual const Eigen::VectorXd& getTorques() {}// const = 0;

  /// \brief
  virtual double getTorque(int _index) {}// const = 0;

  /// \brief
  virtual void keyboard(unsigned char _key);

  /// \brief
  virtual dart::dynamics::Skeleton* getSkeleton();

  /// \brief
  void printDebugInfo() const;

protected:
  /// \brief
  dart::dynamics::Skeleton* mSkel;

  /// \brief
  dart::simulation::World* mWorld;

  /// \brief
  double mCurrentTime;
};

#endif  // APPS_BIOLOIDMOTIONOPTIMIZATION_CONTROLLER_H_
