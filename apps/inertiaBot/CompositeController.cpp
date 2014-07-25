/*
 * Copyright (c) 2014, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Jeffrey T. Bingham <bingjeff@gmail.com>,
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

#include "apps/inertiaBot/CompositeController.h"

#include <iostream>

#include "dart/common/Console.h"
#include "dart/dynamics/Skeleton.h"
#include "dart/dynamics/BodyNode.h"
#include "dart/dynamics/Joint.h"

using namespace std;

using namespace Eigen;

using namespace dart;
using namespace dynamics;

CompositeController::CompositeController(
    Skeleton* _skel,
    dart::constraint::ConstraintSolver* _constDyn,
    ControlMode _cm)
  : Controller(_skel, _constDyn),
    mManualController(new ManualController(_skel, _constDyn)),
    mAutomaticController(new AutomaticController(_skel, _constDyn))
{
  setControlMode(_cm);
}

CompositeController::~CompositeController()
{
  if (mManualController)
    delete mManualController;

  if (mAutomaticController)
    delete mAutomaticController;
}

void CompositeController::prestep(double _currentTime)
{
  Controller::prestep(_currentTime);

  mManualController->prestep(_currentTime);
  mAutomaticController->prestep(_currentTime);
}

void CompositeController::update(double _time)
{
  Controller::update(_time);

  // Update
  mCurrentController->update(_time);
}

void CompositeController::changeControlMode(
    CompositeController::ControlMode _cm)
{
  // If current control mode is same with _cm do nothing
  if (mControlMode == _cm)
    return;

  // Change planner
  ControlMode oldCM = mControlMode;
  mCurrentController->deactivate(mCurrentTime);
  setControlMode(_cm);
  mCurrentController->activate(mCurrentTime);

  // Print message
  dtmsg << "CompositeController (t = "
        << mCurrentTime
        << "): mode changed ["
        << getControlModeString(oldCM)
        << "] -> ["
        << getControlModeString(mControlMode)
        << "]"
        << std::endl;
}

const VectorXd& CompositeController::getTorques() const
{
  return mCurrentController->getTorques();
}

double CompositeController::getTorque(int _index) const
{
  return mCurrentController->getTorque(_index);
}

void CompositeController::keyboard(unsigned char _key)
{
  Controller::keyboard(_key);

  mCurrentController->keyboard(_key);
}

string CompositeController::getControlModeString(
    CompositeController::ControlMode _cm)
{
  if (_cm == CM_AUTOMATIC)
    return string("Automatic");
  else if (_cm == CM_MANUAL)
    return string("Manual");
  else
    return string("Unknown");
}

void CompositeController::setControlMode(CompositeController::ControlMode _cm)
{
  mControlMode = _cm;

  if (mControlMode == CM_AUTOMATIC)
  {
    mCurrentController = mAutomaticController;
  }
  else if (mControlMode == CM_MANUAL)
  {
    mCurrentController = mManualController;
  }
  else
  {
    assert(0);
  }
}

AutomaticController* CompositeController::getAutomaticController()
{
  return mAutomaticController;
}

ManualController* CompositeController::getManualController()
{
  return mManualController;
}


