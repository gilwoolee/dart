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

#ifndef APPS_INERTIABOT_COMPOSITECONTROLLER_H_
#define APPS_INERTIABOT_COMPOSITECONTROLLER_H_

#include <Eigen/Dense>

#include "dart/math/MathTypes.h"

#include "apps/inertiaBot/Controller.h"
#include "apps/inertiaBot/ManualController.h"
#include "apps/inertiaBot/AutomaticController.h"
#include "apps/inertiaBot/Motion.h"

namespace dart {
namespace dynamics {
class Skeleton;
}  // namespace dynamics
namespace constraint {
class ConstraintSolver;
}
}  // namespace dart

class CompositeController : public Controller
{
public:
  enum ControlMode
  {
    CM_AUTOMATIC,
    CM_MANUAL
  };

  /// \brief Constructor
  CompositeController(dart::dynamics::Skeleton* _skel,
                      dart::constraint::ConstraintSolver* _constDyn,
                      ControlMode _cm = CM_AUTOMATIC);

  /// \brief Destructor
  virtual ~CompositeController();

  //------------------------- class Controller ---------------------------------
  // Documentation inherited.
  virtual void prestep(double _currentTime);

  // Documentation inherited.
  virtual void update(double _time);

  // Documentation inherited.
  virtual const Eigen::VectorXd& getTorques() const;

  // Documentation inherited.
  virtual double getTorque(int _index) const;

  // Documentation inherited.
  virtual void keyboard(unsigned char _key);

  //----------------------------------------------------------------------------
  /// \brief Change control mode (Manual <--> Automatic)
  void changeControlMode(CompositeController::ControlMode _cm);

  /// \brief
  AutomaticController* getAutomaticController();

  /// \brief
  ManualController* getManualController();

protected:
  /// \brief
  ControlMode mControlMode;

  /// \brief
  Controller* mCurrentController;

  /// \brief
  ManualController* mManualController;

  /// \brief
  AutomaticController* mAutomaticController;

private:
  /// \brief
  std::string getControlModeString(ControlMode _cm);

  /// \brief
  void setControlMode(ControlMode _cm);

};

#endif  // APPS_INERTIABOT_COMPOSITECONTROLLER_H_
