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

#ifndef APPS_BIOLOID_MANUALCONTROLLER_H_
#define APPS_BIOLOID_MANUALCONTROLLER_H_

#include <Eigen/Dense>

#include "dart/dart.h"

#include "apps/bioloid/Controller.h"

/// \brief
class ManualController : public Controller
{
public:
  /// \brief Constructor
  ManualController(dart::dynamics::Skeleton* _skel,
                   dart::simulation::World* _world);

  /// \brief Destructor
  virtual ~ManualController();

  //------------------------- class Controller ---------------------------------
  // Documentation inherited.
  virtual void prestep(double _currentTime);

  // Documentation inherited.
  virtual void activate(double _currentTime);

  // Documentation inherited.
  virtual void deactivate(double _currentTime);

  // Documentation inherited.
  virtual void update(double _time);

  // Documentation inherited.
  virtual const Eigen::VectorXd& getTorques() const;

  // Documentation inherited.
  virtual double getTorque(int _index) const;

  // Documentation inherited.
  virtual void keyboard(unsigned char _key);

  //----------------------------------------------------------------------------
  /// \brief
  void setZeroDesiredDofs();

  /// \brief
  void setHomeDesiredDofs();

  /// \brief
  void setDesiredDof(int _index, double _val);

  /// \brief
  const Eigen::VectorXd& getDesiredDofs() const;

  /// \brief
  const Eigen::VectorXd& getKp() const;

  /// \brief
  const Eigen::VectorXd& getKd() const;

protected:
  /// \brief
  void evalTorques();

protected:
  /// \brief
  Eigen::VectorXd mTorques;

  /// \brief
  Eigen::VectorXd mDesiredDofs;

  /// \brief
  Eigen::MatrixXd mKp;

  /// \brief
  Eigen::MatrixXd mKd;
};

#endif  // APPS_BIOLOID_MANUALCONTROLLER_H_
