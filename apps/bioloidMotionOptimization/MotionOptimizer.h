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

#ifndef APPS_BIOLOIDMOTIONOPTIMIZATION_MOTIONOPTIMIZER_H_
#define APPS_BIOLOIDMOTIONOPTIMIZATION_MOTIONOPTIMIZER_H_

#include <Eigen/Dense>

#include "dart/dart.h"

#include "Motion.h"

/// \brief Base c
class MotionOptimizer
{
public:
  /// \brief Constructor
  MotionOptimizer(dart::dynamics::Skeleton* _robot);

  /// \brief Destructor
  virtual ~MotionOptimizer();

  void optimize();
  void resetMotion();
  void playback();

  dart::dynamics::Skeleton* getSkeleton() { return mSkel; }

  BezierCurveMotion* getMotion() { return mMotion; }

protected:

  class ObjFunc : public dart::optimizer::Function
  {
  public:
    /// \brief Constructor
    ObjFunc(MotionOptimizer* _motionOptimizer);

    /// \brief Destructor
    virtual ~ObjFunc();

    /// \copydoc Function::eval
    virtual double eval(Eigen::Map<const Eigen::VectorXd>& _x);

    /// \copydoc Function::evalGradient
    virtual void evalGradient(Eigen::Map<const Eigen::VectorXd>& _x,
                              Eigen::Map<Eigen::VectorXd> _grad);

  protected:
    MotionOptimizer* mMotionOptimizer;
  };

  class ConstFunc : public dart::optimizer::Function
  {
  public:
    /// \brief Constructor
    ConstFunc(MotionOptimizer* _motionOptimizer);

    /// \brief Destructor
    virtual ~ConstFunc();

    /// \copydoc Function::eval
    virtual double eval(Eigen::Map<const Eigen::VectorXd>& _x);

    /// \copydoc Function::evalGradient
    virtual void evalGradient(Eigen::Map<const Eigen::VectorXd>& _x,
                              Eigen::Map<Eigen::VectorXd> _grad);

  private:
    MotionOptimizer* mMotionOptimizer;
  };

  dart::dynamics::Skeleton* mSkel;

  BezierCurveMotion* mMotion;

  double mTime;
};

#endif  // APPS_BIOLOIDMOTIONOPTIMIZATION_MOTIONOPTIMIZER_H_
