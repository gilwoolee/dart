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

#ifndef APPS_BIOLOIDMOTIONOPTIMIZATION_MOTION_H_
#define APPS_BIOLOIDMOTIONOPTIMIZATION_MOTION_H_

#include <Eigen/Dense>
#include <unsupported/Eigen/Splines>

#include "dart/dart.h"

/// \brief
class LinearInterpolationMotion
{
public:
  /// \brief
  LinearInterpolationMotion(dart::dynamics::Skeleton* _skel);

  /// \brief
  virtual ~LinearInterpolationMotion();

  /// \brief
  /// \param [in] _rootTrient Predicted root orientation
  void setKeyConfig(const Eigen::VectorXd& _rootOrient,
                    const Eigen::VectorXd& _q1,
                    const Eigen::VectorXd& _q2,
                    const Eigen::VectorXd& _time);

  /// \brief
  int getNumKeyConfig() const;

  /// \brief
  double getKeyPredictedOrientation(int _idx);

  /// \brief
  Eigen::Vector2d getKeyConfig(int _idx);

  /// \brief
  double getKeyTime(int _idx);

  /// \brief
  double getPredictedOrientation(double _time);

  /// \brief
  Eigen::Vector2d getConfigAtTime(double _time);

  /// \brief
  Eigen::Vector2d getVelocityAtTime(double _time, double _dt = 1e-8);

  /// \brief
  Eigen::Vector2d getAccelerationAtTime(double _time, double _dt = 1e-8);

protected:
  /// \brief
  Eigen::Vector2d _getConfigAtTimeLinear(double _time);

  /// \brief
  Eigen::Vector2d _getConfigAtTimeCubicBezierCurve(double _time);

  /// \brief
  Eigen::Vector2d _getConfigAtTime4thBezierCurve(double _time);

protected:
  /// \brief
  Eigen::VectorXd mQ1;

  /// \brief
  Eigen::VectorXd mQ2;

  /// \brief
  Eigen::VectorXd mTime;

  /// \brief
  Eigen::VectorXd mPredictedOrientation;

private:
  /// \brief
  double mTi;

  /// \brief
  double mTf;

  /// \brief
  double mDuration;

  /// \brief
  dart::dynamics::Skeleton* mSkel;
};

class BezierCurveMotion
{
public:
  BezierCurveMotion(dart::dynamics::Skeleton* _skel);
  virtual ~BezierCurveMotion();

  void setControlPoint(int _i, int _j, double _val);

  double getPoint(int i, double _t) const;

protected:
  Eigen::MatrixXd mControlPoints;

private:
  double computePointOnBezierCurve(const Eigen::Vector4d& _ctrPts,
                                   double _t) const;
};

#endif  // APPS_BIOLOIDMOTIONOPTIMIZATION_MOTION_H_
