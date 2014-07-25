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

#include "apps/inertiaBot/Motion.h"

#include <iostream>

#include "dart/common/Console.h"
#include "dart/dynamics/Skeleton.h"
#include "dart/dynamics/BodyNode.h"
#include "dart/dynamics/Joint.h"
#include "apps/inertiaBot/AutomaticController.h"

using namespace std;

using namespace Eigen;

using namespace dart;
using namespace dynamics;

LinearInterpolationMotion::LinearInterpolationMotion(
    Skeleton* _skel)
  : mSkel(_skel)
{
}

LinearInterpolationMotion::~LinearInterpolationMotion()
{
}

void LinearInterpolationMotion::setKeyConfig(const VectorXd& _rootOrient,
                                             const VectorXd& _q1,
                                             const VectorXd& _q2,
                                             const VectorXd& _time)
{
  assert(_q1.size() == _q2.size());
  assert(_q2.size() == _time.size());
  assert(_time.size() == _q1.size());

#ifndef NDEBUG
  for (int i = 0; i < _time.size() - 1; ++i)
    assert(_time[i] <= _time[i + 1]);
#endif

  mPredictedOrientation = _rootOrient;
  mQ1 = _q1;
  mQ2 = _q2;
  mTime = _time;

  mTi = _time[0];
  mTf = _time[_time.size() - 1];
  mDuration = mTf - mTi;
}

double LinearInterpolationMotion::getKeyPredictedOrientation(int _idx)
{
  assert(0 <= _idx && _idx < mQ1.size());
  return mPredictedOrientation[_idx];
}

int LinearInterpolationMotion::getNumKeyConfig() const
{
  return mQ1.size();
}

Vector2d LinearInterpolationMotion::getKeyConfig(int _idx)
{
  assert(0 <= _idx && _idx < mQ1.size());
  return Vector2d(mQ1[_idx], mQ2[_idx]);
}

double LinearInterpolationMotion::getKeyTime(int _idx)
{
  assert(0 <= _idx && _idx < mQ1.size());
  return mTime[_idx];
}

double LinearInterpolationMotion::getPredictedOrientation(double _time)
{
//  assert(mTi <= _time && _time <= mTf);
  if (_time < mTime[0])
    return getKeyPredictedOrientation(0);

  if (mTime[mTime.size() - 1] <= _time)
    return getKeyPredictedOrientation(mTime.size() - 1);

  for (int i = 1; i < mTime.size(); ++i)
  {
    if (_time < mTime[i])
    {
      double duration = mTime[i] - mTime[i - 1];
      double alpha    = (_time - mTime[i - 1]) / duration;
      double beta     = 1.0 - alpha;

      double v0 = getKeyPredictedOrientation(i - 1);
      double v1 = getKeyPredictedOrientation(i);

      return beta*beta*beta*v0
          + 3.0*beta*beta*alpha*v0
          + 3.0*beta*alpha*alpha*v1
          + alpha*alpha*alpha*v1;

//      return (beta * getKeyPredictedOrientation(i - 1)
//              + alpha * getKeyPredictedOrientation(i));
    }
  }
}

Vector2d LinearInterpolationMotion::getPositionsAtTime(double _time)
{
//  return _getPositionsAtTimeLinear(_time);
  return _getPositionsAtTimeCubicBezierCurve(_time);
//  return _getPositionsAtTime4thBezierCurve(_time);
}

Vector2d LinearInterpolationMotion::_getPositionsAtTimeLinear(double _time)
{
//  assert(mTi <= _time && _time <= mTf);

  if (_time < mTime[0])
    return getKeyConfig(0);

  if (mTime[mTime.size() - 1] <= _time)
    return getKeyConfig(mTime.size() - 1);

  for (int i = 1; i < mTime.size(); ++i)
  {
    if (_time < mTime[i])
    {
      double duration = mTime[i] - mTime[i - 1];
      double alpha    = (_time - mTime[i - 1]) / duration;
      double beta     = 1.0 - alpha;

      return (beta * getKeyConfig(i - 1) + alpha * getKeyConfig(i));
    }
  }

  assert(0);
}

Vector2d LinearInterpolationMotion::_getPositionsAtTimeCubicBezierCurve(
    double _time)
{
  if (_time < mTime[0])
    return getKeyConfig(0);

  if (mTime[mTime.size() - 1] <= _time)
    return getKeyConfig(mTime.size() - 1);

  int idx = 0;
  for (int i = 1; i < mTime.size(); ++i)
  {
    if (_time < mTime[i])
    {
      idx = i - 1;
      double duration = mTime[i] - mTime[i - 1];

//      if (duration == 0.0)
//        continue;

      // t
      double alpha    = (_time - mTime[i - 1]) / duration;
      // (1-t)
      double beta     = 1.0 - alpha;

      return beta*beta*beta*getKeyConfig(i - 1)
          + 3.0*beta*beta*alpha*getKeyConfig(i - 1)
          + 3.0*beta*alpha*alpha*getKeyConfig(i)
          + alpha*alpha*alpha*getKeyConfig(i);

//      return (beta * getKeyConfig(i - 1) + alpha * getKeyConfig(i));
    }
  }

  assert(0);
}

Vector2d LinearInterpolationMotion::_getPositionsAtTime4thBezierCurve(
    double _time)
{
  std::cout << "Over here" << std::endl;

  if (_time < mTime[0])
    return getKeyConfig(0);

  if (mTime[mTime.size() - 1] <= _time)
    return getKeyConfig(mTime.size() - 1);

  for (int i = 1; i < mTime.size(); ++i)
  {
    if (_time < mTime[i])
    {
      double duration = mTime[i] - mTime[i - 1];

      // t
      double alpha    = (_time - mTime[i - 1]) / duration;
      // (1-t)
      double beta     = 1.0 - alpha;

      return beta*beta*beta*beta*getKeyConfig(i - 1)
          + 4.0*beta*beta*beta*alpha*getKeyConfig(i - 1)
          + 6.0*beta*beta*alpha*alpha*0.5*(getKeyConfig(i - 1) + getKeyConfig(i))
          + 4.0*beta*alpha*alpha*alpha*getKeyConfig(i)
          + alpha*alpha*alpha*alpha*getKeyConfig(i);
    }
  }

  assert(0);
}

Eigen::Vector2d LinearInterpolationMotion::getVelocityAtTime(double _time,
                                                             double _dt)
{
  return (getPositionsAtTime(_time + 0.5 * _dt)
          - getPositionsAtTime(_time - 0.5 * _dt))
         / _dt;
}

Vector2d LinearInterpolationMotion::getAccelerationAtTime(double _time,
                                                          double _dt)
{
  return (getVelocityAtTime(_time + 0.5 * _dt)
          - getVelocityAtTime(_time - 0.5 * _dt))
         / _dt;
}
