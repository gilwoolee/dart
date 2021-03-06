/*
 * Copyright (c) 2014, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Jeongseok Lee <jslee02@gmail.com>
 *
 * Geoorgia Tech Graphics Lab and Humanoid Robotics Lab
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

#include "dart/constraint/JointLimitConstraint.h"

#include <iostream>

#include "dart/common/Console.h"
#include "dart/dynamics/BodyNode.h"
#include "dart/dynamics/Joint.h"
#include "dart/dynamics/Skeleton.h"
#include "dart/lcpsolver/lcp.h"

#define DART_ERROR_ALLOWANCE 0.0
#define DART_ERP     0.01
#define DART_MAX_ERV 1e+1
#define DART_CFM     1e-9

namespace dart {
namespace constraint {

double JointLimitConstraint::mErrorAllowance            = DART_ERROR_ALLOWANCE;
double JointLimitConstraint::mErrorReductionParameter   = DART_ERP;
double JointLimitConstraint::mMaxErrorReductionVelocity = DART_MAX_ERV;
double JointLimitConstraint::mConstraintForceMixing     = DART_CFM;

//==============================================================================
JointLimitConstraint::JointLimitConstraint(dynamics::Joint* _joint)
  : Constraint(),
    mJoint(_joint),
    mBodyNode(NULL),
    mAppliedImpulseIndex(0)
{
  assert(_joint);

  mLifeTime[0] = 0;
  mLifeTime[1] = 0;
  mLifeTime[2] = 0;
  mLifeTime[3] = 0;
  mLifeTime[4] = 0;
  mLifeTime[5] = 0;

  mActive[0] = false;
  mActive[1] = false;
  mActive[2] = false;
  mActive[3] = false;
  mActive[4] = false;
  mActive[5] = false;

  // TODO(JS): We need to be able to access child body node from joint
  for (int i = 0; i < mJoint->getSkeleton()->getNumBodyNodes(); ++i)
  {
    dynamics::BodyNode* bodyNode = mJoint->getSkeleton()->getBodyNode(i);
    if (bodyNode->getParentJoint() == mJoint)
      mBodyNode = bodyNode;
  }

  assert(mBodyNode);
}

//==============================================================================
JointLimitConstraint::~JointLimitConstraint()
{
}

//==============================================================================
void JointLimitConstraint::setErrorAllowance(double _allowance)
{
  // Clamp error reduction parameter if it is out of the range
  if (_allowance < 0.0)
  {
    dtwarn << "Error reduction parameter[" << _allowance
           << "] is lower than 0.0. "
           << "It is set to 0.0." << std::endl;
    mErrorAllowance = 0.0;
  }

  mErrorAllowance = _allowance;
}

//==============================================================================
double JointLimitConstraint::getErrorAllowance()
{
  return mErrorAllowance;
}

//==============================================================================
void JointLimitConstraint::setErrorReductionParameter(double _erp)
{
  // Clamp error reduction parameter if it is out of the range [0, 1]
  if (_erp < 0.0)
  {
    dtwarn << "Error reduction parameter[" << _erp << "] is lower than 0.0. "
           << "It is set to 0.0." << std::endl;
    mErrorReductionParameter = 0.0;
  }
  if (_erp > 1.0)
  {
    dtwarn << "Error reduction parameter[" << _erp << "] is greater than 1.0. "
           << "It is set to 1.0." << std::endl;
    mErrorReductionParameter = 1.0;
  }

  mErrorReductionParameter = _erp;
}

//==============================================================================
double JointLimitConstraint::getErrorReductionParameter()
{
  return mErrorReductionParameter;
}

//==============================================================================
void JointLimitConstraint::setMaxErrorReductionVelocity(double _erv)
{
  // Clamp maximum error reduction velocity if it is out of the range
  if (_erv < 0.0)
  {
    dtwarn << "Maximum error reduction velocity[" << _erv
           << "] is lower than 0.0. "
           << "It is set to 0.0." << std::endl;
    mMaxErrorReductionVelocity = 0.0;
  }

  mMaxErrorReductionVelocity = _erv;
}

//==============================================================================
double JointLimitConstraint::getMaxErrorReductionVelocity()
{
  return mMaxErrorReductionVelocity;
}

//==============================================================================
void JointLimitConstraint::setConstraintForceMixing(double _cfm)
{
  // Clamp constraint force mixing parameter if it is out of the range
  if (_cfm < 1e-9)
  {
    dtwarn << "Constraint force mixing parameter[" << _cfm
           << "] is lower than 1e-9. " << "It is set to 1e-9." << std::endl;
    mConstraintForceMixing = 1e-9;
  }
  if (_cfm > 1.0)
  {
    dtwarn << "Constraint force mixing parameter[" << _cfm
           << "] is greater than 1.0. " << "It is set to 1.0." << std::endl;
    mConstraintForceMixing = 1.0;
  }

  mConstraintForceMixing = _cfm;
}

//==============================================================================
double JointLimitConstraint::getConstraintForceMixing()
{
  return mConstraintForceMixing;
}

//==============================================================================
void JointLimitConstraint::update()
{
  // Reset dimention
  mDim = 0;

  dynamics::GenCoord* genCoord;

  size_t dof = mJoint->getNumGenCoords();
  for (size_t i = 0; i < dof; ++i)
  {
    genCoord = mJoint->getGenCoord(i);

    // Lower bound check
    mViolation[i] = genCoord->getPos() - genCoord->getPosMin();
    if (mViolation[i] <= 0.0)
    {
      mNegativeVel[i] = -genCoord->getVel();

      mLowerBound[i] = 0.0;
      mUpperBound[i] = dInfinity;

      if (mActive[i])
      {
        ++(mLifeTime[i]);
      }
      else
      {
        mActive[i] = true;
        mLifeTime[i] = 0;
      }

      ++mDim;
      continue;
    }

    // Upper bound check
    mViolation[i] = genCoord->getPos() - genCoord->getPosMax();
    if (mViolation[i] >= 0.0)
    {
      mNegativeVel[i] = -genCoord->getVel();

      mLowerBound[i] = -dInfinity;
      mUpperBound[i] = 0.0;

      if (mActive[i])
      {
        ++(mLifeTime[i]);
      }
      else
      {
        mActive[i] = true;
        mLifeTime[i] = 0;
      }

      ++mDim;
      continue;
    }

    mActive[i] = false;
  }
}

//==============================================================================
void JointLimitConstraint::getInformation(ConstraintInfo* _lcp)
{
  size_t index = 0;
  size_t dof = mJoint->getNumGenCoords();
  for (size_t i = 0; i < dof; ++i)
  {
    if (mActive[i] == false)
      continue;

    assert(_lcp->w[index] == 0.0);

    double bouncingVel = -mViolation[i];

    if (bouncingVel > 0.0)
      bouncingVel = -mErrorAllowance;
    else
      bouncingVel = +mErrorAllowance;

    bouncingVel *= _lcp->invTimeStep * mErrorReductionParameter;

    if (bouncingVel > mMaxErrorReductionVelocity)
      bouncingVel = mMaxErrorReductionVelocity;

    _lcp->b[index] = mNegativeVel[i] + bouncingVel;

    _lcp->lo[index] = mLowerBound[i];
    _lcp->hi[index] = mUpperBound[i];

    if (_lcp->lo[index] > _lcp->hi[index])
    {
      std::cout << "dim: " << mDim << std::endl;
      std::cout << "lb: " << mLowerBound[i] << std::endl;
      std::cout << "ub: " << mUpperBound[i] << std::endl;
      std::cout << "lb: " << _lcp->lo[index] << std::endl;
      std::cout << "ub: " << _lcp->hi[index] << std::endl;
    }

    assert(_lcp->findex[index] == -1);

    if (mLifeTime[i])
      _lcp->x[index] = mOldX[i];
    else
      _lcp->x[index] = 0.0;

    index++;
  }
}

//==============================================================================
void JointLimitConstraint::applyUnitImpulse(size_t _index)
{
  assert(_index < mDim && "Invalid Index.");

  size_t localIndex = 0;
  dynamics::Skeleton* skeleton = mJoint->getSkeleton();

  size_t dof = mJoint->getNumGenCoords();
  for (size_t i = 0; i < dof; ++i)
  {
    if (mActive[i] == false)
      continue;

    if (localIndex == _index)
    {
      skeleton->clearConstraintImpulses();
      mJoint->getGenCoord(i)->setConstraintImpulse(1.0);
      skeleton->updateBiasImpulse(mBodyNode);
      skeleton->updateVelocityChange();
    }

    ++localIndex;
  }

  mAppliedImpulseIndex = _index;
}

//==============================================================================
void JointLimitConstraint::getVelocityChange(double* _delVel, bool _withCfm)
{
  assert(_delVel != NULL && "Null pointer is not allowed.");

  size_t localIndex = 0;
  size_t dof = mJoint->getNumGenCoords();
  for (size_t i = 0; i < dof ; ++i)
  {
    if (mActive[i] == false)
      continue;

    if (mJoint->getSkeleton()->isImpulseApplied())
      _delVel[localIndex] = mJoint->getGenCoord(i)->getVelChange();
    else
      _delVel[localIndex] = 0.0;

    ++localIndex;
  }

  // Add small values to diagnal to keep it away from singular, similar to cfm
  // varaible in ODE
  if (_withCfm)
  {
    _delVel[mAppliedImpulseIndex] += _delVel[mAppliedImpulseIndex]
                                     * mConstraintForceMixing;
  }

  assert(localIndex == mDim);
}

//==============================================================================
void JointLimitConstraint::excite()
{
  mJoint->getSkeleton()->setImpulseApplied(true);
}

//==============================================================================
void JointLimitConstraint::unexcite()
{
  mJoint->getSkeleton()->setImpulseApplied(false);
}

//==============================================================================
void JointLimitConstraint::applyImpulse(double* _lambda)
{
  size_t localIndex = 0;
  size_t dof = mJoint->getNumGenCoords();
  for (size_t i = 0; i < dof ; ++i)
  {
    if (mActive[i] == false)
      continue;

//    std::cout << "lambda[" << localIndex << "]: " << _lambda[_idx + localIndex]
//              << std::endl;

//    std::cout << "mJoint->getGenCoord(i)->getConstraintImpulse(): "
//              << mJoint->getGenCoord(i)->getConstraintImpulse()
//              << std::endl;

    mJoint->getGenCoord(i)->setConstraintImpulse(
//          mJoint->getGenCoord(i)->getConstraintImpulse()
//          +
          _lambda[localIndex]);

//    std::cout << "mJoint->getGenCoord(i)->getConstraintImpulse(): "
//              << mJoint->getGenCoord(i)->getConstraintImpulse()
//              << std::endl;

    mOldX[i] = _lambda[localIndex];

    ++localIndex;
  }
}

//==============================================================================
dynamics::Skeleton* JointLimitConstraint::getRootSkeleton() const
{
  return mJoint->getSkeleton()->mUnionRootSkeleton;
}

//==============================================================================
bool JointLimitConstraint::isActive() const
{
  for (size_t i = 0; i < 6; ++i)
  {
    if (mActive[i])
      return true;
  }

  return false;
}

} // namespace constraint
} // namespace dart
