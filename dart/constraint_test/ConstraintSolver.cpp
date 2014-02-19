/*
 * Copyright (c) 2014, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Karen Liu <karenliu@cc.gatech.edu>,
 *            Jeongseok Lee <jslee02@gmail.com>
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

#include "dart/constraint_test/ConstraintSolver.h"

#include "dart/common/Console.h"
#include "dart/dynamics/Skeleton.h"
#include "dart/collision/fcl_mesh/FCLMeshCollisionDetector.h"
#include "dart/constraint_test/ConstrainedGroup.h"
#include "dart/constraint_test/BallJointConstraint.h"
#include "dart/constraint_test/ClosedLoopConstraint.h"
#include "dart/constraint_test/ContactConstraint.h"
#include "dart/constraint_test/JointLimitConstraint.h"
#include "dart/constraint_test/RevoluteJointConstraint.h"
#include "dart/constraint_test/WeldJointConstraint.h"

namespace dart {
namespace constraint {

using namespace dynamics;

//==============================================================================
ConstraintSolverTEST::ConstraintSolverTEST(const std::vector<dynamics::Skeleton*>& _skeletons,
    double _timeStep,
    bool   _useODE)
  : mTimeStep(_timeStep),
    mUseODE(_useODE),
    mCollisionDetector(new collision::FCLMeshCollisionDetector())
{
  _init();
}

//==============================================================================
ConstraintSolverTEST::~ConstraintSolverTEST()
{
  delete mCollisionDetector;
}

//==============================================================================
void ConstraintSolverTEST::addSkeleton(Skeleton* _skeleton)
{
  assert(_skeleton != NULL
      && "Null pointer skeleton is now allowed to add to ConstraintSover.");

  if (_containSkeleton(_skeleton) == false)
  {
    mSkeletons.push_back(_skeleton);
    _init();
  }
  else
  {
    dtwarn << "Skeleton [" << _skeleton->getName()
           << "] is already in ConstraintSolver." << std::endl;
  }
}

//==============================================================================
void ConstraintSolverTEST::addSkeletons(const std::vector<Skeleton*>& _skeletons)
{
  int numAddedSkeletons = 0;

  for (std::vector<Skeleton*>::const_iterator it = _skeletons.begin();
       it != _skeletons.end(); ++it)
  {
    assert(*it != NULL
        && "Null pointer skeleton is now allowed to add to ConstraintSover.");

    if (_containSkeleton(*it) == false)
    {
      mSkeletons.push_back(*it);
      ++numAddedSkeletons;
    }
    else
    {
      dtwarn << "Skeleton [" << (*it)->getName()
             << "] is already in ConstraintSolver." << std::endl;
    }
  }

  if (numAddedSkeletons > 0)
    _init();
}

//==============================================================================
void ConstraintSolverTEST::removeSkeleton(Skeleton* _skeleton)
{
  assert(_skeleton != NULL
      && "Null pointer skeleton is now allowed to add to ConstraintSover.");

  if (_containSkeleton(_skeleton))
  {
    mSkeletons.erase(remove(mSkeletons.begin(), mSkeletons.end(), _skeleton),
                     mSkeletons.end());
    _init();
  }
  else
  {
    dtwarn << "Skeleton [" << _skeleton->getName()
           << "] is not in ConstraintSolver." << std::endl;
  }
}

//==============================================================================
void ConstraintSolverTEST::removeSkeletons(
    const std::vector<Skeleton*>& _skeletons)
{
  int numRemovedSkeletons = 0;

  for (std::vector<Skeleton*>::const_iterator it = _skeletons.begin();
       it != _skeletons.end(); ++it)
  {
    assert(*it != NULL
        && "Null pointer skeleton is now allowed to add to ConstraintSover.");

    if (_containSkeleton(*it))
    {
      mSkeletons.erase(remove(mSkeletons.begin(), mSkeletons.end(), *it),
                       mSkeletons.end());
      ++numRemovedSkeletons;
    }
    else
    {
      dtwarn << "Skeleton [" << (*it)->getName()
             << "] is not in ConstraintSolver." << std::endl;
    }
  }

  if (numRemovedSkeletons > 0)
    _init();
}

//==============================================================================
void ConstraintSolverTEST::removeAllSkeletons()
{
  std::cout << "ConstraintSolverTEST::removeAllSkeletons(): "
            << "Not implemented yet."
            << std::endl;
}

//==============================================================================
void ConstraintSolverTEST::addConstraint(ConstraintTEST* _constraint)
{
  std::cout << "ConstraintSolverTEST::addConstraint(): "
            << "Not implemented yet."
            << std::endl;
}

//==============================================================================
void ConstraintSolverTEST::addConstraints(const std::vector<ConstraintTEST*>& _constraints)
{
  std::cout << "ConstraintSolverTEST::addConstraints(): "
            << "Not implemented yet."
            << std::endl;
}

//==============================================================================
void ConstraintSolverTEST::removeConstraint(ConstraintTEST* _constraint)
{
  std::cout << "ConstraintSolverTEST::removeConstraint(): "
            << "Not implemented yet."
            << std::endl;
}

//==============================================================================
void ConstraintSolverTEST::removeConstraints(const std::vector<ConstraintTEST*>& _constraints)
{
  std::cout << "ConstraintSolverTEST::removeConstraints(): "
            << "Not implemented yet."
            << std::endl;
}

//==============================================================================
void ConstraintSolverTEST::removeAllConstraints()
{
  std::cout << "ConstraintSolverTEST::removeAllConstraints(): "
            << "Not implemented yet."
            << std::endl;
}

//==============================================================================
void ConstraintSolverTEST::setTimeStep(double _timeStep)
{
  assert(_timeStep > 0.0 && "Time step should be positive value.");
  mTimeStep = _timeStep;
}

//==============================================================================
double ConstraintSolverTEST::getTimeStep() const
{
  return mTimeStep;
}

//==============================================================================
void ConstraintSolverTEST::solve()
{
  _updateDynamicConstraints();
  _buildConstrainedGroups();
  _solveConstrainedGroups();
}

//==============================================================================
void ConstraintSolverTEST::_init()
{
  _bakeConstraints();

  //---------------------------- Static constraints ----------------------------
  mStaticConstraints.clear();

  // Closed loop constraints
  for (std::vector<ClosedLoopConstraintTEST*>::const_iterator it
       = mBakedClosedLoopConstraints.begin();
       it != mBakedClosedLoopConstraints.end(); ++it)
  {
    mStaticConstraints.push_back(*it);
  }

  //--------------------------- Dynamic constraints ----------------------------
  mDynamicConstraints.clear();

  int maxNumDynamicConstraints = 0;
  maxNumDynamicConstraints += mBakedContactConstraints.size();
  maxNumDynamicConstraints += mBakedJointLimitContraints.size();
  maxNumDynamicConstraints += mBakedJointConstraints.size();

  mDynamicConstraints.reserve(maxNumDynamicConstraints);

  //---------------------------- Communities -----------------------------------
  // TODO(JS): Create one community for test
  for (std::vector<ConstrainedGroup*>::iterator it = mConstrainedGroups.begin();
       it != mConstrainedGroups.end(); ++it)
  {
    delete *it;
  }
  mConstrainedGroups.clear();
  mConstrainedGroups.resize(1);
  mConstrainedGroups[0] = new ConstrainedGroup;
}

//==============================================================================
void ConstraintSolverTEST::_bakeConstraints()
{
  // Contact constraints
  __bakeContactConstraints();

  // Joint limit constraints
  __bakeJointLimitConstraints();

  // closed loop constraints
  __bakeClosedLoopConstraints();

  // Joint constraints
  __bakeJointConstraints();
}

//==============================================================================
void ConstraintSolverTEST::__bakeContactConstraints()
{
  std::cout << "ConstraintSolverTEST::__bakeContactConstraints(): "
            << "Not implemented yet."
            << std::endl;

  // Reset backed contact constraints
  for (std::vector<ContactConstraintTEST*>::iterator it
           = mBakedContactConstraints.begin();
       it != mBakedContactConstraints.end(); ++it)
  {
    delete *it;
  }
  mBakedContactConstraints.clear();

  // TODO():
  mCollisionDetector->removeAllSkeletons();
  for (std::vector<Skeleton*>::iterator it = mSkeletons.begin();
       it != mSkeletons.end(); ++it)
  {
    mCollisionDetector->addSkeleton(*it);
  }
}

//==============================================================================
void ConstraintSolverTEST::__bakeJointLimitConstraints()
{
  std::cout << "ConstraintSolverTEST::__bakeJointLimitConstraints(): "
            << "Not implemented yet."
            << std::endl;
}

//==============================================================================
void ConstraintSolverTEST::__bakeClosedLoopConstraints()
{
  std::cout << "ConstraintSolverTEST::__bakeClosedLoopConstraints(): "
            << "Not implemented yet."
            << std::endl;
}

//==============================================================================
void ConstraintSolverTEST::__bakeJointConstraints()
{
  std::cout << "ConstraintSolverTEST::__bakeJointConstraints(): "
            << "Not implemented yet."
            << std::endl;
}

//==============================================================================
bool ConstraintSolverTEST::_containSkeleton(const Skeleton* _skeleton) const
{
  assert(_skeleton != NULL && "Now allowed to insert null pointer skeleton.");

  for (std::vector<Skeleton*>::const_iterator it = mSkeletons.begin();
       it != mSkeletons.end(); ++it)
  {
    if ((*it) == _skeleton)
      return true;
  }

  return false;
}

//==============================================================================
bool ConstraintSolverTEST::_checkAndAddSkeleton(Skeleton* _skeleton)
{
  if (!_containSkeleton(_skeleton))
  {
    mSkeletons.push_back(_skeleton);
    return true;
  }
  else
  {
    dtwarn << "Skeleton [" << _skeleton->getName()
           << "] is already in ConstraintSolver." << std::endl;
    return false;
  }
}

//==============================================================================
bool ConstraintSolverTEST::_containConstraint(
    const ConstraintTEST* _constraint) const
{
  std::cout << "ConstraintSolverTEST::_containConstraint(): "
            << "Not implemented."
            << std::endl;

  return false;
}

//==============================================================================
bool ConstraintSolverTEST::_checkAndAddConstraint(ConstraintTEST* _constraint)
{
  std::cout << "ConstraintSolverTEST::_checkAndAddConstraint(): "
            << "Not implemented."
            << std::endl;

  if (!_containConstraint(_constraint))
  {
//    mConstraints.push_back(_constraint);
  }
  else
  {
//    dtwarn << "Constraint [" << _constraint->getName()
//           << "] is already in ConstraintSolver." << std::endl;
  }

  return false;
}

//==============================================================================
void ConstraintSolverTEST::_updateDynamicConstraints()
{
  // TODO(JS): Use warn starting
  mCollisionDetector->clearAllContacts();
  mCollisionDetector->detectCollision(true, true);

  mDynamicConstraints.clear();

  // Contact constraints
  for (std::vector<ContactConstraintTEST*>::const_iterator it
       = mBakedContactConstraints.begin();
       it != mBakedContactConstraints.end(); ++it)
  {
    delete *it;
  }
  mBakedContactConstraints.clear();
  for (int i = 0; i < mCollisionDetector->getNumContacts(); ++i)
  {
    const collision::Contact& ct = mCollisionDetector->getContact(i);
    ContactConstraintTEST* cc = new ContactConstraintTEST(ct);
    mBakedContactConstraints.push_back(cc);
  }
  for (std::vector<ContactConstraintTEST*>::const_iterator it
       = mBakedContactConstraints.begin();
       it != mBakedContactConstraints.end(); ++it)
  {
    if ((*it)->isActive())
      mDynamicConstraints.push_back(*it);
  }

  // Joint limit constraints
  for (std::vector<JointLimitConstraintTEST*>::const_iterator it
       = mBakedJointLimitContraints.begin();
       it != mBakedJointLimitContraints.end(); ++it)
  {
    if ((*it)->isActive())
      mDynamicConstraints.push_back(*it);
  }

  // Joint constraints
  for (std::vector<JointConstraintTEST*>::const_iterator it
       = mBakedJointConstraints.begin();
       it != mBakedJointConstraints.end(); ++it)
  {
    if ((*it)->isActive())
      mDynamicConstraints.push_back(*it);
  }
}

//==============================================================================
void ConstraintSolverTEST::_buildConstrainedGroups()
{
  std::cout << "ConstraintSolverTEST::_organizeCommunities(): Not implemented."
            << std::endl;

  // TODO(JS):
  mConstrainedGroups[0]->removeAllConstraints();

  //-------------- Add Constraints to constrained groups -----------------------
  // Static constraints
  for (std::vector<ConstraintTEST*>::iterator it = mStaticConstraints.begin();
       it != mStaticConstraints.end(); ++it)
  {
    // TODO(JS):
    mConstrainedGroups[0]->addConstraint(*it);
  }

  // Dynamics constraints
  for (std::vector<ConstraintTEST*>::iterator it = mDynamicConstraints.begin();
       it != mDynamicConstraints.end(); ++it)
  {
    // TODO(JS):
    mConstrainedGroups[0]->addConstraint(*it);
  }
}

//==============================================================================
void ConstraintSolverTEST::_solveConstrainedGroups()
{
  // TODO(JS): Parallel computing is possible here.
  for (std::vector<ConstrainedGroup*>::iterator it = mConstrainedGroups.begin();
      it != mConstrainedGroups.end(); ++it)
  {
    (*it)->solve();
  }
}

}  // namespace constraint
}  // namespace dart
