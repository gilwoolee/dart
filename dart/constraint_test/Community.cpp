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

#include "dart/constraint_test/Community.h"

#include <iostream>
#include <vector>

#include "dart/lcpsolver/LCPSolver.h"
#include "dart/lcpsolver/Lemke.h"
#include "dart/lcpsolver/lcp.h"
#include "dart/constraint_test/Constraint.h"

namespace dart {
namespace constraint {

using namespace lcpsolver;

CommunityTEST::CommunityTEST()
{
}

CommunityTEST::~CommunityTEST()
{
}

void CommunityTEST::addConstraint(ConstraintTEST* _constraint)
{
  assert(_constraint != NULL && "Null constraint pointer is now allowed.");
  assert(_containConstraint(_constraint) == false
         && "Don't try to add same constraint multiple times into Community.");

  mConstraints.push_back(_constraint);
}

void CommunityTEST::removeConstraint(ConstraintTEST* _constraint)
{
  assert(_constraint != NULL && "Null constraint pointer is now allowed.");
  assert(_containConstraint(_constraint) == true
         && "Don't try to remove a constraint not contained in Community.");

  mConstraints.erase(
        remove(mConstraints.begin(), mConstraints.end(), _constraint),
        mConstraints.end());
}

void CommunityTEST::removeAllConstraints()
{
  std::cout << "CommunityTEST::removeAllContraints(): Not implemented."
            << std::endl;
}

bool CommunityTEST::solveConstraints()
{
  std::cout << "CommunityTEST::solveConstraints(): Not implemented."
            << std::endl;

  return _solveODE();
}

bool CommunityTEST::_containConstraint(ConstraintTEST* _constraint) const
{
  std::cout << "CommunityTEST::_containConstraint(): Not implemented."
            << std::endl;

  return false;
}

bool CommunityTEST::_checkAndAddConstraint(ConstraintTEST* _constraint)
{
  std::cout << "CommunityTEST::_checkAndAddConstraint(): Not implemented."
            << std::endl;

  return false;
}

bool CommunityTEST::_solveODE()
{
  // If there is no constraint, then just return true.
  if (mConstraints.size() == 0)
    return true;

  // Count total constraint dimensions and compute offset indices
  int totalDim = 0;
  int* offsetIndex = new int[totalDim];
  for (int i = 0; i < mConstraints.size(); ++i)
  {
    offsetIndex[i] = totalDim;
    totalDim += mConstraints[i]->getDimension();
  }

  // Build LCP terms by aggregating them from constraints
  LCPTerms lcp(totalDim);
  for (int i = 0; i < mConstraints.size(); ++i)
  {
    mConstraints[i]->update();
    mConstraints[i]->aggreateLCPTerms(&lcp, offsetIndex[i]);
  }

  // Solve LCP using ODE's Dantzig algorithm
  double* x = new double[totalDim];
  dSolveLCP(totalDim, lcp.A, x, lcp.b,
            lcp.w, 0, lcp.lb, lcp.ub, lcp.frictionIndex);

  // Apply constraint impulses
  for (std::vector<ConstraintTEST*>::const_iterator it = mConstraints.begin();
       it != mConstraints.end(); ++it)
  {
//    n += (*it)->applyConstraintImpulse();
  }

  // TODO(JS): Do we need to return boolean?
  return true;
}

bool CommunityTEST::_solveLemke()
{

}

bool CommunityTEST::_solvePGS()
{

}

}  // namespace constraint
}  // namespace dart
