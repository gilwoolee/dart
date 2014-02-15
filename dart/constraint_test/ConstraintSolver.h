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

#ifndef DART_CONSTRAINT_CONSTRAINTSOVER_H_TEST
#define DART_CONSTRAINT_CONSTRAINTSOVER_H_TEST

#include <vector>

#include <Eigen/Dense>

#include "dart/constraint_test/Constraint.h"
#include "dart/collision/CollisionDetector.h"

namespace dart {
namespace dynamics {
class Skeleton;
}
namespace constraint {
class CommunityTEST;
class ConstraintTEST;
class BallJointContraintTEST;
class ClosedLoopConstraintTEST;
class ContactConstraintTEST;
class JointLimitConstraintTEST;
class RevoluteJointContraintTEST;
class WeldJointContraintTEST;
class JointConstraintTEST;
}  // namespace constraint
namespace collision {
class FCLMeshCollisionDetector;
}  // namespace collision
}  // namespace dart

namespace dart {
namespace constraint {

class ConstraintSolverTEST
{
public:
  /// \brief
  ConstraintSolverTEST(
      const std::vector<dynamics::Skeleton*>& _skeletons,
      double _timeStep,
      double _frictionalCoefficient = 1.0,
      int    _numFrictionConeBasis  = 4,
      bool   _useODE                = true);

  /// \brief
  virtual ~ConstraintSolverTEST();

  //----------------------------- Setting --------------------------------------
  // It is allowed to call the following functions middle of simulation steps.

  /// \brief Add single skeleton.
  void addSkeleton(dynamics::Skeleton* _skeleton);

  /// \brief Add skeletons.
  void addSkeletons(const std::vector<dynamics::Skeleton*>& _skeletons);

  /// \brief Remove single skeleton.
  void removeSkeleton(dynamics::Skeleton* _skeleton);

  /// \brief Remove skeletons.
  void removeSkeletons(const std::vector<dynamics::Skeleton*>& _skeletons);

  /// \brief Remove all skeletons.
  void removeAllSkeletons();

  /// \brief Add single constraint.
  void addConstraint(ConstraintTEST* _constraint);

  /// \brief Add constraints.
  void addConstraints(const std::vector<ConstraintTEST*>& _constraints);

  /// \brief Remove single constraint.
  void removeConstraint(ConstraintTEST* _constraint);

  /// \brief Remove constraints.
  void removeConstraints(const std::vector<ConstraintTEST*>& _constraints);

  /// \brief Remove all constraints.
  void removeAllConstraints();

  /// \brief
  void setTimeStep(double _timeStep);

  /// \brief
  double getTimeStep() const;

  /// \brief
  void setFrictionalCoefficient(double _frictionalCoefficient);

  /// \brief
  double getFrictionalCoefficient() const;

  /// \brief
  void setNumFrictionConeBasis(int _numFrictionConeBasis);

  /// \brief
  int getNumFrictionConeBasis() const;

  //----------------------------- Solving --------------------------------------
  /// \brief
  virtual void solve();

protected:
  //----------------------------------------------------------------------------
  /// \brief
  std::vector<dynamics::Skeleton*> mSkeletons;

  /// \brief
  std::vector<ConstraintTEST*> mBakedConstraints;

  /// \brief
  std::vector<ContactConstraintTEST*> mBakedContactConstraints;

  /// \brief
  std::vector<JointLimitConstraintTEST*> mBakedJointLimitContraints;

  /// \brief
  std::vector<ClosedLoopConstraintTEST*> mBakedClosedLoopConstraints;

  /// \brief
  std::vector<JointConstraintTEST*> mBakedJointConstraints;

  //----------------------------------------------------------------------------
  /// \brief
  std::vector<ConstraintTEST*> mStaticConstraints;

  /// \brief
  std::vector<ConstraintTEST*> mDynamicConstraints;

  /// \brief List of communities
  std::vector<CommunityTEST*> mCommunities;

  /// \brief
//  std::vector<ClosedLoopContraint_TEST*> mBakedClosedLoopConstraints;

private:
  /// \brief
  void _init();

  /// \brief
  void _bakeConstraints();

  /// \brief
  bool _containSkeleton(const dynamics::Skeleton* _skeleton) const;

  /// \brief
  bool _checkAndAddSkeleton(dynamics::Skeleton* _skeleton);

  /// \brief
  bool _containConstraint(const ConstraintTEST* _constraint) const;

  /// \brief
  bool _checkAndAddConstraint(ConstraintTEST* _constraint);

  /// \brief
  void _updateDynamicConstraints();

  /// \brief
  void _organizeCommunities();

  /// \brief
  void _solveConstrains();

  /// \brief
  collision::CollisionDetector* mCollisionDetector;

  /// \brief
  double mTimeStep;

  /// \brief
  double mFrictionalCofficient;

  /// \brief
  int mNumFrictionConeBasis;

  /// \brief
  bool mUseODE;
};

}  // namespace constraint
}  // namespace dart

#endif  // DART_CONSTRAINT_CONSTRAINTSOVER_H_TEST
