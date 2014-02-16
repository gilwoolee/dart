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

#ifndef DART_CONSTRAINT_CONTACTCONSTRAINT_H_TEST
#define DART_CONSTRAINT_CONTACTCONSTRAINT_H_TEST

#include "dart/constraint_test/Constraint.h"

#include "dart/math/MathTypes.h"
#include "dart/collision/CollisionDetector.h"

// TODO(JS): More meaningful numbers?
#define MIN_NUM_FRICTION_CONE_BASES 2
#define MAX_NUM_FRICTION_CONE_BASES 16

namespace dart {
namespace constraint {

//==============================================================================
/// \brief The ContactConstraintTEST class
class ContactConstraintTEST : public ConstraintTEST
{
public:
  //----------------------------------------------------------------------------
  /// \brief Default contructor
  ContactConstraintTEST(int _numBasisDirections = 4);

  /// \brief Constructor
  explicit ContactConstraintTEST(const collision::Contact& _contact);

  /// \brief Default destructor
  virtual ~ContactConstraintTEST();

  //--------------------------- Settings ---------------------------------------
  /// \brief
  void setFrictionalCoeff(double _frictionalCoeff);

  /// \brief
  double getFrictionalCoeff() const;

  /// \brief
  void setNumFrictionConeBases(int _numFrictionConeBases);

  /// \brief
  int getNumFrictionConeBases() const;

  //-------------------- Constraint virtual function ---------------------------
  // Documentaion inherited.
  virtual void update();

  // Documentaion inherited.
  virtual void aggreateLCPTerms(LCPTerms* _info, int _idx);

  //----------------------------- Solving --------------------------------------
  /// \brief
  bool isActive();

protected:
  /// \brief
  dynamics::Skeleton* mSkeleton1;

  /// \brief
  dynamics::Skeleton* mSkeleton2;

  /// \brief
  dynamics::BodyNode* mBodyNode1;

  /// \brief
  dynamics::BodyNode* mBodyNode2;

  /// \brief Number of friction cone bases
  int mNumFrictionConeBases;

  /// \brief Frictional coefficient
  double mFrictionalCoff;

  /// \brief
  std::vector<Eigen::Vector6d> mJacobians;

private:
  /// \brief Compute change in velocity due to _idx-th impulse.
  void _updateVelocityChange(int _idx);

};

} // namespace constraint
} // namespace dart

#endif  // DART_CONSTRAINT_IBCONTACTCONSTRAINT_H_TEST

