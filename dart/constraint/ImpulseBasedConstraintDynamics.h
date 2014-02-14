/*
 * Copyright (c) 2011-2014, Georgia Tech Research Corporation
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

#ifndef DART_CONSTRAINT_IMPULSEBASEDCONSTRAINTDYNAMICS_H_
#define DART_CONSTRAINT_IMPULSEBASEDCONSTRAINTDYNAMICS_H_

#include <vector>

#include <Eigen/Dense>

#include "dart/constraint/ImpulseBasedConstraint.h"
#include "dart/collision/CollisionDetector.h"
#include "dart/collision/fcl_mesh/FCLMeshCollisionDetector.h"

namespace dart {
namespace dynamics {
class BodyNode;
class Skeleton;
class BodyNode;
}  // namespace dynamics
}  // namespace dart

namespace dart {
namespace constraint {

class ImpulseBasedConstraintDynamics
{
public:
  /// \brief
  ImpulseBasedConstraintDynamics(
      const std::vector<dynamics::Skeleton*>& _skels,
      double _dt,
      double _mu     = 1.0,
      int    _d      = 4,
      bool   _useODE = true,
      collision::CollisionDetector* _collisionDetector
          = new collision::FCLMeshCollisionDetector());

  /// \brief
  virtual ~ImpulseBasedConstraintDynamics();

  /// \brief
  void computeConstraintForces();

  /// \brief
  void addConstraint(ImpulseBasedConstraint *_constr);

  /// \brief
  void deleteConstraint(ImpulseBasedConstraint *_constr);

  /// \brief
  void deleteConstraint();

  /// \brief
  void addSkeleton(dynamics::Skeleton* _skeleton);

  /// \brief
  void removeSkeleton(dynamics::Skeleton* _skeleton);

  /// \brief
  void setTimeStep(double _timeStep);

  /// \brief
  double getTimeStep() const;

  /// \brief
  void setCollisionDetector(collision::CollisionDetector* _collisionDetector);

  /// \brief
  Eigen::VectorXd getTotalConstraintForce(int _skelIndex) const;

  /// \brief
  Eigen::VectorXd getContactForce(int _skelIndex) const;

  /// \brief
  collision::CollisionDetector* getCollisionDetector() const;

  /// \brief
  int getNumContacts() const;

  /// \brief
  ImpulseBasedConstraint* getConstraint(int _index) const;

protected:
  /// \brief
  void initialize();

  /// \brief
  void computeConstraintWithoutContact();

  /// \brief
  virtual void fillMatrices();

  /// \brief
  virtual void fillMatricesODE();

  /// \brief
  bool solve();

  /// \brief
  virtual void applySolution();

  /// \brief
  virtual void applySolutionODE();

  /// \brief
  void updateMassMat();

  /// \brief
  void updateTauStar();

  /// \brief
  virtual void updateNBMatrices();

  /// \brief
  virtual void updateNBMatricesODE();

  /// \brief
  virtual Eigen::MatrixXd getJacobian(dynamics::BodyNode* node,
                                      const Eigen::Vector3d& p);
  /// \brief gets a matrix of tangent dirs.
  Eigen::MatrixXd getTangentBasisMatrix(const Eigen::Vector3d& p,
                                        const Eigen::Vector3d& n);
  /// \brief gets a matrix of tangent dirs.
  Eigen::MatrixXd getTangentBasisMatrixODE(const Eigen::Vector3d& p,
                                           const Eigen::Vector3d& n);

  /// \brief E matrix
  Eigen::MatrixXd getContactMatrix() const;

  /// \brief mu matrix
  Eigen::MatrixXd getMuMatrix() const;

  /// \brief
  void updateConstraintTerms();

  /// \brief
  int getTotalNumDofs() const;

  /// \brief
  std::vector<dynamics::Skeleton*> mSkeletons;

  /// \brief
  std::vector<int> mBodyIndexToSkelIndex;

  /// \brief
  std::vector<int> mIndices;

  /// \brief
  collision::CollisionDetector* mCollisionDetector;

  /// \brief timestep
  double mDt;

  /// \brief friction coeff.
  double mMu;

  /// \brief number of basis directions
  int mNumDir;

  //--------------- Cached (aggregated) mass/tau matrices ----------------------
  /// \brief
  Eigen::MatrixXd mMInv;

  /// \brief
  Eigen::VectorXd mTauStar;

  /// \brief
  Eigen::MatrixXd mN;

  /// \brief
  Eigen::MatrixXd mB;

  //--------------------- Matrices to pass to solver ---------------------------
  /// \brief
  Eigen::MatrixXd mA;

  /// \brief
  Eigen::VectorXd mQBar;

  /// \brief
  Eigen::VectorXd mX;



  /// \brief
  std::vector<Eigen::VectorXd> mContactForces;

  /// \brief solved constraint force in generalized coordinates;
  // mTotalConstrForces[i] is the constraint force for the ith skeleton
  std::vector<Eigen::VectorXd> mTotalConstrForces;

  /// \brief constraints
  std::vector<ImpulseBasedConstraint*> mConstraints;

  /// \brief
  int mTotalRows;

  /// \brief
  Eigen::MatrixXd mZ;  // N x N, symmetric (only lower triangle filled)

  /// \brief
  Eigen::VectorXd mTauHat;  // M x 1

  /// \brief
  Eigen::MatrixXd mGInv;  // M x M, symmetric (only lower triangle filled)

  /// \brief
  std::vector<Eigen::MatrixXd> mJMInv;  // M x N

  /// \brief
  std::vector<Eigen::MatrixXd> mJ;  // M x N

  /// \brief
  std::vector<Eigen::MatrixXd> mPreJ;  // M x N

  /// \brief
  Eigen::VectorXd mC;  // M * 1

  /// \brief
  Eigen::VectorXd mCDot;  // M * 1

  // if dof i hits upper limit, we store this information as
  // mLimitingDofIndex.push_back(i+1), if dof i hits lower limite,
  // mLimitingDofIndex.push_back(-(i+1));

  /// \brief
  std::vector<int> mLimitingDofIndex;

  /// \brief
  bool mUseODELCPSolver;

  /// \brief
  // TODO: this map needs to be rebuilt when the order of skeletons changes
  std::map<ImpulseBasedConstraint*, Eigen::Vector2i> mSkeletonIDMap;
};

}  // namespace constraint
}  // namespace dart

#endif  // DART_CONSTRAINT_IMPULSEBASEDCONSTRAINTDYNAMICS_H_
