/*
 * Copyright (c) 2013-2014, Georgia Tech Research Corporation
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

#include "dart/dynamics/SoftBodyNode.h"

#include <map>
#include <string>
#include <vector>

#include "dart/math/Helpers.h"
#include "dart/common/Console.h"
#include "dart/dynamics/Joint.h"
#include "dart/dynamics/Shape.h"
#include "dart/dynamics/Skeleton.h"
#include "dart/renderer/LoadOpengl.h"
#include "dart/renderer/RenderInterface.h"

#include "dart/dynamics/PointMass.h"
#include "dart/dynamics/SoftMeshShape.h"

using std::vector;
using std::string;
using std::map;

using Eigen::Isometry3d;
using Eigen::Vector3d;
using Eigen::Vector3i;

namespace dart {
namespace dynamics {

//==============================================================================
SoftBodyNode::SoftBodyNode(const std::string& _name)
  : BodyNode(_name),
    mKv(DART_DEFAULT_VERTEX_STIFFNESS),
    mKe(DART_DEFAULT_EDGE_STIFNESS),
    mDampCoeff(DART_DEFAULT_DAMPING_COEFF),
    mSoftVisualShape(NULL),
    mSoftCollShape(NULL)
{
}

//==============================================================================
SoftBodyNode::~SoftBodyNode()
{
  for (size_t i = 0; i < mPointMasses.size(); ++i)
    delete mPointMasses[i];
}

//==============================================================================
size_t SoftBodyNode::getNumPointMasses() const
{
  return mPointMasses.size();
}

//==============================================================================
PointMass* SoftBodyNode::getPointMass(size_t _idx) const
{
  assert(0 <= _idx && _idx < mPointMasses.size());
  return mPointMasses[_idx];
}

//==============================================================================
void SoftBodyNode::init(Skeleton* _skeleton)
{
  BodyNode::init(_skeleton);

  for (size_t i = 0; i < mPointMasses.size(); ++i)
    mPointMasses[i]->init();

//  //----------------------------------------------------------------------------
//  // Visualization shape
//  //----------------------------------------------------------------------------
//  assert(mSoftVisualShape == NULL);
//  mSoftVisualShape = new SoftMeshShape(this);
//  BodyNode::addVisualizationShape(mSoftVisualShape);

//  //----------------------------------------------------------------------------
//  // Collision shape
//  //----------------------------------------------------------------------------
//  assert(mSoftCollShape == NULL);
//  mSoftCollShape = new SoftMeshShape(this);
//  BodyNode::addCollisionShape(mSoftCollShape);
}

//==============================================================================
//void SoftBodyNode::aggregateGenCoords(std::vector<GenCoord*>* _genCoords)
//{
//  BodyNode::aggregateGenCoords(_genCoords);
//  aggregatePointMassGenCoords(_genCoords);
//}

//==============================================================================
//void SoftBodyNode::aggregatePointMassGenCoords(
//    std::vector<GenCoord*>* _genCoords)
//{
//  for (size_t i = 0; i < getNumPointMasses(); ++i)
//  {
//    PointMass* pointMass = getPointMass(i);
//    for (int j = 0; j < pointMass->getNumDofs(); ++j)
//    {
//      GenCoord* genCoord = pointMass->getGenCoord(j);
//      genCoord->setSkeletonIndex(_genCoords->size());
//      _genCoords->push_back(genCoord);
//    }
//  }
//}

//==============================================================================
double SoftBodyNode::getMass() const
{
  double totalMass = BodyNode::getMass();

  for (size_t i = 0; i < mPointMasses.size(); ++i)
    totalMass += mPointMasses.at(i)->getMass();

  return totalMass;
}

//==============================================================================
void SoftBodyNode::setVertexSpringStiffness(double _kv)
{
  assert(0.0 <= _kv);
  mKv = _kv;
}

//==============================================================================
double SoftBodyNode::getVertexSpringStiffness() const
{
  return mKv;
}

//==============================================================================
void SoftBodyNode::setEdgeSpringStiffness(double _ke)
{
  assert(0.0 <= _ke);
  mKe = _ke;
}

//==============================================================================
double SoftBodyNode::getEdgeSpringStiffness() const
{
  return mKe;
}

//==============================================================================
void SoftBodyNode::setDampingCoefficient(double _damp)
{
  assert(_damp >= 0.0);
  mDampCoeff = _damp;
}

//==============================================================================
double SoftBodyNode::getDampingCoefficient() const
{
  return mDampCoeff;
}

//==============================================================================
void SoftBodyNode::removeAllPointMasses()
{
  mPointMasses.clear();
}

//==============================================================================
void SoftBodyNode::addPointMass(PointMass* _pointMass)
{
  assert(_pointMass != NULL);
  mPointMasses.push_back(_pointMass);
}

//==============================================================================
void SoftBodyNode::connectPointMasses(size_t _idx1, size_t _idx2)
{
  assert(_idx1 != _idx2);
  assert(0 <= _idx1 && _idx1 < mPointMasses.size());
  assert(0 <= _idx2 && _idx2 < mPointMasses.size());
  mPointMasses[_idx1]->addConnectedPointMass(mPointMasses[_idx2]);
  mPointMasses[_idx2]->addConnectedPointMass(mPointMasses[_idx1]);
}

//==============================================================================
void SoftBodyNode::addFace(const Eigen::Vector3i& _face)
{
  assert(_face[0] != _face[1]);
  assert(_face[1] != _face[2]);
  assert(_face[2] != _face[0]);
  assert(0 <= _face[0] && _face[0] < math::castUIntToInt(mPointMasses.size()));
  assert(0 <= _face[1] && _face[1] < math::castUIntToInt(mPointMasses.size()));
  assert(0 <= _face[2] && _face[2] < math::castUIntToInt(mPointMasses.size()));
  mFaces.push_back(_face);
}

//==============================================================================
const Eigen::Vector3i& SoftBodyNode::getFace(size_t _idx) const
{
  assert(0 <= _idx && _idx < mFaces.size());
  return mFaces[_idx];
}

//==============================================================================
size_t SoftBodyNode::getNumFaces()
{
  return mFaces.size();
}

//==============================================================================
void SoftBodyNode::clearConstraintImpulse()
{
  BodyNode::clearConstraintImpulse();

  for (size_t i = 0; i < mPointMasses.size(); ++i)
    mPointMasses.at(i)->clearConstraintImpulse();
}

//==============================================================================
void SoftBodyNode::updateTransform()
{
  BodyNode::updateTransform();

  for (size_t i = 0; i < mPointMasses.size(); ++i)
    mPointMasses.at(i)->updateTransform();
}

//==============================================================================
void SoftBodyNode::updateVelocity()
{
  BodyNode::updateVelocity();

  for (size_t i = 0; i < mPointMasses.size(); ++i)
    mPointMasses.at(i)->updateVelocity();
}

//==============================================================================
void SoftBodyNode::updatePartialAcceleration()
{
  BodyNode::updatePartialAcceleration();

  for (size_t i = 0; i < mPointMasses.size(); ++i)
    mPointMasses.at(i)->updatePartialAcceleration();
}

//==============================================================================
void SoftBodyNode::updateAcceleration()
{
  BodyNode::updateAcceleration();

  for (size_t i = 0; i < mPointMasses.size(); ++i)
    mPointMasses.at(i)->updateAcceleration();
}

//==============================================================================
void SoftBodyNode::updateBodyWrench(const Eigen::Vector3d& _gravity,
                                   bool _withExternalForces)
{
  for (size_t i = 0; i < mPointMasses.size(); ++i)
    mPointMasses.at(i)->updateBodyForce(_gravity, _withExternalForces);

  // Gravity force
  if (mGravityMode == true)
    mFgravity.noalias() = mI * math::AdInvRLinear(mW, _gravity);
  else
    mFgravity.setZero();

  // Inertial force
  mF.noalias() = mI * mA;

  // External force
  if (_withExternalForces)
    mF -= mFext;

  // Verification
  assert(!math::isNan(mF));

  // Gravity force
  mF -= mFgravity;

  // Coriolis force
  mF -= math::dad(mV, mI * mV);

  //
  for (std::vector<BodyNode*>::iterator iChildBody = mChildBodyNodes.begin();
       iChildBody != mChildBodyNodes.end(); ++iChildBody)
  {
    Joint* childJoint = (*iChildBody)->getParentJoint();
    assert(childJoint != NULL);

    mF += math::dAdInvT(childJoint->getLocalTransform(),
                        (*iChildBody)->getBodyForce());
  }
//  for (size_t i = 0; i < mPointMasses.size(); i++)
//  {
//    mF.head<3>() += mPointMasses[i]->mX.cross(mPointMasses[i]->mF);
//    mF.tail<3>() += mPointMasses[i]->mF;
//  }

  // TODO(JS): mWrench and mF are duplicated. Remove one of them.
  mParentJoint->mWrench = mF;

  // Verification
  assert(!math::isNan(mF));
}

//==============================================================================
void SoftBodyNode::updateGeneralizedForce(bool _withDampingForces)
{
  for (size_t i = 0; i < mPointMasses.size(); ++i)
    mPointMasses.at(i)->updateGeneralizedForce(_withDampingForces);

  BodyNode::updateGeneralizedForce(_withDampingForces);
}

//==============================================================================
void SoftBodyNode::updateArtInertia(double _timeStep)
{
  for (size_t i = 0; i < mPointMasses.size(); ++i)
    mPointMasses.at(i)->updateArticulatedInertia(_timeStep);

  assert(mParentJoint != NULL);

  // Set spatial inertia to the articulated body inertia
  mArtInertia = mI;
  mArtInertiaImplicit = mI;

  // and add child articulated body inertia
  for (std::vector<BodyNode*>::const_iterator it = mChildBodyNodes.begin();
       it != mChildBodyNodes.end(); ++it)
  {
    (*it)->getParentJoint()->addChildArtInertiaTo(
          mArtInertia, (*it)->mArtInertia);
    (*it)->getParentJoint()->addChildArtInertiaImplicitTo(
          mArtInertiaImplicit, (*it)->mArtInertiaImplicit);
  }

  //
  for (size_t i = 0; i < mPointMasses.size(); i++)
  {
    _addPiToArtInertia(mPointMasses[i]->mX, mPointMasses[i]->mPi);
    _addPiToArtInertiaImplicit(mPointMasses[i]->mX, mPointMasses[i]->mImplicitPi);
  }

  // Verification
  assert(!math::isNan(mArtInertia));
  assert(!math::isNan(mArtInertiaImplicit));

  // Update parent joint's inverse of projected articulated body inertia
  mParentJoint->updateInvProjArtInertia(mArtInertia);
  mParentJoint->updateInvProjArtInertiaImplicit(mArtInertiaImplicit, _timeStep);

  // Verification
  assert(!math::isNan(mArtInertia));
  assert(!math::isNan(mArtInertiaImplicit));
}

//==============================================================================
void SoftBodyNode::updateBiasForce(const Eigen::Vector3d& _gravity,
                                   double _timeStep)
{
  for (size_t i = 0; i < mPointMasses.size(); ++i)
    mPointMasses.at(i)->updateBiasForce(_timeStep, _gravity);

  // Gravity force
  if (mGravityMode == true)
    mFgravity.noalias() = mI * math::AdInvRLinear(mW, _gravity);
  else
    mFgravity.setZero();

  // Set bias force
  mBiasForce = -math::dad(mV, mI * mV) - mFext - mFgravity;

  // Verifycation
  assert(!math::isNan(mBiasForce));

  // And add child bias force
  for (std::vector<BodyNode*>::const_iterator it = mChildBodyNodes.begin();
       it != mChildBodyNodes.end(); ++it)
  {
    (*it)->getParentJoint()->addChildBiasForceTo(mBiasForce,
                                                 (*it)->mArtInertiaImplicit,
                                                 (*it)->mBiasForce,
                                                 (*it)->mPartialAcceleration);
  }

  //
  for (size_t i = 0; i < mPointMasses.size(); i++)
  {
    mBiasForce.head<3>() += mPointMasses[i]->mX.cross(mPointMasses[i]->mBeta);
    mBiasForce.tail<3>() += mPointMasses[i]->mBeta;
  }

  // Verifycation
  assert(!math::isNan(mBiasForce));

  // Update parent joint's total force with implicit joint damping and spring
  // forces
  mParentJoint->updateTotalForce(
        mArtInertiaImplicit * mPartialAcceleration + mBiasForce, _timeStep);
}

//==============================================================================
void SoftBodyNode::updateJointAndBodyAcceleration()
{
  BodyNode::updateJointAndBodyAcceleration();

  for (size_t i = 0; i < mPointMasses.size(); ++i)
    mPointMasses.at(i)->updateJointAndBodyAcceleration();
}

//==============================================================================
void SoftBodyNode::updateTransmittedWrench()
{
  BodyNode::updateTransmittedWrench();

  for (size_t i = 0; i < mPointMasses.size(); ++i)
    mPointMasses.at(i)->updateTransmittedForce();
}

//==============================================================================
void SoftBodyNode::updateBiasImpulse()
{
  for (size_t i = 0; i < mPointMasses.size(); ++i)
    mPointMasses.at(i)->updateBiasImpulse();

  // Update impulsive bias force
  mBiasImpulse = -mConstraintImpulse;
//  assert(mImpFext == Eigen::Vector6d::Zero());

  // And add child bias impulse
  for (std::vector<BodyNode*>::const_iterator it = mChildBodyNodes.begin();
       it != mChildBodyNodes.end(); ++it)
  {
    (*it)->getParentJoint()->addChildBiasImpulseTo(mBiasImpulse,
                                                   (*it)->mArtInertia,
                                                   (*it)->mBiasImpulse);
  }

  // TODO(JS):
  for (size_t i = 0; i < mPointMasses.size(); i++)
  {
    mBiasImpulse.head<3>() += mPointMasses[i]->mX.cross(mPointMasses[i]->mImpBeta);
    mBiasImpulse.tail<3>() += mPointMasses[i]->mImpBeta;
  }

  // Verification
  assert(!math::isNan(mBiasImpulse));

  // Update parent joint's total force
  mParentJoint->updateTotalImpulse(mBiasImpulse);
}

//==============================================================================
void SoftBodyNode::updateJointVelocityChange()
{
  BodyNode::updateJointVelocityChange();

  for (size_t i = 0; i < mPointMasses.size(); ++i)
    mPointMasses.at(i)->updateJointVelocityChange();
}

//==============================================================================
//void SoftBodyNode::updateBodyVelocityChange()
//{
//  BodyNode::updateBodyVelocityChange();

//  for (size_t i = 0; i < mPointMasses.size(); ++i)
//    mPointMasses.at(i)->updateBodyVelocityChange();
//}

//==============================================================================
void SoftBodyNode::updateBodyImpForceFwdDyn()
{
  BodyNode::updateBodyImpForceFwdDyn();

  for (size_t i = 0; i < mPointMasses.size(); ++i)
    mPointMasses.at(i)->updateBodyImpForceFwdDyn();
}

//==============================================================================
void SoftBodyNode::updateConstrainedJointAndBodyAcceleration(double _timeStep)
{
  BodyNode::updateConstrainedJointAndBodyAcceleration(_timeStep);

  for (size_t i = 0; i < mPointMasses.size(); ++i)
    mPointMasses.at(i)->updateConstrainedJointAndBodyAcceleration(_timeStep);
}

//==============================================================================
void SoftBodyNode::updateConstrainedTransmittedForce(double _timeStep)
{
  BodyNode::updateConstrainedTransmittedForce(_timeStep);
}

//==============================================================================
void SoftBodyNode::updateMassMatrix()
{
  BodyNode::updateMassMatrix();

//  for (size_t i = 0; i < mPointMasses.size(); ++i)
//    mPointMasses.at(i)->updateMassMatrix();
}

//==============================================================================
void SoftBodyNode::aggregateMassMatrix(Eigen::MatrixXd* _MCol, int _col)
{
  BodyNode::aggregateMassMatrix(_MCol, _col);
//  //------------------------ PointMass Part ------------------------------------
//  for (size_t i = 0; i < mPointMasses.size(); ++i)
//    mPointMasses.at(i)->aggregateMassMatrix(_MCol, _col);

//  //----------------------- SoftBodyNode Part ----------------------------------
//  //
//  mM_F.noalias() = mI * mM_dV;

//  // Verification
//  assert(!math::isNan(mM_F));

//  //
//  for (std::vector<BodyNode*>::const_iterator it = mChildBodyNodes.begin();
//       it != mChildBodyNodes.end(); ++it)
//  {
//    mM_F += math::dAdInvT((*it)->getParentJoint()->getLocalTransform(),
//                          (*it)->mM_F);
//  }

//  //
//  for (std::vector<PointMass*>::iterator it = mPointMasses.begin();
//       it != mPointMasses.end(); ++it)
//  {
//    mM_F.head<3>() += (*it)->mX.cross((*it)->mM_F);
//    mM_F.tail<3>() += (*it)->mM_F;
//  }

//  // Verification
//  assert(!math::isNan(mM_F));

//  //
//  int dof = mParentJoint->getNumDofs();
//  if (dof > 0)
//  {
//    int iStart = mParentJoint->getIndexInSkeleton(0);
//    _MCol->block(iStart, _col, dof, 1).noalias()
//        = mParentJoint->getLocalJacobian().transpose() * mM_F;
//  }
}

//==============================================================================
void SoftBodyNode::aggregateAugMassMatrix(Eigen::MatrixXd* _MCol, int _col,
                                          double _timeStep)
{
  // TODO(JS): Need to be reimplemented

  //------------------------ PointMass Part ------------------------------------
  for (size_t i = 0; i < mPointMasses.size(); ++i)
    mPointMasses.at(i)->aggregateAugMassMatrix(_MCol, _col, _timeStep);

  //----------------------- SoftBodyNode Part ----------------------------------
  mM_F.noalias() = mI * mM_dV;
  assert(!math::isNan(mM_F));

  for (std::vector<BodyNode*>::const_iterator it = mChildBodyNodes.begin();
       it != mChildBodyNodes.end(); ++it)
  {
    mM_F += math::dAdInvT((*it)->getParentJoint()->getLocalTransform(),
                          (*it)->mM_F);
  }
  for (std::vector<PointMass*>::iterator it = mPointMasses.begin();
       it != mPointMasses.end(); ++it)
  {
    mM_F.head<3>() += (*it)->mX.cross((*it)->mM_F);
    mM_F.tail<3>() += (*it)->mM_F;
  }
  assert(!math::isNan(mM_F));

  size_t dof = mParentJoint->getNumDofs();
  if (dof > 0)
  {
    Eigen::MatrixXd K = Eigen::MatrixXd::Zero(dof, dof);
    Eigen::MatrixXd D = Eigen::MatrixXd::Zero(dof, dof);
    for (size_t i = 0; i < dof; ++i)
    {
      K(i, i) = mParentJoint->getSpringStiffness(i);
      D(i, i) = mParentJoint->getDampingCoefficient(i);
    }
    int iStart = mParentJoint->getIndexInSkeleton(0);

    // TODO(JS): Not recommended to use Joint::getAccelerations
    _MCol->block(iStart, _col, dof, 1).noalias()
        = mParentJoint->getLocalJacobian().transpose() * mM_F
          + D * (_timeStep * mParentJoint->getAccelerations())
          + K * (_timeStep * _timeStep * mParentJoint->getAccelerations());
  }
}

//==============================================================================
void SoftBodyNode::updateInvMassMatrix()
{
  //------------------------ PointMass Part ------------------------------------
  for (size_t i = 0; i < mPointMasses.size(); ++i)
    mPointMasses.at(i)->updateInvMassMatrix();

  //----------------------- SoftBodyNode Part ----------------------------------
  //
  mInvM_c.setZero();

  //
  for (std::vector<BodyNode*>::const_iterator it = mChildBodyNodes.begin();
       it != mChildBodyNodes.end(); ++it)
  {
    (*it)->getParentJoint()->addChildBiasForceForInvMassMatrix(
          mInvM_c, (*it)->mArtInertia, (*it)->mInvM_c);
  }

  //
  for (std::vector<PointMass*>::iterator it = mPointMasses.begin();
       it != mPointMasses.end(); ++it)
  {
    mInvM_c.head<3>() += (*it)->mX.cross((*it)->mBiasForceForInvMeta);
    mInvM_c.tail<3>() += (*it)->mBiasForceForInvMeta;
  }

  // Verification
  assert(!math::isNan(mInvM_c));

  // Update parent joint's total force for inverse mass matrix
  mParentJoint->updateTotalForceForInvMassMatrix(mInvM_c);
}

//==============================================================================
void SoftBodyNode::updateInvAugMassMatrix()
{
  BodyNode::updateInvAugMassMatrix();

//  //------------------------ PointMass Part ------------------------------------
////  for (size_t i = 0; i < mPointMasses.size(); ++i)
////    mPointMasses.at(i)->updateInvAugMassMatrix();

//  //----------------------- SoftBodyNode Part ----------------------------------
//  //
//  mInvM_c.setZero();

//  //
//  for (std::vector<BodyNode*>::const_iterator it = mChildBodyNodes.begin();
//       it != mChildBodyNodes.end(); ++it)
//  {
//    (*it)->getParentJoint()->addChildBiasForceForInvAugMassMatrix(
//          mInvM_c, (*it)->mArtInertiaImplicit, (*it)->mInvM_c);
//  }

//  //
////  for (std::vector<PointMass*>::iterator it = mPointMasses.begin();
////       it != mPointMasses.end(); ++it)
////  {
////    mInvM_c.head<3>() += (*it)->mX.cross((*it)->mBiasForceForInvMeta);
////    mInvM_c.tail<3>() += (*it)->mBiasForceForInvMeta;
////  }

//  // Verification
//  assert(!math::isNan(mInvM_c));

//  // Update parent joint's total force for inverse mass matrix
//  mParentJoint->updateTotalForceForInvMassMatrix(mInvM_c);
}

//==============================================================================
void SoftBodyNode::aggregateInvMassMatrix(Eigen::MatrixXd* _InvMCol, int _col)
{
  if (mParentBodyNode)
  {
    //
    mParentJoint->getInvMassMatrixSegment(
          *_InvMCol, _col, mArtInertia, mParentBodyNode->mInvM_U);

    //
    mInvM_U = math::AdInvT(mParentJoint->mT, mParentBodyNode->mInvM_U);
  }
  else
  {
    //
    mParentJoint->getInvMassMatrixSegment(
          *_InvMCol, _col, mArtInertia, Eigen::Vector6d::Zero());

    //
    mInvM_U.setZero();
  }

  //
  mParentJoint->addInvMassMatrixSegmentTo(mInvM_U);

  //
  for (size_t i = 0; i < mPointMasses.size(); ++i)
    mPointMasses.at(i)->aggregateInvMassMatrix(_InvMCol, _col);
}

//==============================================================================
void SoftBodyNode::aggregateInvAugMassMatrix(Eigen::MatrixXd* _InvMCol,
                                             int _col,
                                             double _timeStep)
{
  BodyNode::aggregateInvAugMassMatrix(_InvMCol, _col, _timeStep);

//  if (mParentBodyNode)
//  {
//    //
//    mParentJoint->getInvAugMassMatrixSegment(
//          *_InvMCol, _col, mArtInertiaImplicit, mParentBodyNode->mInvM_U);

//    //
//    mInvM_U = math::AdInvT(mParentJoint->mT, mParentBodyNode->mInvM_U);
//  }
//  else
//  {
//    //
//    mParentJoint->getInvAugMassMatrixSegment(
//          *_InvMCol, _col, mArtInertiaImplicit, Eigen::Vector6d::Zero());

//    //
//    mInvM_U.setZero();
//  }

//  //
//  mParentJoint->addInvMassMatrixSegmentTo(mInvM_U);

//  //
////  for (size_t i = 0; i < mPointMasses.size(); ++i)
////    mPointMasses.at(i)->aggregateInvAugMassMatrix(_InvMCol, _col, _timeStep);
}

//==============================================================================
void SoftBodyNode::aggregateCoriolisForceVector(Eigen::VectorXd* _C)
{
  BodyNode::aggregateCoriolisForceVector(_C);
}

//==============================================================================
void SoftBodyNode::aggregateGravityForceVector(Eigen::VectorXd* _g,
                                               const Eigen::Vector3d& _gravity)
{
  //------------------------ PointMass Part ------------------------------------
  for (size_t i = 0; i < mPointMasses.size(); ++i)
    mPointMasses.at(i)->aggregateGravityForceVector(_g, _gravity);

  //----------------------- SoftBodyNode Part ----------------------------------
  if (mGravityMode == true)
    mG_F = mI * math::AdInvRLinear(mW, _gravity);
  else
    mG_F.setZero();

  for (std::vector<BodyNode*>::const_iterator it = mChildBodyNodes.begin();
       it != mChildBodyNodes.end(); ++it)
  {
    mG_F += math::dAdInvT((*it)->mParentJoint->getLocalTransform(),
                          (*it)->mG_F);
  }

  for (std::vector<PointMass*>::iterator it = mPointMasses.begin();
       it != mPointMasses.end(); ++it)
  {
    mG_F.head<3>() += (*it)->mX.cross((*it)->mG_F);
    mG_F.tail<3>() += (*it)->mG_F;
  }

  int nGenCoords = mParentJoint->getNumDofs();
  if (nGenCoords > 0)
  {
    Eigen::VectorXd g = -(mParentJoint->getLocalJacobian().transpose() * mG_F);
    int iStart = mParentJoint->getIndexInSkeleton(0);
    _g->segment(iStart, nGenCoords) = g;
  }
}

//==============================================================================
void SoftBodyNode::updateCombinedVector()
{
  BodyNode::updateCombinedVector();

//  for (size_t i = 0; i < mPointMasses.size(); ++i)
//    mPointMasses.at(i)->updateCombinedVector();
}

//==============================================================================
void SoftBodyNode::aggregateCombinedVector(Eigen::VectorXd* _Cg,
                                           const Eigen::Vector3d& _gravity)
{
  BodyNode::aggregateCombinedVector(_Cg, _gravity);
//  //------------------------ PointMass Part ------------------------------------
//  for (size_t i = 0; i < mPointMasses.size(); ++i)
//    mPointMasses.at(i)->aggregateCombinedVector(_Cg, _gravity);

//  //----------------------- SoftBodyNode Part ----------------------------------
//  // H(i) = I(i) * W(i) -
//  //        dad{V}(I(i) * V(i)) + sum(k \in children) dAd_{T(i,j)^{-1}}(H(k))
//  if (mGravityMode == true)
//    mFgravity = mI * math::AdInvRLinear(mW, _gravity);
//  else
//    mFgravity.setZero();

//  mCg_F = mI * mCg_dV;
//  mCg_F -= mFgravity;
//  mCg_F -= math::dad(mV, mI * mV);

//  for (std::vector<BodyNode*>::iterator it = mChildBodyNodes.begin();
//       it != mChildBodyNodes.end(); ++it)
//  {
//    mCg_F += math::dAdInvT((*it)->getParentJoint()->getLocalTransform(),
//                           (*it)->mCg_F);
//  }

//  for (std::vector<PointMass*>::iterator it = mPointMasses.begin();
//       it != mPointMasses.end(); ++it)
//  {
//    mCg_F.head<3>() += (*it)->mX.cross((*it)->mCg_F);
//    mCg_F.tail<3>() += (*it)->mCg_F;
//  }

//  int nGenCoords = mParentJoint->getNumDofs();
//  if (nGenCoords > 0)
//  {
//    Eigen::VectorXd Cg = mParentJoint->getLocalJacobian().transpose() * mCg_F;
//    int iStart = mParentJoint->getIndexInSkeleton(0);
//    _Cg->segment(iStart, nGenCoords) = Cg;
//  }
}

//==============================================================================
void SoftBodyNode::aggregateExternalForces(Eigen::VectorXd* _Fext)
{
  //------------------------ PointMass Part ------------------------------------
  for (size_t i = 0; i < mPointMasses.size(); ++i)
    mPointMasses.at(i)->aggregateExternalForces(_Fext);

  //----------------------- SoftBodyNode Part ----------------------------------
  mFext_F = mFext;

  for (std::vector<BodyNode*>::const_iterator it = mChildBodyNodes.begin();
       it != mChildBodyNodes.end(); ++it)
  {
    mFext_F += math::dAdInvT((*it)->mParentJoint->getLocalTransform(),
                             (*it)->mFext_F);
  }

  for (std::vector<PointMass*>::iterator it = mPointMasses.begin();
       it != mPointMasses.end(); ++it)
  {
    mFext_F.head<3>() += (*it)->mX.cross((*it)->mFext);
    mFext_F.tail<3>() += (*it)->mFext;
  }

  int nGenCoords = mParentJoint->getNumDofs();
  if (nGenCoords > 0)
  {
    Eigen::VectorXd Fext
        = mParentJoint->getLocalJacobian().transpose() * mFext_F;
    int iStart = mParentJoint->getIndexInSkeleton(0);
    _Fext->segment(iStart, nGenCoords) = Fext;
  }
}

//==============================================================================
void SoftBodyNode::clearExternalForces()
{
  BodyNode::clearExternalForces();

  for (size_t i = 0; i < mPointMasses.size(); ++i)
    mPointMasses.at(i)->clearExtForce();
}

//==============================================================================
void SoftBodyNode::draw(renderer::RenderInterface* _ri,
                        const Eigen::Vector4d& _color,
                        bool _useDefaultColor,
                        int _depth) const
{
  if (_ri == NULL)
    return;

  _ri->pushMatrix();

  // render the self geometry
  mParentJoint->applyGLTransform(_ri);

  _ri->pushName((unsigned)mID);
  // rigid body
  for (size_t i = 0; i < mVizShapes.size(); i++)
  {
    _ri->pushMatrix();
    mVizShapes[i]->draw(_ri, _color, _useDefaultColor);
    _ri->popMatrix();
  }

  // vertex
//  if (_showPointMasses)
//  {
//    for (size_t i = 0; i < mPointMasses.size(); ++i)
//    {
//      _ri->pushMatrix();
//      mPointMasses[i]->draw(_ri, _color, _useDefaultColor);
//      _ri->popMatrix();
//    }
//  }

  // edges (mesh)
//  Eigen::Vector4d fleshColor = _color;
//  fleshColor[3] = 0.5;
//  _ri->setPenColor(fleshColor);
//  if (_showMeshs)
  {
    Eigen::Vector3d pos;
    Eigen::Vector3d pos_normalized;
    for (size_t i = 0; i < mFaces.size(); ++i)
    {
      glEnable(GL_AUTO_NORMAL);
      glBegin(GL_TRIANGLES);

      pos = mPointMasses[mFaces[i](0)]->mX;
      pos_normalized = pos.normalized();
      glNormal3f(pos_normalized(0), pos_normalized(1), pos_normalized(2));
      glVertex3f(pos(0), pos(1), pos(2));
      pos = mPointMasses[mFaces[i](1)]->mX;
      pos_normalized = pos.normalized();
      glNormal3f(pos_normalized(0), pos_normalized(1), pos_normalized(2));
      glVertex3f(pos(0), pos(1), pos(2));
      pos = mPointMasses[mFaces[i](2)]->mX;
      pos_normalized = pos.normalized();
      glNormal3f(pos_normalized(0), pos_normalized(1), pos_normalized(2));
      glVertex3f(pos(0), pos(1), pos(2));
      glEnd();
    }
  }

  _ri->popName();

  // render the subtree
  for (unsigned int i = 0; i < mChildBodyNodes.size(); i++)
  {
    getChildBodyNode(i)->draw(_ri, _color, _useDefaultColor);
  }

  _ri->popMatrix();
}

//==============================================================================
void SoftBodyNode::_addPiToArtInertia(const Eigen::Vector3d& _p, double _Pi)
{
  Eigen::Matrix3d tmp = math::makeSkewSymmetric(_p);

  mArtInertia.topLeftCorner<3, 3>()    -= _Pi * tmp * tmp;
  mArtInertia.topRightCorner<3, 3>()   += _Pi * tmp;
  mArtInertia.bottomLeftCorner<3, 3>() -= _Pi * tmp;

  mArtInertia(3, 3) += _Pi;
  mArtInertia(4, 4) += _Pi;
  mArtInertia(5, 5) += _Pi;
}

//==============================================================================
void SoftBodyNode::_addPiToArtInertiaImplicit(const Eigen::Vector3d& _p,
                                              double _ImplicitPi)
{
  Eigen::Matrix3d tmp = math::makeSkewSymmetric(_p);

  mArtInertiaImplicit.topLeftCorner<3, 3>()    -= _ImplicitPi * tmp * tmp;
  mArtInertiaImplicit.topRightCorner<3, 3>()   += _ImplicitPi * tmp;
  mArtInertiaImplicit.bottomLeftCorner<3, 3>() -= _ImplicitPi * tmp;

  mArtInertiaImplicit(3, 3) += _ImplicitPi;
  mArtInertiaImplicit(4, 4) += _ImplicitPi;
  mArtInertiaImplicit(5, 5) += _ImplicitPi;
}

//==============================================================================
void SoftBodyNode::updateInertiaWithPointMass()
{
  // TODO(JS): Not implemented

  mI2 = mI;

  for (size_t i = 0; i < mPointMasses.size(); ++i)
  {

  }
}

//==============================================================================
void SoftBodyNodeHelper::setBox(SoftBodyNode*            _softBodyNode,
                                const Eigen::Vector3d&   _size,
                                const Eigen::Isometry3d& _localTransfom,
                                double                   _totalMass,
                                double                   _vertexStiffness,
                                double                   _edgeStiffness,
                                double                   _dampingCoeff)
{
  assert(_softBodyNode != NULL);

  //----------------------------------------------------------------------------
  // Misc
  //----------------------------------------------------------------------------
  _softBodyNode->setVertexSpringStiffness(_vertexStiffness);
  _softBodyNode->setEdgeSpringStiffness(_edgeStiffness);
  _softBodyNode->setDampingCoefficient(_dampingCoeff);

  //----------------------------------------------------------------------------
  // Point masses
  //----------------------------------------------------------------------------
  // Number of point masses
  size_t nPointMasses = 8;

  // Mass per vertices
  double mass = _totalMass / nPointMasses;

  // Resting positions for each point mass
  std::vector<Eigen::Vector3d> restingPos(nPointMasses,
                                          Eigen::Vector3d::Zero());
  restingPos[0] = _size.cwiseProduct(Eigen::Vector3d(-1.0, -1.0, -1.0)) * 0.5;
  restingPos[1] = _size.cwiseProduct(Eigen::Vector3d(+1.0, -1.0, -1.0)) * 0.5;
  restingPos[2] = _size.cwiseProduct(Eigen::Vector3d(-1.0, +1.0, -1.0)) * 0.5;
  restingPos[3] = _size.cwiseProduct(Eigen::Vector3d(+1.0, +1.0, -1.0)) * 0.5;
  restingPos[4] = _size.cwiseProduct(Eigen::Vector3d(-1.0, -1.0, +1.0)) * 0.5;
  restingPos[5] = _size.cwiseProduct(Eigen::Vector3d(+1.0, -1.0, +1.0)) * 0.5;
  restingPos[6] = _size.cwiseProduct(Eigen::Vector3d(-1.0, +1.0, +1.0)) * 0.5;
  restingPos[7] = _size.cwiseProduct(Eigen::Vector3d(+1.0, +1.0, +1.0)) * 0.5;

  // Point masses
  dynamics::PointMass* newPointMass = NULL;
  for (size_t i = 0; i < nPointMasses; ++i)
  {
    newPointMass = new PointMass(_softBodyNode);
    newPointMass->setRestingPosition(_localTransfom * restingPos[i]);
    newPointMass->setMass(mass);
    _softBodyNode->addPointMass(newPointMass);
  }

  //----------------------------------------------------------------------------
  // Edges
  //----------------------------------------------------------------------------
  // -- Bottoms
  _softBodyNode->connectPointMasses(0, 1);
  _softBodyNode->connectPointMasses(1, 3);
  _softBodyNode->connectPointMasses(3, 2);
  _softBodyNode->connectPointMasses(2, 0);

  // -- Tops
  _softBodyNode->connectPointMasses(4, 5);
  _softBodyNode->connectPointMasses(5, 7);
  _softBodyNode->connectPointMasses(7, 6);
  _softBodyNode->connectPointMasses(6, 4);

  // -- Sides
  _softBodyNode->connectPointMasses(0, 4);
  _softBodyNode->connectPointMasses(1, 5);
  _softBodyNode->connectPointMasses(2, 6);
  _softBodyNode->connectPointMasses(3, 7);

  //----------------------------------------------------------------------------
  // Faces
  //----------------------------------------------------------------------------
  // -- +Z
  _softBodyNode->addFace(Eigen::Vector3i(1, 0, 2));  // 0
  _softBodyNode->addFace(Eigen::Vector3i(1, 2, 3));  // 1

  // -- -Z
  _softBodyNode->addFace(Eigen::Vector3i(5, 6, 4));  // 2
  _softBodyNode->addFace(Eigen::Vector3i(5, 7, 6));  // 3

  // -- -Y
  _softBodyNode->addFace(Eigen::Vector3i(0, 5, 4));  // 4
  _softBodyNode->addFace(Eigen::Vector3i(0, 1, 5));  // 5

  // -- +Y
  _softBodyNode->addFace(Eigen::Vector3i(1, 3, 7));  // 6
  _softBodyNode->addFace(Eigen::Vector3i(1, 7, 5));  // 7

  // -- -X
  _softBodyNode->addFace(Eigen::Vector3i(3, 2, 6));  // 8
  _softBodyNode->addFace(Eigen::Vector3i(3, 6, 7));  // 9

  // -- +X
  _softBodyNode->addFace(Eigen::Vector3i(2, 0, 4));  // 10
  _softBodyNode->addFace(Eigen::Vector3i(2, 4, 6));  // 11
}

//==============================================================================
void SoftBodyNodeHelper::setBox(SoftBodyNode*     _softBodyNode,
                                const Vector3d&   _size,
                                const Isometry3d& _localTransfom,
                                const Vector3i&   _frags,
                                double            _totalMass,
                                double            _vertexStiffness,
                                double            _edgeStiffness,
                                double            _dampingCoeff)
{
  // Check validity of the parameters
  assert(_softBodyNode != NULL);
  assert(_size[0] > 0.0);
  assert(_size[1] > 0.0);
  assert(_size[2] > 0.0);
  assert(_frags[0] >= 2);
  assert(_frags[1] >= 2);
  assert(_frags[2] >= 2);
  assert(_totalMass > 0.0);
  assert(_vertexStiffness >= 0.0);
  assert(_edgeStiffness >= 0.0);
  assert(_dampingCoeff >= 0.0);

  size_t id = 0;

  // Map between point mass and its id
  map<PointMass*, size_t> pmMap;

  // Half size and points at the corners
  const Vector3d halfSize = 0.5 * _size;
  const Vector3d x0y0z0 = halfSize.cwiseProduct(Vector3d(-1.0, -1.0, -1.0));
  const Vector3d x1y0z0 = halfSize.cwiseProduct(Vector3d(+1.0, -1.0, -1.0));
  const Vector3d x1y1z0 = halfSize.cwiseProduct(Vector3d(+1.0, +1.0, -1.0));
  const Vector3d x0y1z0 = halfSize.cwiseProduct(Vector3d(-1.0, +1.0, -1.0));
  const Vector3d x0y0z1 = halfSize.cwiseProduct(Vector3d(-1.0, -1.0, +1.0));
  const Vector3d x1y0z1 = halfSize.cwiseProduct(Vector3d(+1.0, -1.0, +1.0));
  const Vector3d x1y1z1 = halfSize.cwiseProduct(Vector3d(+1.0, +1.0, +1.0));
  const Vector3d x0y1z1 = halfSize.cwiseProduct(Vector3d(-1.0, +1.0, +1.0));

  //----------------------------------------------------------------------------
  // Set soft body properties
  //----------------------------------------------------------------------------
  _softBodyNode->setVertexSpringStiffness(_vertexStiffness);
  _softBodyNode->setEdgeSpringStiffness(_edgeStiffness);
  _softBodyNode->setDampingCoefficient(_dampingCoeff);

  //----------------------------------------------------------------------------
  // Point masses
  //----------------------------------------------------------------------------
  // Number of point masses
  const size_t nCorners = 8;
  const size_t nVerticesAtEdgeX = _frags[0] - 2;
  const size_t nVerticesAtEdgeY = _frags[1] - 2;
  const size_t nVerticesAtEdgeZ = _frags[2] - 2;
  const size_t nVerticesAtSideX = nVerticesAtEdgeY*nVerticesAtEdgeZ;
  const size_t nVerticesAtSideY = nVerticesAtEdgeZ*nVerticesAtEdgeX;
  const size_t nVerticesAtSideZ = nVerticesAtEdgeX*nVerticesAtEdgeY;
  const size_t nVertices
      = nCorners
        + 4*(nVerticesAtEdgeX + nVerticesAtEdgeY + nVerticesAtEdgeZ)
        + 2*(nVerticesAtSideX + nVerticesAtSideY + nVerticesAtSideZ);

  // Mass per vertices
  const double mass = _totalMass / nVertices;

  // Lengths of edgesegments
  const Vector3d segLength(_size[0]/(_frags[0] - 1),
                           _size[1]/(_frags[1] - 1),
                           _size[2]/(_frags[2] - 1));

  // Point masses at the corners of the box
  vector<PointMass*> pmCorners(nCorners);

  // Point masses at the edges of the box
  vector<vector<PointMass*> > pmEdgeX(4, vector<PointMass*>(nVerticesAtEdgeX));
  vector<vector<PointMass*> > pmEdgeY(4, vector<PointMass*>(nVerticesAtEdgeY));
  vector<vector<PointMass*> > pmEdgeZ(4, vector<PointMass*>(nVerticesAtEdgeZ));

  // Point masses at the sides(faces) of the box
  vector<vector<PointMass*> > pmSideXNeg(nVerticesAtEdgeY,
                                         vector<PointMass*>(nVerticesAtEdgeZ));
  vector<vector<PointMass*> > pmSideXPos(nVerticesAtEdgeY,
                                         vector<PointMass*>(nVerticesAtEdgeZ));

  vector<vector<PointMass*> > pmSideYNeg(nVerticesAtEdgeZ,
                                         vector<PointMass*>(nVerticesAtEdgeX));
  vector<vector<PointMass*> > pmSideYPos(nVerticesAtEdgeZ,
                                         vector<PointMass*>(nVerticesAtEdgeX));

  vector<vector<PointMass*> > pmSideZNeg(nVerticesAtEdgeX,
                                         vector<PointMass*>(nVerticesAtEdgeY));
  vector<vector<PointMass*> > pmSideZPos(nVerticesAtEdgeX,
                                         vector<PointMass*>(nVerticesAtEdgeY));

  vector<Vector3d> beginPts;
  Vector3d restPos;

  // Corners
  beginPts.resize(nCorners);
  beginPts[0] = x0y0z0;
  beginPts[1] = x1y0z0;
  beginPts[2] = x1y1z0;
  beginPts[3] = x0y1z0;
  beginPts[4] = x0y0z1;
  beginPts[5] = x1y0z1;
  beginPts[6] = x1y1z1;
  beginPts[7] = x0y1z1;

  for (size_t i = 0; i < nCorners; ++i)
  {
    pmCorners[i] = new PointMass(_softBodyNode);
    pmCorners[i]->setRestingPosition(_localTransfom * beginPts[i]);
    pmCorners[i]->setMass(mass);
    _softBodyNode->addPointMass(pmCorners[i]);
    pmMap[pmCorners[i]] = id++;
  }

  // Edges (along X-axis)
  beginPts.resize(4);
  beginPts[0] = x0y0z0;
  beginPts[1] = x0y1z0;
  beginPts[2] = x0y1z1;
  beginPts[3] = x0y0z1;

  for (size_t i = 0; i < 4; ++i)
  {
    restPos = beginPts[i];

    for (size_t j = 0; j < nVerticesAtEdgeX; ++j)
    {
      restPos[0] += segLength[0];

      pmEdgeX[i][j] = new PointMass(_softBodyNode);
      pmEdgeX[i][j]->setRestingPosition(_localTransfom * restPos);
      pmEdgeX[i][j]->setMass(mass);

      _softBodyNode->addPointMass(pmEdgeX[i][j]);

      pmMap[pmEdgeX[i][j]] = id++;
    }
  }

  // Edges (along Y-axis)
  beginPts[0] = x0y0z0;
  beginPts[1] = x0y0z1;
  beginPts[2] = x1y0z1;
  beginPts[3] = x1y0z0;

  for (size_t i = 0; i < 4; ++i)
  {
    restPos = beginPts[i];

    for (size_t j = 0; j < nVerticesAtEdgeY; ++j)
    {
      restPos[1] += segLength[1];

      pmEdgeY[i][j] = new PointMass(_softBodyNode);
      pmEdgeY[i][j]->setRestingPosition(_localTransfom * restPos);
      pmEdgeY[i][j]->setMass(mass);

      _softBodyNode->addPointMass(pmEdgeY[i][j]);

      pmMap[pmEdgeY[i][j]] = id++;
    }
  }

  // Edges (along Z-axis)
  beginPts[0] = x0y0z0;
  beginPts[1] = x1y0z0;
  beginPts[2] = x1y1z0;
  beginPts[3] = x0y1z0;

  for (size_t i = 0; i < 4; ++i)
  {
    restPos = beginPts[i];

    for (size_t j = 0; j < nVerticesAtEdgeZ; ++j)
    {
      restPos[2] += segLength[2];

      pmEdgeZ[i][j] = new PointMass(_softBodyNode);
      pmEdgeZ[i][j]->setRestingPosition(_localTransfom * restPos);
      pmEdgeZ[i][j]->setMass(mass);

      _softBodyNode->addPointMass(pmEdgeZ[i][j]);

      pmMap[pmEdgeZ[i][j]] = id++;
    }
  }

  // Negative X side
  restPos = x0y0z0;

  for (size_t i = 0; i < nVerticesAtEdgeY; ++i)
  {
    restPos[2] = x0y0z0[2];
    restPos[1] += segLength[1];

    for (size_t j = 0; j < nVerticesAtEdgeZ; ++j)
    {
      restPos[2] += segLength[2];

      pmSideXNeg[i][j] = new PointMass(_softBodyNode);
      pmSideXNeg[i][j]->setRestingPosition(_localTransfom * restPos);
      pmSideXNeg[i][j]->setMass(mass);

      _softBodyNode->addPointMass(pmSideXNeg[i][j]);

      pmMap[pmSideXNeg[i][j]] = id++;
    }
  }

  // Positive X side
  restPos = x1y0z0;

  for (size_t i = 0; i < nVerticesAtEdgeY; ++i)
  {
    restPos[2] = x1y0z0[2];
    restPos[1] += segLength[1];

    for (size_t j = 0; j < nVerticesAtEdgeZ; ++j)
    {
      restPos[2] += segLength[2];

      pmSideXPos[i][j] = new PointMass(_softBodyNode);
      pmSideXPos[i][j]->setRestingPosition(_localTransfom * restPos);
      pmSideXPos[i][j]->setMass(mass);

      _softBodyNode->addPointMass(pmSideXPos[i][j]);

      pmMap[pmSideXPos[i][j]] = id++;
    }
  }

  // Negative Y side
  restPos = x0y0z0;

  for (size_t i = 0; i < nVerticesAtEdgeZ; ++i)
  {
    restPos[0] = x0y0z0[0];
    restPos[2] += segLength[2];

    for (size_t j = 0; j < nVerticesAtEdgeX; ++j)
    {
      restPos[0] += segLength[0];

      pmSideYNeg[i][j] = new PointMass(_softBodyNode);
      pmSideYNeg[i][j]->setRestingPosition(_localTransfom * restPos);
      pmSideYNeg[i][j]->setMass(mass);

      _softBodyNode->addPointMass(pmSideYNeg[i][j]);

      pmMap[pmSideYNeg[i][j]] = id++;
    }
  }

  // Positive Y side
  restPos = x0y1z0;

  for (size_t i = 0; i < nVerticesAtEdgeZ; ++i)
  {
    restPos[0] = x0y1z0[0];
    restPos[2] += segLength[2];

    for (size_t j = 0; j < nVerticesAtEdgeX; ++j)
    {
      restPos[0] += segLength[0];

      pmSideYPos[i][j] = new PointMass(_softBodyNode);
      pmSideYPos[i][j]->setRestingPosition(_localTransfom * restPos);
      pmSideYPos[i][j]->setMass(mass);

      _softBodyNode->addPointMass(pmSideYPos[i][j]);

      pmMap[pmSideYPos[i][j]] = id++;
    }
  }

  // Negative Z side
  restPos = x0y0z0;

  for (size_t i = 0; i < nVerticesAtEdgeX; ++i)
  {
    restPos[1] = x0y0z0[1];
    restPos[0] += segLength[0];

    for (size_t j = 0; j < nVerticesAtEdgeY; ++j)
    {
      restPos[1] += segLength[1];

      pmSideZNeg[i][j] = new PointMass(_softBodyNode);
      pmSideZNeg[i][j]->setRestingPosition(_localTransfom * restPos);
      pmSideZNeg[i][j]->setMass(mass);

      _softBodyNode->addPointMass(pmSideZNeg[i][j]);

      pmMap[pmSideZNeg[i][j]] = id++;
    }
  }

  // Positive Z side
  restPos = x0y0z1;

  for (size_t i = 0; i < nVerticesAtEdgeX; ++i)
  {
    restPos[1] = x0y0z1[1];
    restPos[0] += segLength[0];

    for (size_t j = 0; j < nVerticesAtEdgeY; ++j)
    {
      restPos[1] += segLength[1];

      pmSideZPos[i][j] = new PointMass(_softBodyNode);
      pmSideZPos[i][j]->setRestingPosition(_localTransfom * restPos);
      pmSideZPos[i][j]->setMass(mass);

      _softBodyNode->addPointMass(pmSideZPos[i][j]);

      pmMap[pmSideZPos[i][j]] = id++;
    }
  }

  //----------------------------------------------------------------------------
  // Faces
  //----------------------------------------------------------------------------
  const size_t nFacesX = 2*(_frags[1] - 1)*(_frags[2] - 1);
  const size_t nFacesY = 2*(_frags[2] - 1)*(_frags[0] - 1);
  const size_t nFacesZ = 2*(_frags[0] - 1)*(_frags[1] - 1);
  const size_t nFaces  = 2*nFacesX + 2*nFacesY + 2*nFacesZ;

  vector<Vector3i> facesXPos(nFacesX, Vector3i::Zero());
  vector<Vector3i> facesXNeg(nFacesX, Vector3i::Zero());
  vector<Vector3i> facesYPos(nFacesY, Vector3i::Zero());
  vector<Vector3i> facesYNeg(nFacesY, Vector3i::Zero());
  vector<Vector3i> facesZPos(nFacesZ, Vector3i::Zero());
  vector<Vector3i> facesZNeg(nFacesZ, Vector3i::Zero());

  Vector3i face;

  // +X side
  for (size_t i = 0; i < (size_t)(_frags[1] - 1); ++i)
  {
    for (size_t j = 0; j < (size_t)(_frags[2] - 1); ++j)
    {
//      face[0] = pmMap[facesXNeg[i + 0][j + 0]];
//      face[1] = pmMap[facesXNeg[i + 0][j + 1]];
//      face[2] = pmMap[facesXNeg[i + 1][j + 0]];
      _softBodyNode->addFace(face);

//      face[0] = pmMap[facesXNeg[i + 1][j + 1]];
//      face[1] = pmMap[facesXNeg[i + 1][j + 0]];
//      face[2] = pmMap[facesXNeg[i + 0][j + 1]];
      _softBodyNode->addFace(face);
    }
  }

  vector<Vector3i> faces(nFaces, Vector3i::Zero());

  int fIdx = 0;

  // Corners[0] faces
  faces[fIdx][0] = pmMap[pmCorners[0]];
  faces[fIdx][1] = pmMap[pmEdgeZ[0][0]];
  faces[fIdx][2] = pmMap[pmEdgeY[0][0]];
  fIdx++;

  faces[fIdx][0] = pmMap[pmSideXNeg[0][0]];
  faces[fIdx][1] = pmMap[pmEdgeY[0][0]];
  faces[fIdx][2] = pmMap[pmEdgeZ[0][0]];
  fIdx++;

  faces[fIdx][0] = pmMap[pmCorners[0]];
  faces[fIdx][1] = pmMap[pmEdgeX[0][0]];
  faces[fIdx][2] = pmMap[pmEdgeZ[0][0]];
  fIdx++;

  faces[fIdx][0] = pmMap[pmSideYNeg[0][0]];
  faces[fIdx][1] = pmMap[pmEdgeZ[0][0]];
  faces[fIdx][2] = pmMap[pmEdgeX[0][0]];
  fIdx++;

  faces[fIdx][0] = pmMap[pmCorners[0]];
  faces[fIdx][1] = pmMap[pmEdgeY[0][0]];
  faces[fIdx][2] = pmMap[pmEdgeX[0][0]];
  fIdx++;

  faces[fIdx][0] = pmMap[pmSideZNeg[0][0]];
  faces[fIdx][1] = pmMap[pmEdgeX[0][0]];
  faces[fIdx][2] = pmMap[pmEdgeY[0][0]];
  fIdx++;

  _softBodyNode->addFace(faces[0]);
  _softBodyNode->addFace(faces[1]);
  _softBodyNode->addFace(faces[2]);
  _softBodyNode->addFace(faces[3]);
  _softBodyNode->addFace(faces[4]);
  _softBodyNode->addFace(faces[5]);

  // Corners[1] faces
  faces[fIdx][0] = pmMap[pmCorners[1]];
  faces[fIdx][1] = pmMap[pmEdgeY[3][0]];
  faces[fIdx][2] = pmMap[pmEdgeZ[1][0]];
  fIdx++;

  faces[fIdx][0] = pmMap[pmSideXPos[0][0]];
  faces[fIdx][1] = pmMap[pmEdgeZ[1][0]];
  faces[fIdx][2] = pmMap[pmEdgeY[3][0]];
  fIdx++;

  faces[fIdx][0] = pmMap[pmCorners[1]];
  faces[fIdx][1] = pmMap[pmEdgeZ[1][0]];
  faces[fIdx][2] = pmMap[pmEdgeX[0].back()];
  fIdx++;

  faces[fIdx][0] = pmMap[pmSideYNeg[0].back()];
  faces[fIdx][1] = pmMap[pmEdgeX[0].back()];
  faces[fIdx][2] = pmMap[pmEdgeZ[1][0]];
  fIdx++;

  faces[fIdx][0] = pmMap[pmCorners[1]];
  faces[fIdx][1] = pmMap[pmEdgeX[0].back()];
  faces[fIdx][2] = pmMap[pmEdgeY[3][0]];
  fIdx++;

  faces[fIdx][0] = pmMap[pmSideZNeg.back()[0]];
  faces[fIdx][1] = pmMap[pmEdgeY[3][0]];
  faces[fIdx][2] = pmMap[pmEdgeX[0].back()];
  fIdx++;

  _softBodyNode->addFace(faces[6]);
  _softBodyNode->addFace(faces[7]);
  _softBodyNode->addFace(faces[8]);
  _softBodyNode->addFace(faces[9]);
  _softBodyNode->addFace(faces[10]);
  _softBodyNode->addFace(faces[11]);

  // Corners[2] faces
  faces[fIdx][0] = pmMap[pmCorners[2]];
  faces[fIdx][1] = pmMap[pmEdgeZ[2][0]];
  faces[fIdx][2] = pmMap[pmEdgeY[3].back()];
  fIdx++;

  faces[fIdx][0] = pmMap[pmSideXPos.back()[0]];
  faces[fIdx][1] = pmMap[pmEdgeY[3].back()];
  faces[fIdx][2] = pmMap[pmEdgeZ[2][0]];
  fIdx++;

  faces[fIdx][0] = pmMap[pmCorners[2]];
  faces[fIdx][1] = pmMap[pmEdgeX[1].back()];
  faces[fIdx][2] = pmMap[pmEdgeZ[2][0]];
  fIdx++;

  faces[fIdx][0] = pmMap[pmSideYPos[0].back()];
  faces[fIdx][1] = pmMap[pmEdgeZ[2][0]];
  faces[fIdx][2] = pmMap[pmEdgeX[1].back()];
  fIdx++;

  faces[fIdx][0] = pmMap[pmCorners[2]];
  faces[fIdx][1] = pmMap[pmEdgeY[3].back()];
  faces[fIdx][2] = pmMap[pmEdgeX[1].back()];
  fIdx++;

  faces[fIdx][0] = pmMap[pmSideZNeg.back().back()];
  faces[fIdx][1] = pmMap[pmEdgeX[1].back()];
  faces[fIdx][2] = pmMap[pmEdgeY[3].back()];
  fIdx++;

  _softBodyNode->addFace(faces[12]);
  _softBodyNode->addFace(faces[13]);
  _softBodyNode->addFace(faces[14]);
  _softBodyNode->addFace(faces[15]);
  _softBodyNode->addFace(faces[16]);
  _softBodyNode->addFace(faces[17]);

  // Corners[3] faces
  faces[fIdx][0] = pmMap[pmCorners[3]];
  faces[fIdx][1] = pmMap[pmEdgeY[0].back()];
  faces[fIdx][2] = pmMap[pmEdgeZ[3][0]];
  fIdx++;

  faces[fIdx][0] = pmMap[pmSideXNeg.back()[0]];
  faces[fIdx][1] = pmMap[pmEdgeZ[3][0]];
  faces[fIdx][2] = pmMap[pmEdgeY[0].back()];
  fIdx++;

  faces[fIdx][0] = pmMap[pmCorners[3]];
  faces[fIdx][1] = pmMap[pmEdgeZ[3][0]];
  faces[fIdx][2] = pmMap[pmEdgeX[1][0]];
  fIdx++;

  faces[fIdx][0] = pmMap[pmSideYPos[0][0]];
  faces[fIdx][1] = pmMap[pmEdgeX[1][0]];
  faces[fIdx][2] = pmMap[pmEdgeZ[3][0]];
  fIdx++;

  faces[fIdx][0] = pmMap[pmCorners[3]];
  faces[fIdx][1] = pmMap[pmEdgeX[1][0]];
  faces[fIdx][2] = pmMap[pmEdgeY[0].back()];
  fIdx++;

  faces[fIdx][0] = pmMap[pmSideZNeg[0].back()];
  faces[fIdx][1] = pmMap[pmEdgeY[0].back()];
  faces[fIdx][2] = pmMap[pmEdgeX[1][0]];
  fIdx++;

  _softBodyNode->addFace(faces[18]);
  _softBodyNode->addFace(faces[19]);
  _softBodyNode->addFace(faces[20]);
  _softBodyNode->addFace(faces[21]);
  _softBodyNode->addFace(faces[22]);
  _softBodyNode->addFace(faces[23]);

  // Corners[4] faces
  faces[fIdx][0] = pmMap[pmCorners[4]];
  faces[fIdx][1] = pmMap[pmEdgeZ[0].back()];
  faces[fIdx][2] = pmMap[pmEdgeY[1][0]];
  fIdx++;

  faces[fIdx][0] = pmMap[pmSideXNeg[0].back()];
  faces[fIdx][1] = pmMap[pmEdgeY[1][0]];
  faces[fIdx][2] = pmMap[pmEdgeZ[0].back()];
  fIdx++;

  faces[fIdx][0] = pmMap[pmCorners[4]];
  faces[fIdx][1] = pmMap[pmEdgeX[3][0]];
  faces[fIdx][2] = pmMap[pmEdgeZ[0].back()];
  fIdx++;

  faces[fIdx][0] = pmMap[pmSideYNeg.back()[0]];
  faces[fIdx][1] = pmMap[pmEdgeZ[0].back()];
  faces[fIdx][2] = pmMap[pmEdgeX[3][0]];
  fIdx++;

  faces[fIdx][0] = pmMap[pmCorners[4]];
  faces[fIdx][1] = pmMap[pmEdgeY[1][0]];
  faces[fIdx][2] = pmMap[pmEdgeX[3][0]];
  fIdx++;

  faces[fIdx][0] = pmMap[pmSideZPos[0][0]];
  faces[fIdx][1] = pmMap[pmEdgeX[3][0]];
  faces[fIdx][2] = pmMap[pmEdgeY[1][0]];
  fIdx++;

  _softBodyNode->addFace(faces[24]);
  _softBodyNode->addFace(faces[25]);
  _softBodyNode->addFace(faces[26]);
  _softBodyNode->addFace(faces[27]);
  _softBodyNode->addFace(faces[28]);
  _softBodyNode->addFace(faces[29]);

  // Corners[5] faces
  faces[fIdx][0] = pmMap[pmCorners[5]];
  faces[fIdx][1] = pmMap[pmEdgeZ[1].back()];
  faces[fIdx][2] = pmMap[pmEdgeY[2][0]];
  fIdx++;

  faces[fIdx][0] = pmMap[pmSideXPos[0].back()];
  faces[fIdx][1] = pmMap[pmEdgeY[2][0]];
  faces[fIdx][2] = pmMap[pmEdgeZ[1].back()];
  fIdx++;

  faces[fIdx][0] = pmMap[pmCorners[5]];
  faces[fIdx][1] = pmMap[pmEdgeX[3].back()];
  faces[fIdx][2] = pmMap[pmEdgeZ[1].back()];
  fIdx++;

  faces[fIdx][0] = pmMap[pmSideYNeg.back().back()];
  faces[fIdx][1] = pmMap[pmEdgeZ[1].back()];
  faces[fIdx][2] = pmMap[pmEdgeX[3].back()];
  fIdx++;

  faces[fIdx][0] = pmMap[pmCorners[5]];
  faces[fIdx][1] = pmMap[pmEdgeY[2][0]];
  faces[fIdx][2] = pmMap[pmEdgeX[3].back()];
  fIdx++;

  faces[fIdx][0] = pmMap[pmSideZPos.back()[0]];
  faces[fIdx][1] = pmMap[pmEdgeX[3].back()];
  faces[fIdx][2] = pmMap[pmEdgeY[2][0]];
  fIdx++;

  _softBodyNode->addFace(faces[30]);
  _softBodyNode->addFace(faces[31]);
  _softBodyNode->addFace(faces[32]);
  _softBodyNode->addFace(faces[33]);
  _softBodyNode->addFace(faces[34]);
  _softBodyNode->addFace(faces[35]);

  // Corners[6] faces
  faces[fIdx][0] = pmMap[pmCorners[6]];
  faces[fIdx][1] = pmMap[pmEdgeY[2].back()];
  faces[fIdx][2] = pmMap[pmEdgeZ[2].back()];
  fIdx++;

  faces[fIdx][0] = pmMap[pmSideXPos.back().back()];
  faces[fIdx][1] = pmMap[pmEdgeZ[2].back()];
  faces[fIdx][2] = pmMap[pmEdgeY[2].back()];
  fIdx++;

  faces[fIdx][0] = pmMap[pmCorners[6]];
  faces[fIdx][1] = pmMap[pmEdgeZ[2].back()];
  faces[fIdx][2] = pmMap[pmEdgeX[2].back()];
  fIdx++;

  faces[fIdx][0] = pmMap[pmSideYPos.back().back()];
  faces[fIdx][1] = pmMap[pmEdgeX[2].back()];
  faces[fIdx][2] = pmMap[pmEdgeZ[2].back()];
  fIdx++;

  faces[fIdx][0] = pmMap[pmCorners[6]];
  faces[fIdx][1] = pmMap[pmEdgeX[2].back()];
  faces[fIdx][2] = pmMap[pmEdgeY[2].back()];
  fIdx++;

  faces[fIdx][0] = pmMap[pmSideZPos.back().back()];
  faces[fIdx][1] = pmMap[pmEdgeY[2].back()];
  faces[fIdx][2] = pmMap[pmEdgeX[2].back()];
  fIdx++;

  _softBodyNode->addFace(faces[36]);
  _softBodyNode->addFace(faces[37]);
  _softBodyNode->addFace(faces[38]);
  _softBodyNode->addFace(faces[39]);
  _softBodyNode->addFace(faces[40]);
  _softBodyNode->addFace(faces[41]);

  // Corners[7] faces
  faces[fIdx][0] = pmMap[pmCorners[7]];
  faces[fIdx][1] = pmMap[pmEdgeZ[3].back()];
  faces[fIdx][2] = pmMap[pmEdgeY[1].back()];
  fIdx++;

  faces[fIdx][0] = pmMap[pmSideXNeg.back().back()];
  faces[fIdx][1] = pmMap[pmEdgeY[1].back()];
  faces[fIdx][2] = pmMap[pmEdgeZ[3].back()];
  fIdx++;

  faces[fIdx][0] = pmMap[pmCorners[7]];
  faces[fIdx][1] = pmMap[pmEdgeX[2][0]];
  faces[fIdx][2] = pmMap[pmEdgeZ[3].back()];
  fIdx++;

  faces[fIdx][0] = pmMap[pmSideYPos.back()[0]];
  faces[fIdx][1] = pmMap[pmEdgeZ[3].back()];
  faces[fIdx][2] = pmMap[pmEdgeX[2][0]];
  fIdx++;

  faces[fIdx][0] = pmMap[pmCorners[7]];
  faces[fIdx][1] = pmMap[pmEdgeY[1].back()];
  faces[fIdx][2] = pmMap[pmEdgeX[2][0]];
  fIdx++;

  faces[fIdx][0] = pmMap[pmSideZPos[0].back()];
  faces[fIdx][1] = pmMap[pmEdgeX[2][0]];
  faces[fIdx][2] = pmMap[pmEdgeY[1].back()];
  fIdx++;

  _softBodyNode->addFace(faces[42]);
  _softBodyNode->addFace(faces[43]);
  _softBodyNode->addFace(faces[44]);
  _softBodyNode->addFace(faces[45]);
  _softBodyNode->addFace(faces[46]);
  _softBodyNode->addFace(faces[47]);

  // EdgeX[0]
  for (size_t i = 0; i < pmEdgeX[0].size() - 1; ++i)
  {
    faces[fIdx][0] = pmMap[pmEdgeX[0][i]];
    faces[fIdx][1] = pmMap[pmEdgeX[0][i + 1]];
    faces[fIdx][2] = pmMap[pmSideYNeg[0][i]];
    _softBodyNode->addFace(faces[fIdx]);
    fIdx++;

    faces[fIdx][0] = pmMap[pmSideYNeg[0][i + 1]];
    faces[fIdx][1] = pmMap[pmSideYNeg[0][i]];
    faces[fIdx][2] = pmMap[pmEdgeX[0][i + 1]];
    _softBodyNode->addFace(faces[fIdx]);
    fIdx++;

    faces[fIdx][0] = pmMap[pmEdgeX[0][i]];
    faces[fIdx][1] = pmMap[pmSideZNeg[i][0]];
    faces[fIdx][2] = pmMap[pmEdgeX[0][i + 1]];
    _softBodyNode->addFace(faces[fIdx]);
    fIdx++;

    faces[fIdx][0] = pmMap[pmSideZNeg[i + 1][0]];
    faces[fIdx][1] = pmMap[pmEdgeX[0][i + 1]];
    faces[fIdx][2] = pmMap[pmSideZNeg[i][0]];
    _softBodyNode->addFace(faces[fIdx]);
    fIdx++;
  }

  // EdgeX[1]
  for (size_t i = 0; i < pmEdgeX[1].size() - 1; ++i)
  {
    faces[fIdx][0] = pmMap[pmEdgeX[1][i]];
    faces[fIdx][1] = pmMap[pmSideYPos[0][i]];
    faces[fIdx][2] = pmMap[pmEdgeX[1][i + 1]];
    _softBodyNode->addFace(faces[fIdx]);
    fIdx++;

    faces[fIdx][0] = pmMap[pmSideYPos[0][i + 1]];
    faces[fIdx][1] = pmMap[pmEdgeX[1][i + 1]];
    faces[fIdx][2] = pmMap[pmSideYPos[0][i]];
    _softBodyNode->addFace(faces[fIdx]);
    fIdx++;

    faces[fIdx][0] = pmMap[pmEdgeX[1][i]];
    faces[fIdx][1] = pmMap[pmEdgeX[1][i + 1]];
    faces[fIdx][2] = pmMap[pmSideZNeg[i].back()];
    _softBodyNode->addFace(faces[fIdx]);
    fIdx++;

    faces[fIdx][0] = pmMap[pmSideZNeg[i + 1].back()];
    faces[fIdx][1] = pmMap[pmSideZNeg[i].back()];
    faces[fIdx][2] = pmMap[pmEdgeX[1][i + 1]];
    _softBodyNode->addFace(faces[fIdx]);
    fIdx++;
  }

  // EdgeX[2]
  for (size_t i = 0; i < pmEdgeX[2].size() - 1; ++i)
  {
    faces[fIdx][0] = pmMap[pmEdgeX[2][i + 1]];
    faces[fIdx][1] = pmMap[pmSideYPos.back()[i + 1]];
    faces[fIdx][2] = pmMap[pmEdgeX[2][i]];
    _softBodyNode->addFace(faces[fIdx]);
    fIdx++;

    faces[fIdx][0] = pmMap[pmSideYPos.back()[i]];
    faces[fIdx][1] = pmMap[pmEdgeX[2][i]];
    faces[fIdx][2] = pmMap[pmSideYPos.back()[i + 1]];
    _softBodyNode->addFace(faces[fIdx]);
    fIdx++;

    faces[fIdx][0] = pmMap[pmEdgeX[2][i + 1]];
    faces[fIdx][1] = pmMap[pmEdgeX[2][i]];
    faces[fIdx][2] = pmMap[pmSideZPos[i + 1].back()];
    _softBodyNode->addFace(faces[fIdx]);
    fIdx++;

    faces[fIdx][0] = pmMap[pmSideZPos[i].back()];
    faces[fIdx][1] = pmMap[pmSideZPos[i + 1].back()];
    faces[fIdx][2] = pmMap[pmEdgeX[2][i]];
    _softBodyNode->addFace(faces[fIdx]);
    fIdx++;
  }

  // EdgeX[3]
  for (size_t i = 0; i < pmEdgeX[3].size() - 1; ++i)
  {
    faces[fIdx][0] = pmMap[pmEdgeX[3][i]];
    faces[fIdx][1] = pmMap[pmSideYNeg.back()[i]];
    faces[fIdx][2] = pmMap[pmEdgeX[3][i + 1]];
    _softBodyNode->addFace(faces[fIdx]);
    fIdx++;

    faces[fIdx][0] = pmMap[pmSideYNeg.back()[i + 1]];
    faces[fIdx][1] = pmMap[pmEdgeX[3][i + 1]];
    faces[fIdx][2] = pmMap[pmSideYNeg.back()[i]];
    _softBodyNode->addFace(faces[fIdx]);
    fIdx++;

    faces[fIdx][0] = pmMap[pmEdgeX[3][i]];
    faces[fIdx][1] = pmMap[pmEdgeX[3][i + 1]];
    faces[fIdx][2] = pmMap[pmSideZPos[i][0]];
    _softBodyNode->addFace(faces[fIdx]);
    fIdx++;

    faces[fIdx][0] = pmMap[pmSideZPos[i + 1][0]];
    faces[fIdx][1] = pmMap[pmSideZPos[i][0]];
    faces[fIdx][2] = pmMap[pmEdgeX[3][i + 1]];
    _softBodyNode->addFace(faces[fIdx]);
    fIdx++;
  }

  // edgeY[0]
  for (size_t i = 0; i < pmEdgeY[0].size() - 1; ++i)
  {
    faces[fIdx][0] = pmMap[pmEdgeY[0][i + 1]];
    faces[fIdx][1] = pmMap[pmSideZNeg[0][i + 1]];
    faces[fIdx][2] = pmMap[pmEdgeY[0][i]];
    _softBodyNode->addFace(faces[fIdx]);
    fIdx++;

    faces[fIdx][0] = pmMap[pmSideZNeg[0][i]];
    faces[fIdx][1] = pmMap[pmEdgeY[0][i]];
    faces[fIdx][2] = pmMap[pmSideZNeg[0][i + 1]];
    _softBodyNode->addFace(faces[fIdx]);
    fIdx++;

    faces[fIdx][0] = pmMap[pmEdgeY[0][i]];
    faces[fIdx][1] = pmMap[pmSideXNeg[i][0]];
    faces[fIdx][2] = pmMap[pmEdgeY[0][i + 1]];
    _softBodyNode->addFace(faces[fIdx]);
    fIdx++;

    faces[fIdx][0] = pmMap[pmSideXNeg[i + 1][0]];
    faces[fIdx][1] = pmMap[pmEdgeY[0][i + 1]];
    faces[fIdx][2] = pmMap[pmSideXNeg[i][0]];
    _softBodyNode->addFace(faces[fIdx]);
    fIdx++;
  }

  // edgeY[1]
  for (size_t i = 0; i < pmEdgeY[1].size() - 1; ++i)
  {
    faces[fIdx][0] = pmMap[pmEdgeY[1][i]];
    faces[fIdx][1] = pmMap[pmSideZPos[0][i]];
    faces[fIdx][2] = pmMap[pmEdgeY[1][i + 1]];
    _softBodyNode->addFace(faces[fIdx]);
    fIdx++;

    faces[fIdx][0] = pmMap[pmSideZPos[0][i + 1]];
    faces[fIdx][1] = pmMap[pmEdgeY[1][i + 1]];
    faces[fIdx][2] = pmMap[pmSideZPos[0][i]];
    _softBodyNode->addFace(faces[fIdx]);
    fIdx++;

    faces[fIdx][0] = pmMap[pmEdgeY[1][i]];
    faces[fIdx][1] = pmMap[pmEdgeY[1][i + 1]];
    faces[fIdx][2] = pmMap[pmSideXNeg[i].back()];
    _softBodyNode->addFace(faces[fIdx]);
    fIdx++;

    faces[fIdx][0] = pmMap[pmSideXNeg[i + 1].back()];
    faces[fIdx][1] = pmMap[pmSideXNeg[i].back()];
    faces[fIdx][2] = pmMap[pmEdgeY[1][i + 1]];
    _softBodyNode->addFace(faces[fIdx]);
    fIdx++;
  }

  // edgeY[2]
  for (size_t i = 0; i < pmEdgeY[2].size() - 1; ++i)
  {
    faces[fIdx][0] = pmMap[pmEdgeY[2][i + 1]];
    faces[fIdx][1] = pmMap[pmSideZPos.back()[i + 1]];
    faces[fIdx][2] = pmMap[pmEdgeY[2][i]];
    _softBodyNode->addFace(faces[fIdx]);
    fIdx++;

    faces[fIdx][0] = pmMap[pmSideZPos.back()[i]];
    faces[fIdx][1] = pmMap[pmEdgeY[2][i]];
    faces[fIdx][2] = pmMap[pmSideZPos.back()[i + 1]];
    _softBodyNode->addFace(faces[fIdx]);
    fIdx++;

    faces[fIdx][0] = pmMap[pmEdgeY[2][i + 1]];
    faces[fIdx][1] = pmMap[pmEdgeY[2][i]];
    faces[fIdx][2] = pmMap[pmSideXPos[i + 1].back()];
    _softBodyNode->addFace(faces[fIdx]);
    fIdx++;

    faces[fIdx][0] = pmMap[pmSideXPos[i].back()];
    faces[fIdx][1] = pmMap[pmSideXPos[i + 1].back()];
    faces[fIdx][2] = pmMap[pmEdgeY[2][i]];
    _softBodyNode->addFace(faces[fIdx]);
    fIdx++;
  }

  // edgeY[3]
  for (size_t i = 0; i < pmEdgeY[3].size() - 1; ++i)
  {
    faces[fIdx][0] = pmMap[pmEdgeY[3][i]];
    faces[fIdx][1] = pmMap[pmSideZNeg.back()[i]];
    faces[fIdx][2] = pmMap[pmEdgeY[3][i + 1]];
    _softBodyNode->addFace(faces[fIdx]);
    fIdx++;

    faces[fIdx][0] = pmMap[pmSideZNeg.back()[i + 1]];
    faces[fIdx][1] = pmMap[pmEdgeY[3][i + 1]];
    faces[fIdx][2] = pmMap[pmSideZNeg.back()[i]];
    _softBodyNode->addFace(faces[fIdx]);
    fIdx++;

    faces[fIdx][0] = pmMap[pmEdgeY[3][i]];
    faces[fIdx][1] = pmMap[pmEdgeY[3][i + 1]];
    faces[fIdx][2] = pmMap[pmSideXPos[i][0]];
    _softBodyNode->addFace(faces[fIdx]);
    fIdx++;

    faces[fIdx][0] = pmMap[pmSideXPos[i + 1][0]];
    faces[fIdx][1] = pmMap[pmSideXPos[i][0]];
    faces[fIdx][2] = pmMap[pmEdgeY[3][i + 1]];
    _softBodyNode->addFace(faces[fIdx]);
    fIdx++;
  }

  // edgeZ[0]
  for (size_t i = 0; i < pmEdgeZ[0].size() - 1; ++i)
  {
    faces[fIdx][0] = pmMap[pmEdgeZ[0][i + 1]];
    faces[fIdx][1] = pmMap[pmSideXNeg[0][i + 1]];
    faces[fIdx][2] = pmMap[pmEdgeZ[0][i]];
    _softBodyNode->addFace(faces[fIdx]);
    fIdx++;

    faces[fIdx][0] = pmMap[pmSideXNeg[0][i]];
    faces[fIdx][1] = pmMap[pmEdgeZ[0][i]];
    faces[fIdx][2] = pmMap[pmSideXNeg[0][i + 1]];
    _softBodyNode->addFace(faces[fIdx]);
    fIdx++;

    faces[fIdx][0] = pmMap[pmEdgeZ[0][i]];
    faces[fIdx][1] = pmMap[pmSideYNeg[i][0]];
    faces[fIdx][2] = pmMap[pmEdgeZ[0][i + 1]];
    _softBodyNode->addFace(faces[fIdx]);
    fIdx++;

    faces[fIdx][0] = pmMap[pmSideYNeg[i + 1][0]];
    faces[fIdx][1] = pmMap[pmEdgeZ[0][i + 1]];
    faces[fIdx][2] = pmMap[pmSideYNeg[i][0]];
    _softBodyNode->addFace(faces[fIdx]);
    fIdx++;
  }

  // edgeZ[1]
  for (size_t i = 0; i < pmEdgeZ[1].size() - 1; ++i)
  {
    faces[fIdx][0] = pmMap[pmEdgeZ[1][i]];
    faces[fIdx][1] = pmMap[pmSideXPos[0][i]];
    faces[fIdx][2] = pmMap[pmEdgeZ[1][i + 1]];
    _softBodyNode->addFace(faces[fIdx]);
    fIdx++;

    faces[fIdx][0] = pmMap[pmSideXPos[0][i + 1]];
    faces[fIdx][1] = pmMap[pmEdgeZ[1][i + 1]];
    faces[fIdx][2] = pmMap[pmSideXPos[0][i]];
    _softBodyNode->addFace(faces[fIdx]);
    fIdx++;

    faces[fIdx][0] = pmMap[pmEdgeZ[1][i]];
    faces[fIdx][1] = pmMap[pmEdgeZ[1][i + 1]];
    faces[fIdx][2] = pmMap[pmSideYNeg[i].back()];
    _softBodyNode->addFace(faces[fIdx]);
    fIdx++;

    faces[fIdx][0] = pmMap[pmSideYNeg[i + 1].back()];
    faces[fIdx][1] = pmMap[pmSideYNeg[i].back()];
    faces[fIdx][2] = pmMap[pmEdgeZ[1][i + 1]];
    _softBodyNode->addFace(faces[fIdx]);
    fIdx++;
  }

  // edgeZ[2]
  for (size_t i = 0; i < pmEdgeZ[2].size() - 1; ++i)
  {
    faces[fIdx][0] = pmMap[pmEdgeZ[2][i + 1]];
    faces[fIdx][1] = pmMap[pmSideXPos.back()[i + 1]];
    faces[fIdx][2] = pmMap[pmEdgeZ[2][i]];
    _softBodyNode->addFace(faces[fIdx]);
    fIdx++;

    faces[fIdx][0] = pmMap[pmSideXPos.back()[i]];
    faces[fIdx][1] = pmMap[pmEdgeZ[2][i]];
    faces[fIdx][2] = pmMap[pmSideXPos.back()[i + 1]];
    _softBodyNode->addFace(faces[fIdx]);
    fIdx++;

    faces[fIdx][0] = pmMap[pmEdgeZ[2][i + 1]];
    faces[fIdx][1] = pmMap[pmEdgeZ[2][i]];
    faces[fIdx][2] = pmMap[pmSideYPos[i + 1].back()];
    _softBodyNode->addFace(faces[fIdx]);
    fIdx++;

    faces[fIdx][0] = pmMap[pmSideYPos[i].back()];
    faces[fIdx][1] = pmMap[pmSideYPos[i + 1].back()];
    faces[fIdx][2] = pmMap[pmEdgeZ[2][i]];
    _softBodyNode->addFace(faces[fIdx]);
    fIdx++;
  }

  // edgeZ[3]
  for (size_t i = 0; i < pmEdgeZ[3].size() - 1; ++i)
  {
    faces[fIdx][0] = pmMap[pmEdgeZ[3][i]];
    faces[fIdx][1] = pmMap[pmSideXNeg.back()[i]];
    faces[fIdx][2] = pmMap[pmEdgeZ[3][i + 1]];
    _softBodyNode->addFace(faces[fIdx]);
    fIdx++;

    faces[fIdx][0] = pmMap[pmSideXNeg.back()[i + 1]];
    faces[fIdx][1] = pmMap[pmEdgeZ[3][i + 1]];
    faces[fIdx][2] = pmMap[pmSideXNeg.back()[i]];
    _softBodyNode->addFace(faces[fIdx]);
    fIdx++;

    faces[fIdx][0] = pmMap[pmEdgeZ[3][i]];
    faces[fIdx][1] = pmMap[pmEdgeZ[3][i + 1]];
    faces[fIdx][2] = pmMap[pmSideYPos[i][0]];
    _softBodyNode->addFace(faces[fIdx]);
    fIdx++;

    faces[fIdx][0] = pmMap[pmSideYPos[i + 1][0]];
    faces[fIdx][1] = pmMap[pmSideYPos[i][0]];
    faces[fIdx][2] = pmMap[pmEdgeZ[3][i + 1]];
    _softBodyNode->addFace(faces[fIdx]);
    fIdx++;
  }

  // -X side
  for (size_t i = 0; i < pmSideXNeg.size() - 1; ++i)
  {
    for (size_t j = 0; j < pmSideXNeg[i].size() - 1; ++j)
    {
      faces[fIdx][0] = pmMap[pmSideXNeg[i + 0][j + 0]];
      faces[fIdx][1] = pmMap[pmSideXNeg[i + 0][j + 1]];
      faces[fIdx][2] = pmMap[pmSideXNeg[i + 1][j + 0]];
      _softBodyNode->addFace(faces[fIdx]);
      fIdx++;

      faces[fIdx][0] = pmMap[pmSideXNeg[i + 1][j + 1]];
      faces[fIdx][1] = pmMap[pmSideXNeg[i + 1][j + 0]];
      faces[fIdx][2] = pmMap[pmSideXNeg[i + 0][j + 1]];
      _softBodyNode->addFace(faces[fIdx]);
      fIdx++;
    }
  }

  // +X side
  for (size_t i = 0; i < pmSideXPos.size() - 1; ++i)
  {
    for (size_t j = 0; j < pmSideXPos[i].size() - 1; ++j)
    {
      faces[fIdx][0] = pmMap[pmSideXPos[i + 0][j + 0]];
      faces[fIdx][1] = pmMap[pmSideXPos[i + 1][j + 0]];
      faces[fIdx][2] = pmMap[pmSideXPos[i + 0][j + 1]];
      _softBodyNode->addFace(faces[fIdx]);
      fIdx++;

      faces[fIdx][0] = pmMap[pmSideXPos[i + 1][j + 1]];
      faces[fIdx][1] = pmMap[pmSideXPos[i + 0][j + 1]];
      faces[fIdx][2] = pmMap[pmSideXPos[i + 1][j + 0]];
      _softBodyNode->addFace(faces[fIdx]);
      fIdx++;
    }
  }

  // -Y side
  for (size_t i = 0; i < pmSideYNeg.size() - 1; ++i)
  {
    for (size_t j = 0; j < pmSideYNeg[i].size() - 1; ++j)
    {
      faces[fIdx][0] = pmMap[pmSideYNeg[i + 0][j + 0]];
      faces[fIdx][1] = pmMap[pmSideYNeg[i + 0][j + 1]];
      faces[fIdx][2] = pmMap[pmSideYNeg[i + 1][j + 0]];
      _softBodyNode->addFace(faces[fIdx]);
      fIdx++;

      faces[fIdx][0] = pmMap[pmSideYNeg[i + 1][j + 1]];
      faces[fIdx][1] = pmMap[pmSideYNeg[i + 1][j + 0]];
      faces[fIdx][2] = pmMap[pmSideYNeg[i + 0][j + 1]];
      _softBodyNode->addFace(faces[fIdx]);
      fIdx++;
    }
  }

  // +Y side
  for (size_t i = 0; i < pmSideYPos.size() - 1; ++i)
  {
    for (size_t j = 0; j < pmSideYPos[i].size() - 1; ++j)
    {
      faces[fIdx][0] = pmMap[pmSideYPos[i + 0][j + 0]];
      faces[fIdx][1] = pmMap[pmSideYPos[i + 1][j + 0]];
      faces[fIdx][2] = pmMap[pmSideYPos[i + 0][j + 1]];
      _softBodyNode->addFace(faces[fIdx]);
      fIdx++;

      faces[fIdx][0] = pmMap[pmSideYPos[i + 1][j + 1]];
      faces[fIdx][1] = pmMap[pmSideYPos[i + 0][j + 1]];
      faces[fIdx][2] = pmMap[pmSideYPos[i + 1][j + 0]];
      _softBodyNode->addFace(faces[fIdx]);
      fIdx++;
    }
  }

  // -Z side
  for (size_t i = 0; i < pmSideZNeg.size() - 1; ++i)
  {
    for (size_t j = 0; j < pmSideZNeg[i].size() - 1; ++j)
    {
      faces[fIdx][0] = pmMap[pmSideZNeg[i + 0][j + 0]];
      faces[fIdx][1] = pmMap[pmSideZNeg[i + 0][j + 1]];
      faces[fIdx][2] = pmMap[pmSideZNeg[i + 1][j + 0]];
      _softBodyNode->addFace(faces[fIdx]);
      fIdx++;

      faces[fIdx][0] = pmMap[pmSideZNeg[i + 1][j + 1]];
      faces[fIdx][1] = pmMap[pmSideZNeg[i + 1][j + 0]];
      faces[fIdx][2] = pmMap[pmSideZNeg[i + 0][j + 1]];
      _softBodyNode->addFace(faces[fIdx]);
      fIdx++;
    }
  }

  // +Z side
  for (size_t i = 0; i < pmSideZPos.size() - 1; ++i)
  {
    for (size_t j = 0; j < pmSideZPos[i].size() - 1; ++j)
    {
      faces[fIdx][0] = pmMap[pmSideZPos[i + 0][j + 0]];
      faces[fIdx][1] = pmMap[pmSideZPos[i + 1][j + 0]];
      faces[fIdx][2] = pmMap[pmSideZPos[i + 0][j + 1]];
      _softBodyNode->addFace(faces[fIdx]);
      fIdx++;

      faces[fIdx][0] = pmMap[pmSideZPos[i + 1][j + 1]];
      faces[fIdx][1] = pmMap[pmSideZPos[i + 0][j + 1]];
      faces[fIdx][2] = pmMap[pmSideZPos[i + 1][j + 0]];
      _softBodyNode->addFace(faces[fIdx]);
      fIdx++;
    }
  }

//  // Add to the soft body node
//  for (int i = 0; i < nFaces; ++i)
//  {
//    _softBodyNode->addFace(faces[i]);
//  }
}

//==============================================================================
void SoftBodyNodeHelper::setSinglePointMass(SoftBodyNode* _softBodyNode,
                                        double _totalMass,
                                        double _vertexStiffness,
                                        double _edgeStiffness,
                                        double _dampingCoeff)
{
  assert(_softBodyNode != NULL);

  //----------------------------------------------------------------------------
  // Misc
  //----------------------------------------------------------------------------
  _softBodyNode->setVertexSpringStiffness(_vertexStiffness);
  _softBodyNode->setEdgeSpringStiffness(_edgeStiffness);
  _softBodyNode->setDampingCoefficient(_dampingCoeff);

  //----------------------------------------------------------------------------
  // Point masses
  //----------------------------------------------------------------------------
  // Number of point masses
  size_t nPointMasses = 1;\

  // Mass per vertices
  double mass = _totalMass / nPointMasses;

  // Resting positions for each point mass
  std::vector<Eigen::Vector3d> restingPos(nPointMasses,
                                          Eigen::Vector3d::Zero());
  restingPos[0] = Eigen::Vector3d(+0.1, +0.1, +0.1);

  // Point masses
  dynamics::PointMass* newPointMass = NULL;
  for (size_t i = 0; i < nPointMasses; ++i)
  {
    newPointMass = new PointMass(_softBodyNode);
    newPointMass->setRestingPosition(restingPos[i]);
    newPointMass->setMass(mass);
    _softBodyNode->addPointMass(newPointMass);
  }
}

//==============================================================================
void SoftBodyNodeHelper::setEllipsoid(SoftBodyNode*          _softBodyNode,
                                      const Eigen::Vector3d& _size,
                                      size_t                 _nSlices,
                                      size_t                 _nStacks,
                                      double                 _totalMass,
                                      double                 _vertexStiffness,
                                      double                 _edgeStiffness,
                                      double                 _dampingCoeff)
{
  assert(_softBodyNode != NULL);

  //----------------------------------------------------------------------------
  // Misc
  //----------------------------------------------------------------------------
  _softBodyNode->setVertexSpringStiffness(_vertexStiffness);
  _softBodyNode->setEdgeSpringStiffness(_edgeStiffness);
  _softBodyNode->setDampingCoefficient(_dampingCoeff);

  //----------------------------------------------------------------------------
  // Point masses
  //----------------------------------------------------------------------------
  // Number of point masses
  int nPointMasses = (_nStacks - 1) * _nSlices + 2;

  // Mass per vertices
  double mass = _totalMass / nPointMasses;

  // Point mass pointer
  PointMass* newPointMass = NULL;

  // Resting positions for each point mass
  // -- top
  newPointMass = new dynamics::PointMass(_softBodyNode);
  newPointMass->setMass(mass);
  newPointMass->setRestingPosition(Eigen::Vector3d(0.0, 0.0, 0.5 * _size(2)));
  _softBodyNode->addPointMass(newPointMass);
  // middle
  float drho = (DART_PI / _nStacks);
  float dtheta = (DART_2PI / _nSlices);
  for (size_t i = 1; i < _nStacks; i++)
  {
    float rho = i * drho;
    float srho = (sin(rho));
    float crho = (cos(rho));

    for (size_t j = 0; j < _nSlices; j++)
    {
      float theta = (j == _nSlices) ? 0.0f : j * dtheta;
      float stheta = (-sin(theta));
      float ctheta = (cos(theta));

      float x = 0.5 * srho * stheta;
      float y = 0.5 * srho * ctheta;
      float z = 0.5 * crho;

      newPointMass = new dynamics::PointMass(_softBodyNode);
      newPointMass->setMass(mass);
      newPointMass->setRestingPosition(
            Eigen::Vector3d(x * _size(0), y * _size(1), z * _size(2)));
      _softBodyNode->addPointMass(newPointMass);
    }
  }
  // bottom
  newPointMass = new dynamics::PointMass(_softBodyNode);
  newPointMass->setMass(mass);
  newPointMass->setRestingPosition(Eigen::Vector3d(0.0, 0.0, -0.5 * _size(2)));
  _softBodyNode->addPointMass(newPointMass);


  //----------------------------------------------------------------------------
  // Edges
  //----------------------------------------------------------------------------
  // a) longitudinal
  // -- top
  for (size_t i = 0; i < _nSlices; i++)
    _softBodyNode->connectPointMasses(0, i + 1);
  // -- middle
  for (size_t i = 0; i < _nStacks - 2; i++)
    for (size_t j = 0; j < _nSlices; j++)
      _softBodyNode->connectPointMasses(i*_nSlices + j + 1,
                                        (i + 1)*_nSlices + j + 1);
  // -- bottom
  for (size_t i = 0; i < _nSlices; i++)
    _softBodyNode->connectPointMasses((_nStacks-1)*_nSlices + 1,
                                      (_nStacks-2)*_nSlices + i + 1);

  // b) latitudinal
  for (size_t i = 0; i < _nStacks - 1; i++)
  {
    for (size_t j = 0; j < _nSlices - 1; j++)
    {
      _softBodyNode->connectPointMasses(i*_nSlices + j + 1, i*_nSlices + j + 2);
    }
    _softBodyNode->connectPointMasses((i+1)*_nSlices, i*_nSlices + 1);
  }

  // c) cross (shear)
  for (size_t i = 0; i < _nStacks - 2; i++)
  {
    for (size_t j = 0; j < _nSlices - 1; j++)
    {
      _softBodyNode->connectPointMasses(i * _nSlices + j + 1,
                                        (i + 1) * _nSlices + j + 2);
      _softBodyNode->connectPointMasses(i * _nSlices + j + 2,
                                        (i + 1) * _nSlices + j + 1);
    }
    _softBodyNode->connectPointMasses((i+1)*_nSlices, (i+1)*_nSlices + 1);
    _softBodyNode->connectPointMasses(i*_nSlices + 1, (i+2)*_nSlices);
  }

  //----------------------------------------------------------------------------
  // Faces
  //----------------------------------------------------------------------------
  int meshIdx1 = 0;
  int meshIdx2 = 0;
  int meshIdx3 = 0;

  // top
  meshIdx1 = 0;
  for (size_t i = 0; i < _nSlices - 1; i++)
  {
    meshIdx2 = i + 1;
    meshIdx3 = i + 2;
    _softBodyNode->addFace(Eigen::Vector3i(meshIdx1, meshIdx2, meshIdx3));
  }
  meshIdx2 = _nSlices;
  meshIdx3 = 1;
  _softBodyNode->addFace(Eigen::Vector3i(meshIdx1, meshIdx2, meshIdx3));

  // middle
  for (size_t i = 0; i < _nStacks - 2; i++)
  {
    for (size_t j = 0; j < _nSlices - 1; j++)
    {
      meshIdx1 = i*_nSlices + j + 1;
      meshIdx2 = (i + 1)*_nSlices + j + 1;
      meshIdx3 = i*_nSlices + j + 2;
      _softBodyNode->addFace(Eigen::Vector3i(meshIdx1, meshIdx2, meshIdx3));

      meshIdx1 = i*_nSlices + j + 2;
      meshIdx2 = (i + 1)*_nSlices + j + 1;
      meshIdx3 = (i + 1)*_nSlices + j + 2;
      _softBodyNode->addFace(Eigen::Vector3i(meshIdx1, meshIdx2, meshIdx3));
    }

    meshIdx1 = (i + 1)*_nSlices;
    meshIdx2 = (i + 2)*_nSlices;
    meshIdx3 = i*_nSlices + 1;
    _softBodyNode->addFace(Eigen::Vector3i(meshIdx1, meshIdx2, meshIdx3));

    meshIdx1 = i*_nSlices + 1;
    meshIdx2 = (i + 2)*_nSlices;
    meshIdx3 = (i + 2)*_nSlices + 1;
    _softBodyNode->addFace(Eigen::Vector3i(meshIdx1, meshIdx2, meshIdx3));
  }

  // bottom
  meshIdx1 = (_nStacks-1)*_nSlices + 1;
  for (size_t i = 0; i < _nSlices - 1; i++)
  {
    meshIdx2 = (_nStacks-2)*_nSlices + i + 2;
    meshIdx3 = (_nStacks-2)*_nSlices + i + 1;
    _softBodyNode->addFace(Eigen::Vector3i(meshIdx1, meshIdx2, meshIdx3));
  }
  meshIdx2 = (_nStacks-2)*_nSlices + 2;
  meshIdx3 = (_nStacks-1)*_nSlices;
  _softBodyNode->addFace(Eigen::Vector3i(meshIdx1, meshIdx2, meshIdx3));
}

//==============================================================================
void SoftBodyNodeHelper::setCylinder(SoftBodyNode* _softBodyNode,
                                     double _radius,
                                     double _height,
                                     size_t _nSlices,
                                     size_t _nStacks,
                                     size_t _nRings,
                                     double _totalMass,
                                     double _vertexStiffness,
                                     double _edgeStiffness,
                                     double _dampingCoeff)
{
  assert(_softBodyNode != NULL);

  //----------------------------------------------------------------------------
  // Misc
  //----------------------------------------------------------------------------
  _softBodyNode->setVertexSpringStiffness(_vertexStiffness);
  _softBodyNode->setEdgeSpringStiffness(_edgeStiffness);
  _softBodyNode->setDampingCoefficient(_dampingCoeff);

  //----------------------------------------------------------------------------
  // Point masses
  //----------------------------------------------------------------------------
  // Number of point masses
  size_t nTopPointMasses = _nSlices * (_nRings - 1) + 1;
  size_t nDrumPointMasses = (_nStacks + 1) * _nSlices;
  size_t nTotalMasses = nDrumPointMasses + 2 * nTopPointMasses;

  // Mass per vertices
  double mass = _totalMass / nTotalMasses;

  // Point mass pointer
  PointMass* newPointMass = NULL;

  // Resting positions for each point mass
  float dradius = _radius / static_cast<float>(_nRings);
  float dtheta = DART_2PI / static_cast<float>(_nSlices);

  // -- top
  newPointMass = new dynamics::PointMass(_softBodyNode);
  newPointMass->setMass(mass);
  newPointMass->setRestingPosition(Eigen::Vector3d(0.0, 0.0, 0.5 * _height));
  _softBodyNode->addPointMass(newPointMass);
  for (size_t i = 1; i < _nRings; ++i)
  {
    float z = 0.5;
    float radius = i * dradius;

    for (size_t j = 0; j < _nSlices; j++)
    {
      float theta = (j == _nSlices) ? 0.0f : j * dtheta;
      float stheta = (-sin(theta));
      float ctheta = (cos(theta));

      float x = stheta;
      float y = ctheta;

      newPointMass = new dynamics::PointMass(_softBodyNode);
      newPointMass->setMass(mass);
      newPointMass->setRestingPosition(
            Eigen::Vector3d(x * radius, y * radius, z * _height));
      _softBodyNode->addPointMass(newPointMass);
    }
  }

  // -- middle
  float dz     = -1.0 / static_cast<float>(_nStacks);
  for (size_t i = 0; i < _nStacks + 1; i++)
  {
    float z = 0.5 + i * dz;

    for (size_t j = 0; j < _nSlices; j++)
    {
      float theta = (j == _nSlices) ? 0.0f : j * dtheta;
      float stheta = (-sin(theta));
      float ctheta = (cos(theta));

      float x = stheta;
      float y = ctheta;

      newPointMass = new dynamics::PointMass(_softBodyNode);
      newPointMass->setMass(mass);
      newPointMass->setRestingPosition(
            Eigen::Vector3d(x * _radius, y * _radius, z * _height));
      _softBodyNode->addPointMass(newPointMass);
    }
  }

  // -- bottom
  for (size_t i = 1; i < _nRings; ++i)
  {
    float z = -0.5;
    float radius = _radius - i * dradius;

    for (size_t j = 0; j < _nSlices; j++)
    {
      float theta = (j == _nSlices) ? 0.0f : j * dtheta;
      float stheta = (-sin(theta));
      float ctheta = (cos(theta));

      float x = stheta;
      float y = ctheta;

      newPointMass = new dynamics::PointMass(_softBodyNode);
      newPointMass->setMass(mass);
      newPointMass->setRestingPosition(
            Eigen::Vector3d(x * radius, y * radius, z * _height));
      _softBodyNode->addPointMass(newPointMass);
    }
  }
  newPointMass = new dynamics::PointMass(_softBodyNode);
  newPointMass->setMass(mass);
  newPointMass->setRestingPosition(
        Eigen::Vector3d(0.0, 0.0, -0.5 * _height));
  _softBodyNode->addPointMass(newPointMass);

  //----------------------------------------------------------------------------
  // Edges
  //----------------------------------------------------------------------------
  // A. Drum part

  // a) longitudinal
  // -- top
  for (size_t i = 0; i < _nSlices; i++)
    _softBodyNode->connectPointMasses(0, i + 1);
  for (size_t i = 0; i < _nRings - 1; i++)
  {
    for (size_t j = 0; j < _nSlices; j++)
    {
      _softBodyNode->connectPointMasses(
            _nSlices + 1 + (i + 0) * _nSlices + j,
            _nSlices + 1 + (i + 1) * _nSlices + j);
    }
  }
  // -- middle
  for (size_t i = 0; i < _nStacks - 1; i++)
  {
    for (size_t j = 0; j < _nSlices; j++)
    {
      _softBodyNode->connectPointMasses(
            nTopPointMasses + (i + 0) * _nSlices + j,
            nTopPointMasses + (i + 1) * _nSlices + j);
    }
  }
  // -- bottom
  for (size_t i = 0; i < _nRings - 1; i++)
  {
    for (size_t j = 0; j < _nSlices; j++)
    {
      _softBodyNode->connectPointMasses(
            nTopPointMasses + (nDrumPointMasses - _nSlices)
            + (i + 0) * _nSlices + j,
            nTopPointMasses + (nDrumPointMasses - _nSlices)
            + (i + 1) * _nSlices + j);
    }
  }
  for (size_t i = 1; i < _nSlices; i++)
    _softBodyNode->connectPointMasses(nTotalMasses - 1 - i,
                                      nTotalMasses - 1);

  // b) latitudinal
  for (size_t i = 0; i < _nStacks; i++)
  {
    for (size_t j = 0; j < _nSlices - 1; j++)
    {
      _softBodyNode->connectPointMasses(
            nTopPointMasses + i * _nSlices + j + 0,
            nTopPointMasses + i * _nSlices + j + 1);
    }

    _softBodyNode->connectPointMasses(
          nTopPointMasses + (i + 0) * _nSlices + _nSlices - 1,
          nTopPointMasses + (i + 0) * _nSlices);
  }
  // -- disk parts
  // TODO(JS): No latitudinal connections for top and bottom disks

  // c) cross (shear)
  // -- drum part
  for (size_t i = 0; i < _nStacks - 2; i++)
  {
    for (size_t j = 0; j < _nSlices - 1; j++)
    {
      _softBodyNode->connectPointMasses(
            nTopPointMasses + (i + 0) * _nSlices + j + 0,
            nTopPointMasses + (i + 1) * _nSlices + j + 1);
      _softBodyNode->connectPointMasses(
            nTopPointMasses + (i + 0) * _nSlices + j + 1,
            nTopPointMasses + (i + 1) * _nSlices + j + 0);
    }

    _softBodyNode->connectPointMasses(
          nTopPointMasses + (i + 0) * _nSlices + _nSlices - 1,
          nTopPointMasses + (i + 1) * _nSlices);
    _softBodyNode->connectPointMasses(
          nTopPointMasses + (i + 0) * _nSlices,
          nTopPointMasses + (i + 1) * _nSlices + _nSlices - 1);
  }
  // -- disk parts
  // TODO(JS): No cross connections for top and bottom disks

  //----------------------------------------------------------------------------
  // Faces
  //----------------------------------------------------------------------------
  int meshIdx1 = 0;
  int meshIdx2 = 0;
  int meshIdx3 = 0;

  // top
  size_t nConePointMass = 1;
  meshIdx1 = 0;
  for (size_t i = 0; i < _nSlices - 1; i++)
  {
    meshIdx2 = i + 1;
    meshIdx3 = i + 2;
    _softBodyNode->addFace(Eigen::Vector3i(meshIdx1, meshIdx2, meshIdx3));
  }
  meshIdx2 = _nSlices;
  meshIdx3 = 1;
  _softBodyNode->addFace(Eigen::Vector3i(meshIdx1, meshIdx2, meshIdx3));
  for (size_t i = 0; i < _nRings - 1; ++i)
  {
    for (size_t j = 0; j < _nSlices - 1; ++j)
    {
      meshIdx1 = (i + 0) * _nSlices + j;
      meshIdx2 = (i + 1) * _nSlices + j;
      meshIdx3 = (i + 0) * _nSlices + j + 1;
      _softBodyNode->addFace(Eigen::Vector3i(nConePointMass + meshIdx1,
                                             nConePointMass + meshIdx2,
                                             nConePointMass + meshIdx3));

      meshIdx1 = (i + 0) * _nSlices + j + 1;
      meshIdx2 = (i + 1) * _nSlices + j;
      meshIdx3 = (i + 1) * _nSlices + j + 1;
      _softBodyNode->addFace(Eigen::Vector3i(nConePointMass + meshIdx1,
                                             nConePointMass + meshIdx2,
                                             nConePointMass + meshIdx3));
    }

    meshIdx1 = (i + 0) * _nSlices + _nSlices - 1;
    meshIdx2 = (i + 1) * _nSlices + _nSlices - 1;
    meshIdx3 = (i + 0) * _nSlices + 0;
    _softBodyNode->addFace(Eigen::Vector3i(nConePointMass + meshIdx1,
                                           nConePointMass + meshIdx2,
                                           nConePointMass + meshIdx3));

    meshIdx1 = (i + 0) * _nSlices + 0;
    meshIdx2 = (i + 1) * _nSlices + _nSlices - 1;
    meshIdx3 = (i + 1) * _nSlices + 0;
    _softBodyNode->addFace(Eigen::Vector3i(nConePointMass + meshIdx1,
                                           nConePointMass + meshIdx2,
                                           nConePointMass + meshIdx3));
  }

  // middle
  for (size_t i = 0; i < _nStacks; i++)
  {
    for (size_t j = 0; j < _nSlices - 1; j++)
    {
      meshIdx1 = (i + 0) * _nSlices + j;
      meshIdx2 = (i + 1) * _nSlices + j;
      meshIdx3 = (i + 0) * _nSlices + j + 1;
      _softBodyNode->addFace(Eigen::Vector3i(nTopPointMasses + meshIdx1,
                                             nTopPointMasses + meshIdx2,
                                             nTopPointMasses + meshIdx3));

      meshIdx1 = (i + 0) * _nSlices + j + 1;
      meshIdx2 = (i + 1) * _nSlices + j;
      meshIdx3 = (i + 1) * _nSlices + j + 1;
      _softBodyNode->addFace(Eigen::Vector3i(nTopPointMasses + meshIdx1,
                                             nTopPointMasses + meshIdx2,
                                             nTopPointMasses + meshIdx3));
    }

    meshIdx1 = (i + 0) * _nSlices;
    meshIdx2 = (i + 1) * _nSlices;
    meshIdx3 = (i + 0) * _nSlices + _nSlices - 1;
    _softBodyNode->addFace(Eigen::Vector3i(nTopPointMasses + meshIdx1,
                                           nTopPointMasses + meshIdx2,
                                           nTopPointMasses + meshIdx3));

    meshIdx1 = (i + 0) * _nSlices + _nSlices - 1;
    meshIdx2 = (i + 1) * _nSlices + 0;
    meshIdx3 = (i + 1) * _nSlices + _nSlices - 1;
    _softBodyNode->addFace(Eigen::Vector3i(nTopPointMasses + meshIdx1,
                                           nTopPointMasses + meshIdx2,
                                           nTopPointMasses + meshIdx3));
  }

  // bottom
  for (size_t i = 0; i < _nRings - 1; ++i)
  {
    for (size_t j = 0; j < _nSlices - 1; ++j)
    {
      meshIdx1 = (i + 0) * _nSlices + j;
      meshIdx2 = (i + 1) * _nSlices + j;
      meshIdx3 = (i + 0) * _nSlices + j + 1;
      _softBodyNode->addFace(
            Eigen::Vector3i(
              nTopPointMasses + (nDrumPointMasses - _nSlices) + meshIdx1,
              nTopPointMasses + (nDrumPointMasses - _nSlices) + meshIdx2,
              nTopPointMasses + (nDrumPointMasses - _nSlices) + meshIdx3));

      meshIdx1 = (i + 0) * _nSlices + j + 1;
      meshIdx2 = (i + 1) * _nSlices + j;
      meshIdx3 = (i + 1) * _nSlices + j + 1;
      _softBodyNode->addFace(
            Eigen::Vector3i(
              nTopPointMasses + (nDrumPointMasses - _nSlices) + meshIdx1,
              nTopPointMasses + (nDrumPointMasses - _nSlices) + meshIdx2,
              nTopPointMasses + (nDrumPointMasses - _nSlices) + meshIdx3));
    }

    meshIdx1 = (i + 0) * _nSlices + _nSlices - 1;
    meshIdx2 = (i + 1) * _nSlices + _nSlices - 1;
    meshIdx3 = (i + 0) * _nSlices + 0;
    _softBodyNode->addFace(
          Eigen::Vector3i(
            nTopPointMasses + (nDrumPointMasses - _nSlices) + meshIdx1,
            nTopPointMasses + (nDrumPointMasses - _nSlices) + meshIdx2,
            nTopPointMasses + (nDrumPointMasses - _nSlices) + meshIdx3));

    meshIdx1 = (i + 0) * _nSlices + 0;
    meshIdx2 = (i + 1) * _nSlices + _nSlices - 1;
    meshIdx3 = (i + 1) * _nSlices + 0;
    _softBodyNode->addFace(
          Eigen::Vector3i(
            nTopPointMasses + (nDrumPointMasses - _nSlices) + meshIdx1,
            nTopPointMasses + (nDrumPointMasses - _nSlices) + meshIdx2,
            nTopPointMasses + (nDrumPointMasses - _nSlices) + meshIdx3));
  }
  meshIdx1 = 1;
  for (size_t i = 0; i < _nSlices - 1; i++)
  {
    meshIdx2 = i + 2;
    meshIdx3 = i + 3;
    _softBodyNode->addFace(Eigen::Vector3i(
                             nTotalMasses - meshIdx1,
                             nTotalMasses - meshIdx2,
                             nTotalMasses - meshIdx3));
  }
  meshIdx2 = _nSlices + 1;
  meshIdx3 = 2;
  _softBodyNode->addFace(Eigen::Vector3i(
                           nTotalMasses - meshIdx1,
                           nTotalMasses - meshIdx2,
                           nTotalMasses - meshIdx3));
}

}  // namespace dynamics
}  // namespace dart

