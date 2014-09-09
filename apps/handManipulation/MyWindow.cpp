/*
 * Copyright (c) 2013-2014, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Yunfei Bai <ybai30@mail.gatech.edu>,
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

#include "MyWindow.h"

#include "dart/common/Timer.h"
#include "dart/math/Helpers.h"
#include "dart/math/Geometry.h"
#include "dart/dynamics/BodyNode.h"
//#include "dart/dynamics/Dof.h"
#include "dart/dynamics/Joint.h"
#include "dart/constraint/ConstraintSolver.h"
#include "dart/collision/CollisionDetector.h"
#include "dart/collision/fcl_mesh/FCLMeshCollisionDetector.h"
#include "dart/gui/GLFuncs.h"
//#include "dart/optimizer/ObjectiveBox.h"
//#include "dart/optimizer/snopt/SnoptSolver.h"
//#include "dart/optimizer/Var.h"

#include "Controller.h"
//#include "IKProblem.h"
//#include "PositionConstraint.h"
//#include "OrientationConstraint.h"
//#include "ContactForceProblem.h"
//#include "ForceProjConstraint.h"
//#include "ForceTorqueConstraint.h"
//#include "ForceMagniConstraint.h"
//#include "ForcePlaneConstraint.h"
//#include "LCPInvProblem.h"
//#include "LCPConstraint.h"
//#include "BoundEllipsoidProblem.h"
//#include "EllipsoidVolumeConstraint.h"
//#include "InEllipsoidConstraint.h"

#define EPS_DISTANCE 0.001

using namespace std;
using namespace Eigen;

using namespace dart;
using namespace common;
using namespace dynamics;
using namespace collision;
using namespace constraint;
using namespace tasks;
using namespace optimizer;
#ifdef WIN32
using namespace nv;
#endif

const char * const MyWindow::mFileName = "roll_cube";
#ifdef WIN32
nv::GlutUIContext ui;
#endif
float initPose[] = {-0.8000000119, 0, 0, 0, 0, 0, 0, -0.7428935766, 0, 0, 0, 0, 1.072659612, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.8000000119, 0, 0.200000003};
//float initObjPose[] = {0.0, 0.0, 0.0, 0.08, 0.15, 0.43};
float initObjPose[] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

static dart::common::Timer tIter("timeIter");
static dart::common::Timer tInternal("timeInternal");
static dart::common::Timer tExternal("timeExternal");
static dart::common::Timer tSimulation("timeSimulation");
static dart::common::Timer tHandIK("timeHandIK");
static dart::common::Timer tSetPose("timeSetPose");
static dart::common::Timer tContact("timeContact");
static dart::common::Timer tJointLimit("timeJointLimit");

double extTorque = 0.0;

MyWindow::MyWindow(Skeleton* _mList, ...): Win3D()
{
  mBackground[0] = 1.0;
  mBackground[1] = 1.0;
  mBackground[2] = 1.0;
  mBackground[3] = 1.0;

  mSim = false;
  mPlay = false;
  mSimFrame = 0;
  mPlayFrame = 0;
  mShowMarkers = false;
  mDrawOri = false;
  mDrawIK = false;
  mDrawModel = true;
  mDrawTarget = false;
  mDrawBoundEllipsoid = false;
  mDrawContactForce = false;

  liftFlag = false;
  mAxisAngle.setZero();
  mTargetAxisAngle.setZero();

  mPersp = 30.f;

  mGravity = Eigen::Vector3d(0.0, -9.81, 0.0);
  mForce = Eigen::Vector3d::Zero();
  mFingerNum = 5;
  mTimeStep = 1.0/1000.0;
  mFrictionBasis = 4;
  mBoundEllipsoidR = Eigen::Vector3d(0.0,0.0,0.0);
  mObjRadius = 0.032787;
  int numEdges = 4;

  // the order is determined by the order of pivoting, related to roll direction
  //
  //      Y
  //      ^
  //      |  mEdges[1] ------- mEdges[2]
  //      |    |                 |
  // Z <---    |                 |
  //           |                 |
  //         mEdges[0] ------- mEdges[3]
  //
  mEdges.resize(numEdges);
  mEdges[3] = Eigen::Vector3d(0.0, -0.02, -0.02);
  mEdges[2] = Eigen::Vector3d(0.0,  0.02, -0.02);
  mEdges[1] = Eigen::Vector3d(0.0,  0.02,  0.02);
  mEdges[0] = Eigen::Vector3d(0.0, -0.02,  0.02);

  //      Y
  //      ^      mCorners[1][1] ---- mCorners[2][1]
  //      |        /|                  /|
  //      |       / |                 / |
  // Z <---      /  |                /  |
  //     /   mCorners[1][0] ---- mCorners[2][0]
  //    /       |mCorners[0][1] ----|mCorners[3][1]
  //   X        |  /                |  /
  //            | /                 | /
  //            |/                  |/
  //         mCorners[0][0] ---- mCorners[3][0]
  //
  //                    ___
  //                    |   \_
  //          ___________|    \__
  //       __/________           /______
  //      /__________
  //      |/________           _______
  //       |_/________________/|______
  //         |________________|/
  //
  //
  mCorners.resize(numEdges);
  for (int i = 0; i < numEdges; ++i)
  {
    mCorners[i].resize(2);
    mCorners[i][0] = mCorners[i][1] = mEdges[i];
    mCorners[i][0][0] =  0.02;
    mCorners[i][1][0] = -0.02;
  }

  // Number of rolling + 1
  mN = 3;

  mBackN = 0;
  mRollNum = 0;
  mRollBackNum = 0;
  mRollBackIndex = 0;
  mAngles.resize(mN);

  // Precomputed angles
  mAngles[0] = 0.740675;
  mAngles[1] = 0.137237;
  mAngles[2] = -0.358023;

  if (_mList)
  {
    mSkels.push_back(_mList);
    va_list ap;
    va_start(ap, _mList);
    while (true)
    {
      dart::dynamics::Skeleton* skel = va_arg(ap, dart::dynamics::Skeleton*);
      if(skel)
        mSkels.push_back(skel);
      else
        break;
    }
    va_end(ap);
  }

  mDofNum = mSkels[1]->getPositions().size();
  mActiveDofIndex = 0;
  mObjDofNum = mSkels[2]->getPositions().size();
  mObjActiveDofIndex = 0;

  int sumNDofs = 0;
  mIndices.push_back(sumNDofs);
  for (unsigned int i = 0; i < mSkels.size(); i++)
  {
    int nDofs = mSkels[i]->getNumDofs();
    sumNDofs += nDofs;
    mIndices.push_back(sumNDofs);
  }

  // TODO(JS): Just commented out
  initDyn();
}

void MyWindow::initDyn()
{
  mDofs.resize(mSkels.size());
  mDofVels.resize(mSkels.size());
  mDofAccs.resize(mSkels.size());

  for (unsigned int i = 0; i < mSkels.size(); i++)
  {
    mSkels[i]->init();
    mDofs[i].resize(mSkels[i]->getNumDofs());
    mDofVels[i].resize(mSkels[i]->getNumDofs());
    mDofAccs[i].resize(mSkels[i]->getNumDofs());
    mDofs[i].setZero();
    mDofVels[i].setZero();
    mDofAccs[i].setZero();
  }

  // initial position of the ground
  mDofs[0][4] = -0.3;

  // initial pose for hand
  for (size_t i = 0; i < mSkels[1]->getNumDofs(); i++)
    mDofs[1][i] = initPose[i];

  // modify hand pose based on rolling, related to roll direction
  //mDofs[1][2] = -mAngles[0];

  Isometry3d T = Isometry3d::Identity();
  T.translation()[0] = initObjPose[3];
  T.translation()[1] = initObjPose[4];
  T.translation()[2] = initObjPose[5];
  Vector6d initObjPoseScrewParam = math::logMap(T);

  // initial position of the box
  for (size_t i = 0; i < mSkels[2]->getNumDofs(); i++)
    mDofs[2][i] = initObjPoseScrewParam[i];

  for (unsigned int i = 0; i < mSkels.size(); i++)
  {
    mSkels[i]->setGravity(mGravity);
    mSkels[i]->setPositions(mDofs[i]);
    mSkels[i]->setVelocities(mDofVels[i]);
    mSkels[i]->computeForwardKinematics(true, true, false);
    //mSkels[i]->computeDynamics(mGravity, mDofVels[i], false);
  }

  // modify box pose based on rolling, related to roll direction
  Vector3d localPos(-0.01, -(0.02 + 0.011 + 0.0005), 0.06);
  Vector3d worldPos = mSkels[1]->getBodyNode(4)->getTransform() * localPos;

  T.translation() = worldPos;

  //mDofs[2].tail<3>() = worldPos;
  mDofs[2] = math::logMap(T);

  for (unsigned int i = 0; i < mSkels.size(); i++)
  {
    mSkels[i]->init();
    mSkels[i]->setGravity(mGravity);
    mSkels[i]->setPositions(mDofs[i]);
    mSkels[i]->setVelocities(mDofVels[i]);
    mSkels[i]->computeForwardKinematics(true, true, false);
    //mSkels[i]->computeDynamics(mGravity, mDofVels[i], false);
  }

  // the the initial target angle of the palm, related to roll direction
  setHandAngle(mAngles[0]);
  mPreOri = mSkels[1]->getPositions().head<3>();
  mPreContactEdge = 0;

  mUpFace = 2;

  mPreHighCorners.resize(2);
  mCurHighCorners.resize(2);
  mPreHighCorners[0] = mSkels[1]->getBodyNode(4)->getTransform().inverse()
                       * mSkels[2]->getBodyNode(0)->getTransform()
                       * mCorners[1][0];
  mPreHighCorners[1] = mSkels[1]->getBodyNode(4)->getTransform().inverse()
                       * mSkels[2]->getBodyNode(0)->getTransform()
                       * mCorners[2][0];
  mCurHighCorners[0] = mPreHighCorners[0];
  mCurHighCorners[1] = mPreHighCorners[1];

  mRollDir = 0;

  // set joint limit
  mSkels[1]->setPositionUpperLimit(2,  1.0);
  mSkels[1]->setPositionLowerLimit(2, -1.0);

  mSkels[1]->setPositionUpperLimit(3,  0.5);
  mSkels[1]->setPositionLowerLimit(3, -0.5);

  mSkels[1]->setPositionUpperLimit(4,  1.0);
  mSkels[1]->setPositionLowerLimit(4, -1.0);

  mSkels[1]->setPositionUpperLimit(5,  0.5);
  mSkels[1]->setPositionLowerLimit(5, -0.5);

  mSkels[1]->setPositionUpperLimit(6,  0.5);
  mSkels[1]->setPositionLowerLimit(6, -0.5);

  mSkels[1]->setPositionUpperLimit(7,  1.2);
  mSkels[1]->setPositionLowerLimit(7, -1.2);

  mSkels[1]->setPositionUpperLimit(8,  1.7);
  mSkels[1]->setPositionLowerLimit(8, -0.4);

  mSkels[1]->setPositionUpperLimit(9,  0.5);
  mSkels[1]->setPositionLowerLimit(9, -0.5);

  mSkels[1]->setPositionUpperLimit(10,  1.7);
  mSkels[1]->setPositionLowerLimit(10, -0.4);

  mSkels[1]->setPositionUpperLimit(11,  1.7);
  mSkels[1]->setPositionLowerLimit(11, -0.4);

  mSkels[1]->setPositionUpperLimit(12,  1.2);
  mSkels[1]->setPositionLowerLimit(12, -0.3);

  mSkels[1]->setPositionUpperLimit(13,  1.8);
  mSkels[1]->setPositionLowerLimit(13, -0.5);

  mSkels[1]->setPositionUpperLimit(14,  1.7);
  mSkels[1]->setPositionLowerLimit(14, -0.4);

  mSkels[1]->setPositionUpperLimit(15,  1.8);
  mSkels[1]->setPositionLowerLimit(15, -0.5);

  mSkels[1]->setPositionUpperLimit(16,  1.8);
  mSkels[1]->setPositionLowerLimit(16, -0.5);

  mSkels[1]->setPositionUpperLimit(17,  1.0);
  mSkels[1]->setPositionLowerLimit(17, -1.0);

  mSkels[1]->setPositionUpperLimit(18,  1.5);
  mSkels[1]->setPositionLowerLimit(18, -0.4);

  mSkels[1]->setPositionUpperLimit(19,  1.8);
  mSkels[1]->setPositionLowerLimit(19, -0.5);

  mSkels[1]->setPositionUpperLimit(20,  1.5);
  mSkels[1]->setPositionLowerLimit(20, -0.4);

  mSkels[1]->setPositionUpperLimit(21,  1.5);
  mSkels[1]->setPositionLowerLimit(21, -0.4);

  mSkels[1]->setPositionUpperLimit(22,  1.8);
  mSkels[1]->setPositionLowerLimit(22, -0.5);

  mSkels[1]->setPositionUpperLimit(23,  1.5);
  mSkels[1]->setPositionLowerLimit(23, -1.4);

  mSkels[1]->setPositionUpperLimit(24,  1.5);
  mSkels[1]->setPositionLowerLimit(24, -1.4);

  // set the ground to be an immobile object. It will still participate in
  // collision
  mSkels[0]->setMobile(false);

  // set self collidable
  // TODO(JS): Disabled self collision
  //mSkels[1]->enableSelfCollision();
  mSkels[1]->disableSelfCollision();

  // create a collision handler
  // set the number of friction basis
  mFrictionBasis = 4;
  mConstraintSolver = new ConstraintSolver(mTimeStep);
  // Add skeletons to constraint solver
  mConstraintSolver->addSkeletons(mSkels);
  // Set friction coefficient as 1.5
  for (size_t i = 0; i < mSkels.size(); ++i)
  {
    for (size_t j = 0; j < mSkels[i]->getNumBodyNodes(); ++j)
      mSkels[i]->getBodyNode(j)->setFrictionCoeff(1.5);
  }

  mInContactVec.resize(mFingerNum);
  mContactForceVec.resize(mFingerNum);
  mFingerTargetContactForceVec.resize(mFingerNum);
  mFingerContactPointVec.resize(mFingerNum);
  mContactForces.resize(mFingerNum);
  mContactNormals.resize(mFingerNum);
  mContactPointsBallStart.resize(mFingerNum);
  mContactPointsBallEnd.resize(mFingerNum);
  mTargetContactPoints.resize(mFingerNum);
  mFingerContactPoints.resize(mFingerNum);
  mFingerTargetContactForces.resize(mFingerNum);

  mFingerTipNames.resize(mFingerNum);
  mFingerRootNames.resize(mFingerNum);

  mPalmName = "palm";

  mFingerTipNames[0] = "thdistal";
  mFingerTipNames[1] = "ffdistal";
  mFingerTipNames[2] = "mfdistal";
  mFingerTipNames[3] = "rfdistal";
  mFingerTipNames[4] = "lfdistal";

  mFingerRootNames[0] = "thbase";
  mFingerRootNames[1] = "ffknuckle";
  mFingerRootNames[2] = "mfknuckle";
  mFingerRootNames[3] = "rfknuckle";
  mFingerRootNames[4] = "lfknuckle";

  // create controller
  mController = new Controller(mSkels[1], mTimeStep, mTasks, mFingerNum,
                               mFingerRootNames, mFingerTipNames, mPalmName);


  // plan related

  // TODO(JS): Just commented out
  mController->mOriFlag = true;
  mTargetOri = Vector3d(0.0,0.0,1.0).normalized();
  mPalmObjAngle = 0.0;


  // initialize task
  /*
  updateTaskArray();
  mController->resetController(mTasks); // every time update task array, need to reset controller to adapt to new tasks
  */

  // initialize target contact point

  updateContactPoint();

  // initialize hand pose
  // if the initial configuration is all the fingers touch the object
  for (int i = 0; i < mFingerNum; ++i)
  {
    mContactPointsBallStart[i]
        = cartesianToBall(mSkels[2]->getBodyNode(0)->getTransform().inverse()
                          * mFingerContactPoints[i],
                          mObjRadius);
  }
  updateHandPose();

  // 	mSkels[1]->setPose(mController->mFingerEndPose, true, true);

  mConstraintSolver->solve();

  // solve bounding ellipsoid
  // 	solveBoundEllipsoid();
  mBoundEllipsoidR(0) = mBoundEllipsoidR(1) = mBoundEllipsoidR(2) = 0.025;
}

//void MyWindow::solveBoundEllipsoid()
//{
//  // an example of solving the bounding ellipsoid
//  std::vector<Vector3d> featurePoints;
//  Vector3d firstPoint = Vector3d(mObjRadius,mObjRadius,mObjRadius);
//  Vector3d secondPoint = Vector3d(-mObjRadius,-mObjRadius,-mObjRadius);
//  featurePoints.push_back(firstPoint);
//  featurePoints.push_back(secondPoint);
//  BoundEllipsoidProblem prob(featurePoints);
//  snopt::SnoptSolver solver(&prob);
//  solver.solve();
//  for (int i = 0; i < 3; ++i)
//  {
//    std::cout << i << " : " << prob.vars()[i]->mVal << std::endl;
//    mBoundEllipsoidR(i) = prob.vars()[i]->mVal;
//    mBoundEllipsoidR(i) += 0.005;
//  }
//}

void MyWindow::setObjInitPose()
{
  mSkels[2]->setGravity(mGravity);
  mSkels[2]->setPositions(mDofs[2]);
  mSkels[2]->setVelocities(mDofVels[2]);
  mSkels[2]->computeForwardKinematics(true, true, false);
}

void MyWindow::setHandInitPose()
{
  mSkels[1]->setGravity(mGravity);
  mSkels[1]->setPositions(mDofs[1]);
  mSkels[1]->setVelocities(mDofVels[1]);
  mSkels[1]->computeForwardKinematics(true, true, false);
}

void MyWindow::saveHandInitPose()
{
  std::string fileName = "init_pose.txt";
  std::ofstream outFile(fileName.c_str());
  outFile.precision(10);
  for (int i = 0; i < mSkels[1]->getNumDofs(); ++i)
    outFile << mDofs[1][i] << ", ";
}

//VectorXd MyWindow::getState()
//{
//  VectorXd state(mIndices.back() * 2);
//  for (unsigned int i = 0; i < mSkels.size(); i++) {
//    int start = mIndices[i] * 2;
//    int size = mDofs[i].size();
//    state.segment(start, size) = mDofs[i];
//    state.segment(start + size, size) = mDofVels[i];
//  }
//  return state;
//}

//VectorXd MyWindow::evalDeriv()
//{
//  VectorXd deriv = VectorXd::Zero(mIndices.back() * 2);

//  VectorXd extForce = mSkels[2]->getExternalForces().transpose();
//  extForce(4) = extTorque;
//  mSkels[2]->setExternalForces(extForce);

//  for (unsigned int i = 0; i < mSkels.size(); i++)
//  {
//    // skip immobile objects in forward simulation
//    if (!mSkels[i]->isMobile())
//      continue;
//    int start = mIndices[i] * 2;
//    int size = mDofs[i].size();

//    VectorXd qddot;
//    qddot = (mSkels[i]->getMassMatrix()).fullPivHouseholderQr().solve(
//              -mSkels[i]->getCombinedVector()
//              + mSkels[i]->getExternalForces()
//              + mSkels[i]->getConstraintForce(i)
//              + mSkels[i]->getInternalForces());

//    mSkels[i]->clampRotation(mDofs[i], mDofVels[i]);
//    deriv.segment(start, size) = mDofVels[i] + (qddot * mTimeStep); // set velocities
//    deriv.segment(start + size, size) = qddot; // set qddot (accelerations)
//    mDofAccs[i] = qddot;
//  }

//  //     std::cout << mSkels[2]->getExternalForces().transpose() << std::endl;

//  return deriv;
//}

//void MyWindow::setState(const VectorXd& newState) {
//  for (unsigned int i = 0; i < mSkels.size(); i++) {
//    int start = mIndices[i] * 2;
//    int size = mDofs[i].size();
//    mDofs[i] = newState.segment(start, size);
//    mDofVels[i] = newState.segment(start + size, size);
//  }
//}

//==============================================================================
void MyWindow::step()
{
  // TODO(JS): Need to know which joint is correspons 4-th generalized coordinate
//  VectorXd extForce = mSkels[2]->getExternalForces();
//  extForce(4) = extTorque;
//  mSkels[2]->setExternalForces(extForce);

  // Integrate velocity unconstrained skeletons
  for (std::vector<dynamics::Skeleton*>::iterator it = mSkels.begin();
       it != mSkels.end(); ++it)
  {
    if (!(*it)->isMobile())
      continue;

    (*it)->computeForwardDynamicsRecursionPartB();
    (*it)->integrateVelocities(mTimeStep);
    (*it)->computeForwardKinematics(false, true, false);
  }

  // Detect active constraints and compute constraint impulses
  mConstraintSolver->solve();

  // Compute velocity changes given constraint impulses
  for (std::vector<dynamics::Skeleton*>::iterator it = mSkels.begin();
       it != mSkels.end(); ++it)
  {
    if ((*it)->isImpulseApplied() && (*it)->isMobile())
    {
      (*it)->computeImpulseForwardDynamics();
      (*it)->setImpulseApplied(false);
    }
  }

  //
  for (std::vector<dynamics::Skeleton*>::iterator it = mSkels.begin();
       it != mSkels.end(); ++it)
  {
    if (!(*it)->isMobile())
      continue;

    (*it)->integratePositions(mTimeStep);
  }

  for (std::vector<dynamics::Skeleton*>::iterator it = mSkels.begin();
       it != mSkels.end(); ++it)
  {
    if (!(*it)->isMobile())
      continue;

    (*it)->computeForwardDynamicsRecursionPartA();
    (*it)->resetForces();
    (*it)->clearExternalForces();
//    (*it)->clearConstraintImpulses();

  }

  for (size_t i = 0; i < mSkels.size(); ++i)
    mDofAccs[i] = mSkels[i]->getAccelerations();

//  mTime += mTimeStep;
//  mFrame++;
}

//==============================================================================
void MyWindow::setPose()
{
  Skeleton* cubeSkel     = mSkels[2];
  BodyNode* cube         = cubeSkel->getBodyNode(0);
  const Isometry3d cubeT = cube->getTransform();

  //dynamics::BodyNode* wrist = mSkels[1]->getBodyNode(mPalmName);
  int contactEdgeIndex = evalContactEdge();

  if (contactEdgeIndex == mPreContactEdge || contactEdgeIndex == -1)
  {
    //setHandTrans(mPreOri,mEdges[contactEdgeIndex]);
  }
  else if (contactEdgeIndex != -1)
  {
    mPreContactEdge = contactEdgeIndex;
  }

  // If we found contact edge, then set it as previous contact edge
  if (contactEdgeIndex != -1)
    mPreContactEdge = contactEdgeIndex;

  // TODO(JS): Is this for orientation or translation
  mPreOri = mSkels[1]->getPositions().head<3>();

  const bool isLastRoll = mRollNum == mN - 1 ? true : false;
  assert(mRollNum <= mN - 1);

  int index1 = (mRollNum                    ) % mEdges.size();
  int index2 = (mRollNum + mEdges.size() / 2) % mEdges.size();

  if (!isLastRoll)
  {
    if (contactEdgeIndex == mRollNum % mEdges.size())
    {
      // contact edge position in world coordinate
      Vector3d contactPos = cube->getTransform() * mEdges[contactEdgeIndex];

      // if change the roll direction, the condition will be changed
      // accordingly, related to roll direction
      const double& comZ        = cubeSkel->getWorldCOM()[2];
      const double& contactPosZ = contactPos[2];

      const bool& cond1 = comZ - contactPosZ > 0.005 ? true : false;
      const bool& cond2 = comZ - contactPosZ > 0.002 ? true : false;
      const bool& isFirstRoll = mRollNum == 0 ? true : false;

      //
      if ((cond1 && isFirstRoll) || (cond2 && !isFirstRoll))
      {
        const int index1 = (mRollNum + mEdges.size() - 1) % mEdges.size();
        const int index2 = (mRollNum + mEdges.size() + 1) % mEdges.size();
        const int index3 = (mRollNum                    ) % mEdges.size();

        const Vector3d& liftEdge    = cubeT * mEdges[index1];
        const Vector3d& dropEdge    = cubeT * mEdges[index2];
        const Vector3d& contactEdge = cubeT * mEdges[index3];

        const double& liftDistY = liftEdge(1) - contactEdge(1);
        const double& liftDistZ = contactEdge(2) - liftEdge(2);
        const double& dropDistY = dropEdge(1) - contactEdge(1);
        const double& dropDistZ = dropEdge(2) - contactEdge(2);

        const double& liftAngle = atan(liftDistY / liftDistZ);
        const double& dropAngle = atan(dropDistY / dropDistZ);

        const double& nextAngle = mAngles[mRollNum + 1];

        if (liftAngle > nextAngle && dropAngle > -nextAngle)
        {
          mRollNum++;
          setHandAngle(mAngles[mRollNum]);
        }
      }
    }
  }
  else if (isLastRoll && (isEdgeInContact(index1) || evalUpFace() == index2))
  {
    setHandAngle(0.0);
  }

  // calculate up face if know the high corners local coordinates
  if (evalUpFace() != mUpFace)
    mUpFace = evalUpFace();

//  for (unsigned int i = 0; i < mSkels.size(); i++)
//  {
//    mSkels[i]->setPositions(mDofs[i]);
//    mSkels[i]->setVelocities(mDofVels[i]);
//    mSkels[i]->computeForwardKinematics(true, false, false);

//    if (mSkels[i]->isMobile())
//    {
//      // need to update first derivatives for collision
//      mSkels[i]->setGravity(mGravity);
//      mSkels[i]->setPositions(mDofs[i]);
//      mSkels[i]->setVelocities(mDofVels[i]);
//      mSkels[i]->computeForwardKinematics(true, true, false);
//    }
//    else
//    {
//      // need to update node transformation for collision
//      mSkels[i]->setPositions(mDofs[i]);
//      mSkels[i]->computeForwardKinematics(true, false, false);
//    }
//  }
}

//==============================================================================
void MyWindow::resize(int w, int h)
{
#ifdef WIN32
  ui.reshape(w,h);
#endif
  mWinWidth = w;
  mWinHeight = h;

  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  glViewport(0, 0, mWinWidth, mWinHeight);
  gluPerspective(mPersp, (double)mWinWidth/(double)mWinHeight, 0.1,10.0);

  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();

  int smallValue = w<h?w:h;
  mTrackBall.setCenter(Vector2d(w*0.5,h*0.5));
  mTrackBall.setRadius(smallValue/2.5);

  glutPostRedisplay();
}

//==============================================================================
void MyWindow::displayTimer(int _val)
{	
  int numIter = 10;
  if (mPlay)
  {
    mPlayFrame += 20;
    if (mPlayFrame >= mBakedStates.size())
      mPlayFrame = 0;
    glutPostRedisplay();
    glutTimerFunc(mDisplayTimeout, refreshTimer, _val);
  }
  else if (mSim)
  {
    // 		tHandIK.startTimer();
    // TODO(JS): Just commented out
    // updateHandPose();
    // 		tHandIK.stopTimer();
    for (int i = 0; i < numIter; i++)
    {
      // TODO(JS): Just commented out
      // tIter.startTimer();
      {
        //
        setPose();

        //
        // updateHandPose();

        updateContact();

        // tInternal.startTimer();
        {
          // Evaluate all the other hand control force before evaluating object
          // control force, and updateIntForce is inside updateConact, then
          // comment evaluate the hand control force including object control
          // force, the force evaluation is not inside updateContact
          updateIntForce();
        }
        // tInternal.stopTimer();

        // tExternal.startTimer();
        {
          //
          updateExtForce();
        }
        // tExternal.stopTimer();

        // tSimulation.startTimer();
        {
          // mIntegrator.integrate(this, mTimeStep);

          // Step forward dynamics
          step();
        }
        // tSimulation.stopTimer();

        //bake();

        mSimFrame++;
      }
      // tIter.stopTimer();
    }
    mForce.setZero();

    glutPostRedisplay();

    glutTimerFunc(mDisplayTimeout, refreshTimer, _val);

    // tIter.print();
    // tInternal.print();
    // tExternal.print();
    // tSimulation.print();
    // tHandIK.print();
    // tSetPose.print();
    // tContact.print();
    // tJointLimit.print();
  }
}

Vector3d MyWindow::ballToCartesian(const VectorXd& _ball)
{
  Vector3d cartesian;
  cartesian[0] = mBoundEllipsoidR(0)*cos(_ball(0))*cos(_ball(1));
  cartesian[1] = mBoundEllipsoidR(1)*cos(_ball(0))*sin(_ball(1));
  cartesian[2] = mBoundEllipsoidR(2)*sin(_ball(0));
  return cartesian;
}

Vector3d MyWindow::ballToCartesian(const VectorXd& _ball, double _radius)
{
  Vector3d cartesian;
  cartesian[0] = _radius*cos(_ball(0))*cos(_ball(1));
  cartesian[1] = _radius*cos(_ball(0))*sin(_ball(1));
  cartesian[2] = _radius*sin(_ball(0));
  return cartesian;
}

Vector2d MyWindow::cartesianToBall(const Vector3d& _cartesian)
{
  Vector2d ball = Vector2d::Zero();
  ball(0) = asin(_cartesian(2)/mBoundEllipsoidR(2));
  ball(1) = asin(_cartesian(1)/(mBoundEllipsoidR(1)*cos(ball(0))));

  if ((_cartesian(0)-mBoundEllipsoidR(0)*cos(ball(0))*cos(ball(1)))*(_cartesian(0)-mBoundEllipsoidR(0)*cos(ball(0))*cos(ball(1))) < 0.00001) {
    return ball;
  }

  if (ball(1) > 0.0) {
    ball(1) = M_PI - ball(1);
  }
  else if (ball(1) < 0.0) {
    ball(1) = -M_PI - ball(1);
  }
  if ((_cartesian(0)-mBoundEllipsoidR(0)*cos(ball(0))*cos(ball(1)))*(_cartesian(0)-mBoundEllipsoidR(0)*cos(ball(0))*cos(ball(1))) < 0.00001) {
    return ball;
  }

  if (ball(0) > 0.0) {
    ball(0) = M_PI - ball(0);
  }
  else if (ball(0) < 0.0) {
    ball(0) = -M_PI - ball(0);
  }
  ball(1) = asin(_cartesian(1)/(mBoundEllipsoidR(1)*cos(ball(0))));
  if ((_cartesian(0)-mBoundEllipsoidR(0)*cos(ball(0))*cos(ball(1)))*(_cartesian(0)-mBoundEllipsoidR(0)*cos(ball(0))*cos(ball(1))) < 0.00001) {
    return ball;
  }

  if (ball(1) > 0.0) {
    ball(1) = M_PI - ball(1);
  }
  else if (ball(1) < 0.0) {
    ball(1) = -M_PI - ball(1);
  }
  if ((_cartesian(0)-mBoundEllipsoidR(0)*cos(ball(0))*cos(ball(1)))*(_cartesian(0)-mBoundEllipsoidR(0)*cos(ball(0))*cos(ball(1))) < 0.00001) {
    return ball;
  }

  return ball;
}

Vector2d MyWindow::cartesianToBall(Vector3d _cartesian, double _radius)
{
  Vector2d ball = Vector2d::Zero();

  if (_cartesian(2)/_radius < 1.0 && _cartesian(2)/_radius > -1.0) {
    ball(0) = asin(_cartesian(2)/_radius);
  }
  else if (_cartesian(2)/_radius > 0.99) {
    ball(0) = asin(0.99);
  }
  else if (_cartesian(2)/_radius < -0.99) {
    ball(0) = asin(-0.99);
  }

  if (_cartesian(1)/(_radius*cos(ball(0))) < 1.0 && _cartesian(1)/(_radius*cos(ball(0))) > -1.0) {
    ball(1) = asin(_cartesian(1)/(_radius*cos(ball(0))));
  }
  else if (_cartesian(1)/(_radius*cos(ball(0))) > 0.99) {
    ball(1) = asin(0.99);
  }
  else if (_cartesian(1)/(_radius*cos(ball(0))) < -0.99) {
    ball(1) = asin(-0.99);
  }

  if ((_cartesian(0)-_radius*cos(ball(0))*cos(ball(1)))*(_cartesian(0)-_radius*cos(ball(0))*cos(ball(1))) < 0.00001) {
    return ball;
  }

  if (ball(1) > 0.0) {
    ball(1) = M_PI - ball(1);
  }
  else if (ball(1) < 0.0) {
    ball(1) = -M_PI - ball(1);
  }
  if ((_cartesian(0)-_radius*cos(ball(0))*cos(ball(1)))*(_cartesian(0)-_radius*cos(ball(0))*cos(ball(1))) < 0.00001) {
    return ball;
  }

  if (ball(0) > 0.0) {
    ball(0) = M_PI - ball(0);
  }
  else if (ball(0) < 0.0) {
    ball(0) = -M_PI - ball(0);
  }
  ball(1) = asin(_cartesian(1)/(_radius*cos(ball(0))));
  if ((_cartesian(0)-_radius*cos(ball(0))*cos(ball(1)))*(_cartesian(0)-_radius*cos(ball(0))*cos(ball(1))) < 0.00001) {
    return ball;
  }

  if (ball(1) > 0.0) {
    ball(1) = M_PI - ball(1);
  }
  else if (ball(1) < 0.0) {
    ball(1) = -M_PI - ball(1);
  }
  if ((_cartesian(0)-_radius*cos(ball(0))*cos(ball(1)))*(_cartesian(0)-_radius*cos(ball(0))*cos(ball(1))) < 0.00001) {
    return ball;
  }

  return ball;
}

Vector3d MyWindow::objToEllipsoid(Vector3d _obj)
{
  double t = 0.0;
  t = pow(1/((_obj(0)*_obj(0)/(mBoundEllipsoidR(0)*mBoundEllipsoidR(0)))+(_obj(1)*_obj(1)/(mBoundEllipsoidR(1)*mBoundEllipsoidR(1)))+(_obj(2)*_obj(2)/(mBoundEllipsoidR(2)*mBoundEllipsoidR(2)))),0.5);
  if (t < 0.0) {
    t = -t;
  }
  return t*_obj;
}

void MyWindow::draw()
{
  glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

  mSkels[0]->draw(mRI);

  if (mDrawModel)
    mSkels[2]->draw(mRI);

  if (!mSim)
  {
    if (mPlayFrame < mBakedStates.size())
    {
      for (unsigned int i = 0; i < mSkels.size(); i++)
      {
        int start = mIndices[i];
        int size = mDofs[i].size();

        // TODO(JS):
        //        mSkels[1]->getJoint(0)->setPosition(tratrmRootTrans[mPlayFrame](0));
        //        mSkels[1]->getJoint(0)->getTransform(0)->getDof(1)->setValue(mRootTrans[mPlayFrame](1));
        //        mSkels[1]->getJoint(0)->getTransform(0)->getDof(2)->setValue(mRootTrans[mPlayFrame](2));
        //        mSkels[1]->getJoint(0)->updateStaticTransform();

        mSkels[i]->setPositions(mBakedStates[mPlayFrame].segment(start, size));
        mSkels[i]->computeForwardKinematics(true, false, false);
      }

      int sumDofs = mIndices[mSkels.size()];
      int nContact = (mBakedStates[mPlayFrame].size() - sumDofs) / 6;
      for (int i = 0; i < nContact; i++)
      {
        Vector3d v = mBakedStates[mPlayFrame].segment(sumDofs + i * 6, 3);
        Vector3d f = mBakedStates[mPlayFrame].segment(sumDofs + i * 6 + 3, 3) / 10.0;

        // draw contact force
        glBegin(GL_LINES);
        mRI->setPenColor(Vector3d(0.2, 0.8, 0.2));
        glVertex3f(v[0], v[1], v[2]);
        glVertex3f(v[0] - f[0], v[1] - f[1], v[2] - f[2]);
        glEnd();

        // draw in contact point and break contact point
        mRI->pushMatrix();
        glTranslated(v[0], v[1], v[2]);
        if (f.norm() > 0.0001)
          mRI->setPenColor(Vector3d(0.8, 0.2, 0.2));
        else
          mRI->setPenColor(Vector3d(0.2, 0.8, 0.2));
        mRI->drawEllipsoid(Vector3d(0.002, 0.002, 0.002));
        mRI->popMatrix();
      }
    }
  }
  else
  {
    for (int k = 0;
         k < mConstraintSolver->getCollisionDetector()->getNumContacts(); k++)
    {
      Vector3d v = mConstraintSolver->getCollisionDetector()->getContact(k).point;
      Vector3d n = mConstraintSolver->getCollisionDetector()->getContact(k).normal / 10.0;
      Vector3d f = mConstraintSolver->getCollisionDetector()->getContact(k).force / 100.0;
      /*
      mRI->setPenColor(Vector3d(0.2, 0.2, 0.8));
      glBegin(GL_LINES);
      glVertex3f(v[0], v[1], v[2]);
      glVertex3f(v[0] + f[0], v[1] + f[1], v[2] + f[2]);
      glEnd();
      mRI->setPenColor(Vector3d(0.2, 0.8, 0.2));
      mRI->pushMatrix();
      glTranslated(v[0], v[1], v[2]);
      mRI->drawEllipsoid(Vector3d(0.002, 0.002, 0.002));
      mRI->popMatrix();
      */
    }
  }


  if (mDrawBoundEllipsoid) {
    for (int i = 0; i < mFingerNum; ++i) {
      // TODO(JS): Just commented out
//      if (mController->mActiveFingers[i]) {
//        Vector2d startPointB = mContactPointsBallStart[i];
//        Vector2d endPointB = mContactPointsBallEnd[i];
//        Vector2d middlePointB;

//        Vector3d startPointC = ballToCartesian(startPointB);
//        Vector3d endPointC = ballToCartesian(endPointB);
//        Vector3d middlePointC;

//        Vector3d startPointCW
//            = mSkels[2]->getBodyNode(0)->getTransform() * startPointC;
//        Vector3d endPointCW
//            = mSkels[2]->getBodyNode(0)->getTransform() * endPointC;
//        Vector3d middlePointCW;

//        // draw the bounding ball
//        /*
//        mRI->setPenColor(Vector3d(0.0, 0.0, 1.0));
//        mRI->pushMatrix();
//        glTranslated(mDofs[2][0], mDofs[2][1], mDofs[2][2]);
//        glScaled(mBoundEllipsoidR(0),mBoundEllipsoidR(1),mBoundEllipsoidR(2));
//        glutSolidSphere(1.0,64,64);
//        mRI->popMatrix();
//        */

//        mRI->setPenColor(Vector3d(1.0, 0.0, 0.0));
//        mRI->pushMatrix();
//        glTranslated(startPointCW(0), startPointCW(1), startPointCW(2));
//        mRI->drawEllipsoid(Vector3d(0.01, 0.01, 0.01));
//        mRI->popMatrix();

//        // draw the end point
//        /*
//        mRI->setPenColor(Vector3d(1.0, 1.0, 0.0));
//        mRI->pushMatrix();
//        glTranslated(endPointCW(0), endPointCW(1), endPointCW(2));
//        mRI->drawEllipsoid(Vector3d(0.01, 0.01, 0.01));
//        mRI->popMatrix();
//        */

//        for (int j = 1; j < 10; ++j) {
//          middlePointCW = evalFingerTipTraj(startPointB,endPointB,j,10);
//          mRI->setPenColor(Vector3d(0.0, 1.0, 0.0));
//          mRI->pushMatrix();
//          glTranslated(middlePointCW(0), middlePointCW(1), middlePointCW(2));
//          mRI->drawEllipsoid(Vector3d(0.01, 0.01, 0.01));
//          mRI->popMatrix();
//        }
//      }
    }
  }

  mSkels[1]->draw(mRI);

  if (mDrawIK)
  {
    // draw IK result
    if (mSim)
    {
      VectorXd currentPose = mSkels[1]->getPositions();
      // TODO(JS): Just commented out
//      mSkels[1]->setPositions(mController->mFingerEndPose);
      mSkels[1]->computeForwardKinematics(true, true, false);
      mSkels[1]->draw(mRI, Vector4d(1.0, 0.0, 1.0, 1.0), false);
      mSkels[1]->setPositions(currentPose);
      mSkels[1]->computeForwardKinematics(true, true, false);
    }
  }

  glDisable(GL_LIGHTING);
#ifdef WIN32
  doUI();
#endif
  glEnable(GL_LIGHTING);

  // display the frame count in 2D text
  glDisable(GL_LIGHTING);
  char buff[64];
  if (!mSim)
    sprintf(buff, "%d", mPlayFrame);
  else
    sprintf(buff, "%d", mSimFrame+10);
  string frame(buff);
  glColor3f(0.0, 0.0, 0.0);
  gui::drawStringOnScreen(0.02f, 0.02f, frame);
  glEnable(GL_LIGHTING);

  // screen shot
  int fr = mSimFrame;
  string scrShotName;
  stringstream convert;
  convert << fr;

  scrShotName = mFileName;
  if (fr < 10) {
    scrShotName += "000" + convert.str() + ".tga";
  }
  else if (fr > 9 && fr < 100) {
    scrShotName += "00" + convert.str() + ".tga";
  }
  else if (fr > 99 && fr < 1000) {
    scrShotName += "0" + convert.str() + ".tga";
  }
  else {
    scrShotName += convert.str() + ".tga";
  }
  if (mSim && mSimFrame%50 == 0) {
    //yui::screenShot(640,480,scrShotName.c_str(),false);
  }

}

void MyWindow::keyboard(unsigned char key, int x, int y)
{
  VectorXd pose = mSkels[1]->getPositions();
  VectorXd objPose = mSkels[2]->getPositions();
  VectorXd extForce = mSkels[2]->getExternalForces();
  switch(key)
  {
    case ' ': // use space key to play or stop the motion
      mSim = !mSim;
      if (mSim)
      {
        mPlay = false;
        glutTimerFunc( mDisplayTimeout, refreshTimer, 0);
      }
      break;
    case 's':
      saveBake();
      break;
    case 'l':
      loadBake();
      break;
    case 'h':
      saveHandInitPose();
      break;
    case 'p': // playBack
      mPlay = !mPlay;
      if (mPlay)
      {
        mSim = false;
        glutTimerFunc(mDisplayTimeout, refreshTimer, 0);
      }
      break;
    case '[': // step backward
      if (!mSim)
      {
        mPlayFrame--;
        if(mPlayFrame < 0)
          mPlayFrame = 0;
        glutPostRedisplay();
      }
      break;
    case ']': // step forwardward
      if (!mSim)
      {
        mPlayFrame++;
        if(mPlayFrame >= mBakedStates.size())
          mPlayFrame = 0;
        glutPostRedisplay();
      }
      break;
    case 'v': // show or hide markers
      mShowMarkers = !mShowMarkers;
      break;
    case 'i':
      updateContactPoint();
      for (int i = 0; i < mFingerNum; ++i)
      { // if the initial configuration is all the fingers touch the object
        mContactPointsBallStart[i]
            = cartesianToBall(
                mSkels[2]->getBodyNode(0)->getTransform().inverse()
                * mFingerContactPoints[i],
                mObjRadius);
      }
      updateHandPose();
      // TODO(JS): Just commented out
//      mSkels[1]->setPositions(mController->mFingerEndPose);
      mSkels[1]->computeForwardKinematics(true, false, false);
      // TODO(JS): Just commented out
//      mDofs[1] = mController->mFingerEndPose;
      break;
    case 'o':
      mDrawOri = !mDrawOri;
      glutPostRedisplay();
      break;
    case 'k':
      mDrawIK = !mDrawIK;
      glutPostRedisplay();
      break;
    case 'm':
      mDrawModel = !mDrawModel;
      glutPostRedisplay();
      break;
    case 't':
      mDrawTarget = !mDrawTarget;
      glutPostRedisplay();
      break;
    case 'b':
      mDrawBoundEllipsoid = !mDrawBoundEllipsoid;
      glutPostRedisplay();
      break;
    case 'f':
      mDrawContactForce = !mDrawContactForce;
      glutPostRedisplay();
      break;
    case 'n': // choose next DoF
      if (mActiveDofIndex == mDofNum-1) {
        mActiveDofIndex = 0;
      }
      else {
        mActiveDofIndex++;
      }
      std::cout << "DoF " << mActiveDofIndex << std::endl;
      break;
    case '=': // increase DoF value
      pose(mActiveDofIndex) = pose(mActiveDofIndex) + 0.1;
      mDofs[1][mActiveDofIndex] = mDofs[1][mActiveDofIndex] + 0.1;
      mSkels[1]->setPositions(pose);
      mSkels[1]->computeForwardKinematics(true, true, false);
      std::cout << "value: " << pose(mActiveDofIndex) << std::endl;
      break;
    case '-': // decrease DoF value
      pose(mActiveDofIndex) = pose(mActiveDofIndex) - 0.1;
      mDofs[1][mActiveDofIndex] = pose(mActiveDofIndex) - 0.1;
      mSkels[1]->setPositions(pose);
      mSkels[1]->computeForwardKinematics(true, true, false);
      std::cout << "value: " << pose(mActiveDofIndex) << std::endl;
      break;
    case 'N': // choose next DoF of the object
      if (mObjActiveDofIndex == mObjDofNum-1) {
        mObjActiveDofIndex = 0;
      }
      else {
        mObjActiveDofIndex++;
      }
      std::cout << "DoF " << mObjActiveDofIndex << std::endl;
      break;
    case '+': // increase DoF value of the object
      objPose(mObjActiveDofIndex) = objPose(mObjActiveDofIndex) + 0.01;
      mDofs[2][mObjActiveDofIndex] = mDofs[2][mObjActiveDofIndex] + 0.01;
      mSkels[2]->setPositions(objPose);
      mSkels[2]->computeForwardKinematics(true, true, false);
      std::cout << "value: " << objPose(mObjActiveDofIndex) << std::endl;
      break;
    case '_': // decrease DoF value of the object
      objPose(mObjActiveDofIndex) = objPose(mObjActiveDofIndex) - 0.01;
      mDofs[2][mObjActiveDofIndex] = mDofs[2][mObjActiveDofIndex] - 0.01;
      mSkels[2]->setPositions(objPose);
      mSkels[2]->computeForwardKinematics(true, true, false);
      std::cout << "value: " << objPose(mObjActiveDofIndex) << std::endl;
      break;
    case 'x':
      /*
    extForce(4) = extForce(4) + 0.1;
    mSkels[2]->setExternalForces(extForce);
    */
      extTorque = extTorque+0.1;
      break;
    case 'z':
      /*
    extForce(4) = extForce(4) - 0.1;
    mSkels[2]->setExternalForces(extForce);
    */
      extTorque = extTorque-0.1;
      break;
    default:
      Win3D::keyboard(key,x,y);
  }


  glutPostRedisplay();
}

void MyWindow::click(int button, int state, int x, int y)
{
  mMouseDown = !mMouseDown;
  int mask = glutGetModifiers();
  if (mMouseDown)	{
    if(mMouseDown && mask == GLUT_ACTIVE_SHIFT){
      mZooming = true;
    }
    else if (mMouseDown && mask == GLUT_ACTIVE_CTRL) {
      mRotate = true;
      mTrackBall.startBall(x,mWinHeight-y);
    }
    else if (button == GLUT_RIGHT_BUTTON) {
      mTranslate = true;
    }
    mMouseX = x;
    mMouseY = y;
  }
  else {
    mTranslate = false;
    mRotate = false;
    mZooming = false;
  }
#ifdef WIN32
  ui.mouse(button,state,x,y);
#endif
  glutPostRedisplay();
}

void MyWindow::drag(int x, int y)
{
  double deltaX = x - mMouseX;
  double deltaY = y - mMouseY;

  mMouseX = x;
  mMouseY = y;

  if(mRotate){
    if(deltaX!=0 || deltaY!=0)
      mTrackBall.updateBall(x,mWinHeight-y);
  }
  if(mTranslate){
    Matrix3d rot = mTrackBall.getRotationMatrix();
    mTrans += rot.transpose()*Vector3d(deltaX, -deltaY, 0.0);
  }
  if(mZooming){
    mZoom += deltaY*0.01;
  }
#ifdef WIN32
  ui.mouseMotion(x,y);
#endif
  glutPostRedisplay();
}

void MyWindow::bake()
{
  int nContact = mConstraintSolver->getCollisionDetector()->getNumContacts();
  VectorXd state(mIndices.back() + 6 * nContact);
  for (unsigned int i = 0; i < mSkels.size(); i++)
    state.segment(mIndices[i], mDofs[i].size()) = mDofs[i];
  for (int i = 0; i < nContact; i++)
  {
    int begin = mIndices.back() + i * 6;
    state.segment(begin, 3) = mConstraintSolver->getCollisionDetector()->getContact(i).point;
    state.segment(begin + 3, 3) = mConstraintSolver->getCollisionDetector()->getContact(i).force;
  }
  mBakedStates.push_back(state);
  Vector3d rootTrans;
  // TODO(JS):
//  rootTrans(0) = mSkels[1]->getJoint(0)->getTransform(0)->getDof(0)->getValue();
//  rootTrans(1) = mSkels[1]->getJoint(0)->getTransform(0)->getDof(1)->getValue();
//  rootTrans(2) = mSkels[1]->getJoint(0)->getTransform(0)->getDof(2)->getValue();
  mRootTrans.push_back(rootTrans);
}

void MyWindow::saveBake()
{
  string fileName = "rotate_ODE.txt";
  ofstream outFile(fileName.c_str());
  outFile.precision(10);
  outFile << mBakedStates.size() << std::endl;
  for (int i = 0; i < mBakedStates.size(); ++i) {
    outFile << mBakedStates[i].size() << "  ";
    outFile << mBakedStates[i].transpose() << std::endl;
  }

  std::cout << "save finished" << std::endl;
}

bool MyWindow::loadBake()
{
  mBakedStates.clear();

  string fileName = mBakeFileName;
  ifstream inFile(fileName.c_str());
  if (inFile.fail()) {
    return false;
  }
  inFile.precision(10);
  string buffer;
  inFile >> buffer;
  int nFrame = atoi(buffer.c_str());
  mBakedStates.resize(nFrame);

  for (int i = 0; i < nFrame; ++i) {
    inFile >> buffer;
    int stateSize = atoi(buffer.c_str());
    mBakedStates[i] = VectorXd::Zero(stateSize);
    for (int j = 0; j < stateSize; ++j) {
      inFile >> buffer;
      mBakedStates[i][j] = atof(buffer.c_str());
    }
  }

  return true;
}

// this function should assign tasks for controller, based on high level requirement, the trajectory of object, and actual motion of object
void MyWindow::updateTaskArray()
{

}

Vector3d MyWindow::evalFingerTipTraj(VectorXd _start, VectorXd _end, int _curFrame, int _totalFrame)
{
  Vector2d startPointB = _start;
  Vector2d endPointB = _end;
  Vector2d middlePointB;

  Vector3d middlePointC;
  Vector3d middlePointCW;

  double i = (double)(_curFrame)/(double)(_totalFrame);
  middlePointB = (endPointB-startPointB)*i+startPointB;

  for (int i = 0; i < 2; ++i) {
    if (endPointB(i)-startPointB(i) < -2.0) {
      middlePointB = endPointB;
    }
    else if (endPointB(i)-startPointB(i) > 2.0) {
      middlePointB = endPointB;
    }
  }

  middlePointC = ballToCartesian(middlePointB);
  middlePointCW = mSkels[2]->getBodyNode(0)->getTransform() * middlePointC;

  return Vector3d(middlePointCW(0),middlePointCW(1),middlePointCW(2));
}

// this function should assign reference hand pose based on task and IK, can utilize the bounding sphere to avoid collision, and can be called on the fly as update task array 
void MyWindow::updateHandPose()
{
  //	VectorXd currentPose;
  //	currentPose = mSkels[1]->getPose();
  //	// solve IK
  //	IKProblem prob(mSkels[1],mFingerNames);
  //	snopt::SnoptSolver solver(&prob);

  //	// set the position target for fingers, if the finger is not related to task, we can set the current position of the finger as the target, otherwise, we can use the contact point or the tip trajectory
  //	PositionConstraint* p;
  //	OrientationConstraint* ori;
  //	Vector3d target = Vector3d::Zero();
  //	for (int i = 0; i < mFingerNum; ++i) {
  //		p = prob.getConstraint(i);
  //		target = mSkels[2]->getBodyNode(0)->getTransform() * mTargetContactPoints[i]);
  //		p->setTarget(target);


  //		if (mController->mActiveSimFrame[i] < mController->mActiveNumFrame[i] && mController->mActiveFingers[i]) {
  //			mController->mActiveSimFrame[i] += 3;
  //		}


  //		ori = prob.getOriConstraint(i);
  //		Vector3d orientationVector = Vector3d(0.0,0.0,-1.0);
  //		target = orientationVector;
  //		ori->setTarget(target);
  //	}


  //	solver.solve();
  //	for (int i = 0; i < mSkels[1]->getNumDofs(); ++i) {
  //		mController->mFingerEndPose[i] = prob.vars()[i]->mVal;
  //	}

  //	mSkels[1]->setPose(currentPose, true, true);

}

// This function should assign the contact points and contact forces, based on
// the contact point from collision detection, and do contact planning
void MyWindow::updateContact()
{
  //mConstraintSolver->evaluateConstraint();
  mConstraintSolver->solve();

  mContactBodyNames.clear();
  mContactPoints.clear();
  mTargetContactForces.clear();

  for (int i = 0; i < mFingerNum; ++i)
  {
    mFingerTargetContactForces[i] = Vector3d::Zero();
    mFingerContactPoints[i] = Vector3d::Zero();
  }

  Isometry3d transformMatrix = mSkels[2]->getBodyNode(0)->getTransform();
  Matrix3d rotationMatrix = transformMatrix.linear();

  std::vector<std::vector<Vector3d> > contactPointsOnHand;
  std::vector<std::vector<Vector3d> > contactNormalsOnHand;
  std::vector<std::vector<int> > contactIndicesOnHand;
  std::vector<int> numContact;
  std::vector<double> avePenetration;
  std::vector<int> contactEraseTotal;
  std::vector<int> contactEraseCount;
  std::vector<int> contactLeastKeep;
  contactPointsOnHand.resize(mFingerNum);
  contactNormalsOnHand.resize(mFingerNum);
  contactIndicesOnHand.resize(mFingerNum);
  numContact.resize(mFingerNum);
  avePenetration.resize(mFingerNum);
  contactEraseTotal.resize(mFingerNum);
  contactEraseCount.resize(mFingerNum);
  contactLeastKeep.resize(mFingerNum);

  for (int i = 0; i < mFingerNum; ++i)
  {
    numContact[i] = 0;
    avePenetration[i] = 0.0;
    contactEraseTotal[i] = 20;
    contactEraseCount[i] = 0;
    contactLeastKeep[i] = 4;
  }

  // calculate the contact number for each finger
  collision::CollisionDetector* cd = mConstraintSolver->getCollisionDetector();
  for (size_t i = 0; i < cd->getNumContacts(); ++i)
  {
    for (int j = 0; j < mFingerNum; ++j)
    {
      string bodyNodeName1 = cd->getContact(i).bodyNode1->getName();

      if (bodyNodeName1 == mFingerTipNames[j])
      {
        numContact[j]++;
        break;
      }
    }
  }

  // only keep part of contacts for each finger
  // TODO(JS): Just commented out
//  for (int i = cd->getNumContacts() - 1; i >= 0; --i)
//  {
//    for (int j = 0; j < mFingerNum; ++j)
//    {
//      if (cd->getContact(i).bodyNode1->getName() == mFingerNames[j])
//      {
//        if (contactEraseCount[j] < contactEraseTotal[j]
//            && contactEraseCount[j] < numContact[j] - contactLeastKeep[j])
//        {
//          cd->mContacts.erase(cd->mContacts.begin() + i);
//          contactEraseCount[j]++;
//          break;
//        }
//      }
//    }
//  }

  for (int i = 0; i < mFingerNum; ++i)
    numContact[i] = 0;

  // 	std::cout << "#contact:  " << mConstraintHandle->getNumContacts() << std::endl;

  // The original contact information
  for (int i = 0; i < cd->getNumContacts(); ++i)
  {
    Contact contact = cd->getContact(i);

    for (int j = 0; j < mFingerNum; ++j)
    {
      if (contact.bodyNode1->getName() == mFingerTipNames[j])
      {
        numContact[j]++;
        contactPointsOnHand[j].push_back(contact.point);
        contactNormalsOnHand[j].push_back(contact.normal);
        contactIndicesOnHand[j].push_back(i);
        avePenetration[j] += contact.penetrationDepth;
        break;
      }
    }
  }

  // Average the contact point and penetration
  for (int i = 0; i < mFingerNum; ++i)
  {
    for (int j = 0; j < numContact[i]; ++j)
    {
      mFingerContactPoints[i] = mFingerContactPoints[i]
                                + contactPointsOnHand[i][j];
    }

    if (numContact[i] > 0)
    {
      mFingerContactPoints[i] = mFingerContactPoints[i]/numContact[i];
      avePenetration[i] = avePenetration[i]/numContact[i];
    }
  }

  // update finger state
  Vector3d dirToThumb(-1.0,0.0,0.0);
  Vector3d dirToLF(1.0,0.0,0.0);
  double deviateAngleToThumb = acos(mObjOri.dot(dirToThumb)/(mObjOri.norm()*dirToThumb.norm()));
  double deviateAngleToLF = acos(mObjOri.dot(dirToLF)/(mObjOri.norm()*dirToLF.norm()));

  if (abs(deviateAngleToLF-1.57) < 0.01) {
    // TODO(JS): Just commented out
//    for (int i = 0; i < mFingerNum; ++i) {
//      mController->mRestFingers[i] = true;
//      mController->mActiveFingers[i] = false;
//      mController->mContactFingers[i] = false;
//      mController->mInContactFingers[i] = false;
//      mController->mActiveSimFrame[i] = 0;
//    }
  }
  // set the contact finger from acitve finger
  for (int i = 0; i < mFingerNum; ++i) {
    // TODO(JS): Just commented out
//    if (mController->mActiveSimFrame[i] >=  mController->mActiveNumFrame[i]) {
//      // find the moment the finger changes from active to contact
//      if (mController->mActiveFingers[i] == true) {
//        mController->mTrackFingers[i] = true;
//      }
//      mController->mActiveFingers[i] = false;
//      mController->mContactFingers[i] = true;
//      mController->mActiveSimFrame[i] = mController->mActiveNumFrame[i];
//      mController->mTrackSimFrame[i]++;
//    }
  }

  // set the in contact fingers
  for (int i = 0; i < mFingerNum; ++i) {
    // TODO(JS): Just commented out
//    if (mController->mContactFingers[i] && numContact[i] > 0) {
//      mController->mInContactFingers[i] = true;
//      mController->mInContactFrame[i]++;
//      mController->mTrackFingers[i] = false;
//      mController->mControlFlag = true;
//      mController->mTrackSimFrame[i] = 0;
//    }
//    else if (mController->mContactFingers[i] && mController->mTrackSimFrame[i] >= mController->mTrackNumFrame[i]) {
//      mController->mTrackFingers[i] = false;
//      mController->mInContactFingers[i] = false;
//      mController->mControlFlag = true;
//      mController->mTrackSimFrame[i] = 0;

//    }
//    else {
//      mController->mInContactFingers[i] = false;
//    }
  }

  // update the orientation of the object
  dynamics::BodyNode* object = mSkels[2]->getBodyNode(0);
  Isometry3d objTransformMatrix = object->getTransform();
  Matrix3d objRotationMatrix = objTransformMatrix.linear();
  mObjOri = objRotationMatrix*Vector3d(0.0,1.0,0.0);

  // update the orientation of the palm
  dynamics::BodyNode* wrist = mSkels[1]->getBodyNode(mPalmName.c_str());
  Isometry3d palmTransformMatrix = wrist->getTransform();
  Matrix3d palmRotationMatrix = palmTransformMatrix.linear();
  Vector3d orientationVector = palmRotationMatrix*Vector3d(0.0,-1.0,0.0);

  // update the angle between the object and the palm
  double angle = acos(orientationVector.dot(mObjOri)/(orientationVector.norm()*mObjOri.norm()));
  if (abs(angle - mPalmObjAngle) < 0.001)
  {
    // TODO(JS): Just commented out
//    mController->mOriSimFrame++;

//    if (mController->mOriSimFrame > mController->mOriNumFrame && mController->mOriFlag == true)
//    { // the relative orientation between palm and object does not change
//      mController->mOriFlag = false;
//      mController->mOriSimFrame = 0;
//    }

  }
  mPalmObjAngle = angle;

  //updateContactPoint();

  // set start point in ball coordinate
  for (int i = 0; i < mFingerNum; ++i)
  {
    if (numContact[i] > 0)
    {
      mContactPointsBallStart[i] = cartesianToBall(mSkels[2]->getBodyNode(0)->getTransform().inverse() * mFingerContactPoints[i], mObjRadius);
    }
    else { // if the finger has no contact with object, then use the target contact position as start position
      mContactPointsBallStart[i] = mContactPointsBallEnd[i];
    }
  }


  // solve the contact force
  // the normal information is based on the object
  std::vector<Vector3d> normals;
  Vector3d normalVector;
  for (int i = 0; i < mFingerNum; ++i)
  {
    // TODO(JS): Just commented out
//    if (mController->mInContactFingers[i])
//    {
//      normalVector = -(mSkels[2]->getBodyNode(0)->getTransform().inverse() * mFingerContactPoints[i]);
//      normalVector = normalVector.normalized();
//      normals.push_back(rotationMatrix*normalVector);
//      mContactNormals[i] = rotationMatrix*normalVector;
//      mContactPoints.push_back(mFingerContactPoints[i]);
//      mContactBodyNames.push_back(mFingerNames[i]);
//    }
  }

  // the force and torque should be based on the hand frame
  VectorXd forceTorqueConstraint = VectorXd::Zero(6);
  forceTorqueConstraint = updateForceTorqueConstraint();
  Vector3d totalForce = forceTorqueConstraint.head(3);
  totalForce = totalForce;
  Vector3d totalTorque = forceTorqueConstraint.tail(3);
  Vector3d objCOM = mSkels[2]->getWorldCOM();

  std::vector<bool> fingerFlag;
  fingerFlag.resize(mFingerNum);
  for (int i = 0; i < mFingerNum; ++i) {
    fingerFlag[i] = true;
  }

  // have no feedback for contact force
  int kp = 0.0;

  // TODO(JS): Just commented out
  // solve the desired contact forces using approximated contact points
//  if (mContactPoints.size() > 0)
//  {
//    ContactForceProblem prob(mContactPoints, normals, totalForce, totalTorque, objCOM);
//    snopt::SnoptSolver solver(&prob);
//    solver.solve();

//    VectorXd z = solver.getState();

//    for (int i = 0; i < mContactPoints.size(); ++i) {
//      for (int j = 0; j < mFingerNum; ++j) {
//        if (fingerFlag[j] && mController->mInContactFingers[j]) {
//          fingerFlag[j] = false;
//          mTargetContactForces.push_back(z.segment(i*3,3));
//          std::cout << "ID " << j << " contact force:" << std::endl << mTargetContactForces[i].transpose() << std::endl;
//          break;
//        }
//      }
//    }
//  }
}

void MyWindow::updateTaskTarget()
{
  // TODO(JS): Just commented out
//  // update task target example
//  // state of hand
//  VectorXd state(mDofs[1].size()+mDofVels[1].size());
//  state.head(mDofs[1].size()) = mDofs[1];
//  state.tail(mDofVels[1].size()) = mDofVels[1];

//  // other force
//  VectorXd otherForce = VectorXd::Zero(mSkels[1]->getNumDofs());
//  otherForce = mSkels[1]->getExternalForces();
//  otherForce += mController->mGravityCompensationForce+mController->mObjControlForce+mController->mTrackForce+mController->mDampForce/*+mController->mOriForce+mController->mMaintainForce*/;
//  for (int i = 0; i < mTasks.size(); ++i)
//  {
//    if (mTasks[i]->mTaskType == tasks::MAINTAIN)
//    {
//      dynamic_cast<tasks::MaintainTask*>(mTasks[i])->updateTask(state,Vector3d(0.0,0.0,0.0),otherForce);
//    }
//    else if (mTasks[i]->mTaskType == tasks::ORI)
//    {
//      dynamic_cast<tasks::TrackOriTask*>(mTasks[i])->updateTask(state,Vector3d(angleX,angleY,angleZ).normalized(),otherForce);
//    }
//    else if (mTasks[i]->mTaskType == tasks::EE)
//    {
//      Vector3d curPos;
//      int endEffector = dynamic_cast<tasks::TrackEETask*>(mTasks[i])->getEEIndex();
//      curPos = mSkels[1]->getBodyNode(dynamic_cast<tasks::TrackEETask*>(mTasks[i])->getEEIndex())->getWorldCOM();
//      if (i == 0)
//        curPos += dynamic_cast<tasks::TrackEETask*>(mTasks[i])->getEETraj()->getEndPoint() - mSkels[2]->getBodyNode(0)->getTransform() * mTargetContactPoints[0];
//      else if (i == 1)
//        curPos += dynamic_cast<tasks::TrackEETask*>(mTasks[i])->getEETraj()->getEndPoint() - mSkels[2]->getBodyNode(0)->getTransform() * mTargetContactPoints[1];
//      else if (i == 2)
//        curPos += dynamic_cast<tasks::TrackEETask*>(mTasks[i])->getEETraj()->getEndPoint() - mSkels[2]->getBodyNode(0)->getTransform() * mTargetContactPoints[2];

//      Vector3d curTarget;
//      if (i == 0)
//        curTarget = dynamic_cast<tasks::TrackEETask*>(mTasks[i])->getEETraj()->evalNextTarget(curPos) - mSkels[2]->getBodyNode(0)->getTransform() * mTargetContactPoints[0];
//      else if (i == 1)
//        curTarget = dynamic_cast<tasks::TrackEETask*>(mTasks[i])->getEETraj()->evalNextTarget(curPos) - mSkels[2]->getBodyNode(0)->getTransform() * mTargetContactPoints[1];
//      else if (i == 2)
//        curTarget = dynamic_cast<tasks::TrackEETask*>(mTasks[i])->getEETraj()->evalNextTarget(curPos) - mSkels[2]->getBodyNode(0)->getTransform() * mTargetContactPoints[2];

//      Vector3d curTargetVel;
//      curTargetVel = dynamic_cast<tasks::TrackEETask*>(mTasks[i])->getEETraj()->evalNextTargetVel(curPos);
//      dynamic_cast<tasks::TrackEETask*>(mTasks[i])->setTargetVel(curTargetVel);
//      if (i == 0)
//        dynamic_cast<tasks::TrackEETask*>(mTasks[i])->setFinalTarget(mSkels[2]->getBodyNode(0)->getTransform() * mTargetContactPoints[0]);
//      else if (i == 1)
//        dynamic_cast<tasks::TrackEETask*>(mTasks[i])->setFinalTarget(mSkels[2]->getBodyNode(0)->getTransform() * mTargetContactPoints[1]);
//      else if (i == 2)
//        dynamic_cast<tasks::TrackEETask*>(mTasks[i])->setFinalTarget(mSkels[2]->getBodyNode(0)->getTransform() * mTargetContactPoints[2]);

//      // not use trajectory
//      if (i == 0)
//        curTarget = mSkels[2]->getBodyNode(0)->getTransform() * mTargetContactPoints[0];
//      else if (i == 1)
//        curTarget = mSkels[2]->getBodyNode(0)->getTransform() * mTargetContactPoints[1];
//      else if (i == 2)
//        curTarget = mSkels[2]->getBodyNode(0)->getTransform() * mTargetContactPoints[2];
//      else if (i == 3)
//        curTarget = mSkels[2]->getBodyNode(0)->getTransform() * mTargetContactPoints[3];
//      else if (i == 4)
//        curTarget = mSkels[2]->getBodyNode(0)->getTransform() * mTargetContactPoints[4];
//      curTargetVel = Vector3d(0.0,0.0,0.0);

//      dynamic_cast<tasks::TrackEETask*>(mTasks[i])->updateTask(state,curTarget,otherForce);
//    }
//  }
}

//==============================================================================
void MyWindow::updateIntForce()
{
  // TODO(JS): Just commented out
//  mController->computeTorques(mDofs[1], mDofVels[1], mDofAccs[1], mDofs[2],
//                              mDofVels[2], mContactPoints, mTargetContactForces,
//                              mContactBodyNames, mTargetOri, mObjOri);
//  mSkels[1]->setForces(mController->getTorques());
}

//==============================================================================
void MyWindow::updateExtForce()
{
  // Compute all the constraint force, including contact force and joint limit
  // force
  bool ODEFlag = true;

  // TODO(JS): Just commented out
  //mConstraintSolver->applyConstraintForcesHand(ODEFlag);

  std::vector<int> numContact;
  numContact.resize(mFingerNum);
  for (int i = 0; i < mFingerNum; ++i)
  {
    mContactForces[i] = Vector3d::Zero();
    numContact[i] = 0;
  }

  collision::CollisionDetector* cd = mConstraintSolver->getCollisionDetector();

  // tangent relative velocity to be baked
  VectorXd bakeTanRelVel = VectorXd::Zero(cd->getNumContacts()*3);

  for (size_t i = 0; i < cd->getNumContacts(); ++i)
  {
    Contact contact = cd->getContact(i);

    for (int j = 0; j < mFingerNum; ++j)
    {
      if (contact.bodyNode1->getName() == mFingerTipNames[j])
      {
        numContact[j]++;
        mContactForces[j] = mContactForces[j] - contact.force;
        break;
      }
    }

    Vector3d normal       = contact.normal.normalized();
    Vector3d normalForce  = contact.force.dot(normal) * normal;
    Vector3d tangentForce = contact.force - normalForce;

    // TODO(JS): Just commented out
    MatrixXd B
        = mConstraintSolver->getTangentBasisMatrix(contact.point, contact.normal);
    bakeTanRelVel.segment(i * 3, 3)
        = mConstraintSolver->mB.col(i * mFrictionBasis).dot(
            mConstraintSolver->mQDot)*B.col(0);

    for (int j = 1; j < mFrictionBasis/2; ++j)
    {
      bakeTanRelVel.segment(i * 3, 3)
          += mConstraintSolver->mB.col(i * mFrictionBasis + j * 2).dot(
               mConstraintSolver->mQDot) * B.col(j * 2);
    }

    // bake the tangent relative velocity
    mBakedTanRelVels.push_back(bakeTanRelVel);
  }

  // TODO(JS): Just commented out
  //mController->mConstraintForce = mConstraintSolver->getTotalConstraintForce(1);
}

//==============================================================================
VectorXd MyWindow::updateForceTorqueConstraint()
{
  // example of setting force torque constraint
  // update initial direction
  mFirstInitDir = mSkels[2]->getBodyNode(0)->getTransform() * mFirstInitPoint
                  - Vector3d(mDofs[2][0], mDofs[2][1], mDofs[2][2]);
  mSecondInitDir = mSkels[2]->getBodyNode(0)->getTransform() * mSecondInitPoint
                   - Vector3d(mDofs[2][0], mDofs[2][1], mDofs[2][2]);

  Vector3d v
      = applyHandTransformInvDir(
          mSkels[2]->getBodyNode(0)->getWorldLinearVelocity());
  Vector3d p = applyHandTransformInv(mSkels[2]->getBodyNode(0)->getWorldCOM());
  double w
      = applyHandTransformInvDir(
          mSkels[2]->getBodyNode(0)->getWorldAngularVelocity()
        ).norm();

  Vector3d totalForce = Vector3d::Zero();
  Vector3d totalTorque = Vector3d::Zero();
  VectorXd forceTorqueConstraint = VectorXd::Zero(6);

  // the force should be transformed from hand frame to world frame
  forceTorqueConstraint.head(3) = applyHandTransformDir(totalForce);
  forceTorqueConstraint.tail(3) = applyHandTransformDir(totalTorque);

  forceTorqueConstraint(2) = -10.0;


  return forceTorqueConstraint;
}

void MyWindow::updateContactPoint()
{
  // example of target contact point
  // set target contact point and end point in ball coordinates
  double theta = 0.0;
  double phi = 0.0;
  double r = mObjRadius; // the radius of the approximate sphere

  theta = 2.0;
  phi = 1.2;
  Vector3d center(mDofs[2][0], mDofs[2][1], mDofs[2][2]);
  Vector3d position = center + Vector3d(r * cos(theta) * sin(phi),
                                        r * cos(phi),
                                        -r * sin(theta) * sin(phi));
  Vector3d localPosition = mSkels[2]->getBodyNode(0)->getTransform().inverse()
                           * position;
  Vector2d localBallPosition = cartesianToBall(localPosition, r);
  mContactPointsBallEnd[0] = localBallPosition;
  mTargetContactPoints[0] = ballToCartesian(localBallPosition, r);

  theta = -2.5;
  phi = 1.5;
  position = center + Vector3d(r * cos(theta) * sin(phi),
                               r * cos(phi),
                               -r * sin(theta) * sin(phi));
  localPosition = mSkels[2]->getBodyNode(0)->getTransform().inverse()
                  * position;
  localBallPosition = cartesianToBall(localPosition, r);
  mContactPointsBallEnd[1] = localBallPosition;
  mTargetContactPoints[1] = ballToCartesian(localBallPosition, r);

  theta = -1.8;
  phi = 1.5;
  position = center + Vector3d(r*cos(theta)*sin(phi),r*cos(phi),-r*sin(theta)*sin(phi));
  localPosition = mSkels[2]->getBodyNode(0)->getTransform().inverse()
                  * position;
  localBallPosition = cartesianToBall(localPosition, r);
  mContactPointsBallEnd[2] = localBallPosition;
  mTargetContactPoints[2] = ballToCartesian(localBallPosition, r);

  theta = -1.0;
  phi = 1.5;
  position = center + Vector3d(r*cos(theta)*sin(phi),r*cos(phi),-r*sin(theta)*sin(phi));
  localPosition = mSkels[2]->getBodyNode(0)->getTransform().inverse()
                  * position;
  localBallPosition = cartesianToBall(localPosition, r);
  mContactPointsBallEnd[3] = localBallPosition;
  mTargetContactPoints[3] = ballToCartesian(localBallPosition, r);

  theta = 0.5;
  phi = 1.5;
  position = center + Vector3d(r*cos(theta)*sin(phi),r*cos(phi),-r*sin(theta)*sin(phi));
  localPosition = mSkels[2]->getBodyNode(0)->getTransform().inverse()
                  * position;
  localBallPosition = cartesianToBall(localPosition, r);
  mContactPointsBallEnd[4] = localBallPosition;
  mTargetContactPoints[4] = ballToCartesian(localBallPosition, r);
}

Isometry3d MyWindow::evalHandTransform()
{
  return mSkels[1]->getBodyNode(1)->getTransform();
}

Isometry3d MyWindow::evalHandTransformInv()
{
  return mSkels[1]->getBodyNode(1)->getTransform();
}

Vector3d MyWindow::applyHandTransform(Vector3d _x)
{
  return evalHandTransform() *_x;
}

Vector3d MyWindow::applyHandTransformDir(Vector3d _v)
{
  return evalHandTransform().linear() * _v;
}

Vector3d MyWindow::applyHandTransformInv(Vector3d _x)
{
  return evalHandTransformInv() * _x;
}

Vector3d MyWindow::applyHandTransformInvDir(Vector3d _v)
{
  return evalHandTransformInv().linear() * _v;
}

Vector4d MyWindow::matrixToAxisAngle(Matrix3d& _m)
{
  Quaterniond q = Quaterniond(_m);
  Vector3d exp = math::quatToExp(q);
  Vector4d a;
  a(3) = exp.norm();
  a.head(3) = exp.normalized();
  return a;
}

Matrix3d MyWindow::axisAngleToMatrix(Vector4d& _v)
{
  Vector3d exp = _v(3)*_v.head(3);
  Quaterniond q = math::expToQuat(exp);
  Matrix3d m = Matrix3d(q);
  return m;
}

Matrix3d MyWindow::applyAxisAngle(Vector4d& _v1, Vector4d& _v2)
{
  Matrix3d m1 = axisAngleToMatrix(_v1);
  Matrix3d m2 = axisAngleToMatrix(_v2);

  return m2 * m1;
}

Vector3d MyWindow::evalVelOnObj(Vector3d& _l)
{
  Matrix<double, 3, 6> Jv
      = mSkels[2]->getBodyNode(0)->getWorldLinearJacobian(_l);

  return Jv * mDofVels[2];
}

bool MyWindow::forceAchievable(int _index, Vector3d _point, Vector3d _force)
{
  // TODO(JS): Just commented out
//  int numDepDofs = mSkels[1]->getBodyNode(_index)->getNumDependentGenCoords()
//                   - 4;
//  MatrixXd J = MatrixXd::Zero(3, numDepDofs);

//  //J =

//  for (int j = 4; j < numDepDofs + 4; j++)
//    J.col(j - 4) = dart_math::xformHom(mSkels[1]->getBodyNode(_index)->getDerivWorldTransform(j),_point);

//  MatrixXd B = J*J.transpose();
//  FullPivLU<MatrixXd> lu_decompB(B);

//  if (lu_decompB.rank() < 3)
//    return false;

//  MatrixXd A = B.inverse()*J;
//  MatrixXd Aug = MatrixXd::Zero(3,A.cols()+1);

//  Aug.block(0, 0, 3, A.cols()) = A;
//  Aug.block(0, A.cols(), 3, 1) = _force;

//  FullPivLU<MatrixXd> lu_decompA(A);
//  FullPivLU<MatrixXd> lu_decompAug(Aug);

//  if (lu_decompA.rank() == lu_decompAug.rank())
//    return true;
//  else
//    return false;
  return true;
}

void MyWindow::setHandAngle(double _angle)
{
  // change the rotation axis to change the roll direction
  Vector3d palmRotateAxis(1.0, 0.0, 0.0);
  Quaternion<double> q;
  q = (Quaternion<double>)AngleAxis<double>(_angle, palmRotateAxis);

  Vector3d palmInitOri = Vector3d(0.0, 1.0, 0.0).normalized();
  Vector3d palmOri = q*palmInitOri;

  angleX = palmOri(0);
  angleY = palmOri(1);
  angleZ = palmOri(2);
}

void MyWindow::setHandTrans(const Vector3d& _preOri, const Vector3d& _axis)
{
  // TODO(JS): This function is not used anywhere

  Skeleton* handSkel = mSkels[1];

  VectorXd pose = handSkel->getPositions();
//  Vector3d curOri = pose.head(3);
  VectorXd prePose = pose;
  prePose.head(3) = _preOri;
  mSkels[1]->setPositions(prePose);
  dynamics::BodyNode* wrist = handSkel->getBodyNode(mPalmName.c_str());
  Isometry3d preWorldTransformation = wrist->getTransform();
  Matrix3d preWorldRotation = preWorldTransformation.linear();
  Vector3d preWorldTranslation = preWorldTransformation.translation();
  mSkels[1]->setPositions(pose);

  Isometry3d curWorldTransformation = wrist->getTransform();
  Matrix3d curWorldRotation    = curWorldTransformation.linear();
  Vector3d curWorldTranslation = preWorldRotation * _axis
                                 + preWorldTranslation
                                 - curWorldRotation * _axis;

  Joint* rootJoint = handSkel->getJoint(0);
  VectorXd rootJointConfig = rootJoint->getPositions();

  Isometry3d rootT = Isometry3d::Identity();
  curWorldTranslation - preWorldTranslation;

//  handSkel->getJoint(0)->getTransform(0)->getDof(0)->setValue(
//        mSkels[1]->getJoint(0)->getTransform(0)->getDof(0)->getValue()
//        + curWorldTranslation(0)
//        - preWorldTranslation(0));
//  handSkel->getJoint(0)->getTransform(0)->getDof(1)->setValue(
//        mSkels[1]->getJoint(0)->getTransform(0)->getDof(1)->getValue()
//        + curWorldTranslation(1)
//        - preWorldTranslation(1));
//  handSkel->getJoint(0)->getTransform(0)->getDof(2)->setValue(
//        mSkels[1]->getJoint(0)->getTransform(0)->getDof(2)->getValue()
//        + curWorldTranslation(2)
//        - preWorldTranslation(2));
//  handSkel->getJoint(0)->updateStaticTransform();

  handSkel->setPositions(pose);
}

int MyWindow::evalContactEdge()
{
  CollisionDetector* cd       = mConstraintSolver->getCollisionDetector();
  Skeleton*          cubeSkel = mSkels[2];
  BodyNode*          cube     = cubeSkel->getBodyNode(0);

  // Find closest edges with the contact points in YZ-plane
  std::vector<bool> inContactEdges(mEdges.size(), false);
  for (size_t i = 0; i < cd->getNumContacts(); ++i)
  {
    const Vector3d&   contactPos      = cd->getContact(i).point;
    const Isometry3d& cubeInvT        = cube->getTransform().inverse();
    const Vector3d&   contactLocalPos = cubeInvT * contactPos;

    for (size_t j = 0; j < mEdges.size(); ++j)
    {
      // Distance between contact point and edges in YZ-plane
      double distance = (contactLocalPos.tail(2) - mEdges[j].tail(2)).norm();

      // If the distance is less than 0.001, mark inContactEdges[i] as true
      if (distance < 0.001)
        inContactEdges[j] = true;
    }
  }

  // Return the index of contact edges
  for (size_t i = 0; i < mEdges.size(); ++i)
  {
    if (inContactEdges[i] == true)
    {
      // If the contact edge is not single, then return -1
      for (size_t j = i + 1; j < mEdges.size(); ++j)
      {
        if (inContactEdges[j] == true)
          return -1;
      }

      // return the contact edge
      return i;
    }
  }

  // If we couldn't find any contact edge, then return -1
  return -1;
}

//==============================================================================
bool MyWindow::isEdgeInContact(int _edgeIndex)
{
  Skeleton* objectSkel = mSkels[2];
  BodyNode* object = objectSkel->getBodyNode(0);
  CollisionDetector* cd = mConstraintSolver->getCollisionDetector();

  for (size_t i = 0; i < cd->getNumContacts(); ++i)
  {
    Vector3d contactPos      = cd->getContact(i).point;
    Vector3d contactLocalPos = object->getTransform().inverse() * contactPos;

    const Vector2d& contactLocalPosYZ = contactLocalPos.tail<2>();
    const Vector2d& edgeYZ            = mEdges[_edgeIndex].tail<2>();
    const double& distance            = (contactLocalPosYZ - edgeYZ).norm();

    if (distance < EPS_DISTANCE)
      return true;
  }

  return false;
}

//==============================================================================
Matrix3d MyWindow::evalObjOri()
{
  const Skeleton*   objectSkel = mSkels[1];
  const BodyNode*   object     = objectSkel->getBodyNode(0);
  const Isometry3d& objectT    = object->getTransform();

  Vector3d oriAxisX = (mCorners[2][1] - mCorners[2][0]).normalized();
  Vector3d oriAxisY = (mCorners[3][1] - mCorners[2][1]).normalized();
  Vector3d oriAxisZ = oriAxisX.cross(oriAxisY);

  Vector3d axisX = (objectT * mCorners[2][1] - objectT * mCorners[2][0]).normalized();
  Vector3d axisY = (objectT * mCorners[3][1] - objectT * mCorners[2][1]).normalized();
  Vector3d axisZ = axisX.cross(axisY);

  Matrix3d oriRotMat;
  oriRotMat.col(0) = oriAxisX;
  oriRotMat.col(1) = oriAxisY;
  oriRotMat.col(2) = oriAxisZ;

  Matrix3d rotMat;
  rotMat.col(0) = axisX;
  rotMat.col(1) = axisY;
  rotMat.col(2) = axisZ;

  return rotMat * oriRotMat.inverse();
}

//==============================================================================
int MyWindow::evalUpFace() 
{
  const Skeleton*   handSkel = mSkels[1];
  const BodyNode*   palm     = handSkel->getBodyNode(4);
  const Isometry3d& invT     = palm->getTransform().inverse();

  int    highVerIndex = -1;
  double height       = 10.0;

  const Matrix3d& rotMat = evalObjOri();
  Vector3d transVec(mDofs[2][0], mDofs[2][1], mDofs[2][2]);

  for (size_t i = 0; i < mCorners.size(); ++i)
  {
    const Vector3d& cornerPinky       = mCorners[i][0];
    const Vector3d& cornerPinkyInPalm = invT * rotMat * cornerPinky + transVec;
    const double& cornerPinkyInPalmY = cornerPinkyInPalm[1];

    if (cornerPinkyInPalmY < height)
    {
      height = cornerPinkyInPalmY;
      highVerIndex = i;
    }
  }

  const size_t& index1 = (highVerIndex + 1) % mCorners.size();
  const size_t& index2 = (highVerIndex - 1) % mCorners.size();

  const double& val1 = (invT * rotMat * mCorners[index1][0] + transVec)[1];
  const double& val2 = (invT * rotMat * mCorners[index2][0] + transVec)[1];

  if (val1 < val2)
    return ((highVerIndex + 1) % mCorners.size());
  else
    return highVerIndex;
}

//==============================================================================
// can be implemented as get every corners global coordinate, translate to local coordinate, and sort based on the coordinates in other dimensions
void MyWindow::evalHighCorners()
{
  int upFaceIndex = evalUpFace();
  // TODO(JS): Just commented out
//  mCurHighCorners[0] = dart_math::xformHom(mSkels[1]->getBodyNode(4)->getWorldInvTransform(), mSkels[2]->getBodyNode(0)->getTransform() * mCorners[(upFaceIndex-1)%mCorners.size()][0]));
//  mCurHighCorners[1] = dart_math::xformHom(mSkels[1]->getBodyNode(4)->getWorldInvTransform(), mSkels[2]->getBodyNode(0)->getTransform() * mCorners[upFaceIndex][0]));
}

bool MyWindow::evalCornerChange()
{
  double cornerDis = 0.0;
  cornerDis = cornerDis + (mCurHighCorners[0]-mPreHighCorners[0]).norm() + (mCurHighCorners[1]-mPreHighCorners[1]).norm();
  if (cornerDis > 0.01) {
    return true;
  }
  else return false;
}

void MyWindow::evalRollDir()
{
  if (mCurHighCorners[0](2) > mPreHighCorners[0](2)) {
    mRollDir = 1;
  }
  else if (mCurHighCorners[0](2) < mPreHighCorners[0](2)) {
    mRollDir = -1;
  }
  else {
    mRollDir = 0;
  }
}
