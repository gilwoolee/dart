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

#include "Controller.h"

#include "dart/common/Console.h"
#include "dart/math/Helpers.h"
#include "dart/dynamics/Skeleton.h"
#include "dart/dynamics/BodyNode.h"
#include "dart/dynamics/Shape.h"
#include "dart/dynamics/Joint.h"
#include "dart/constraint/ConstraintSolver.h"
#include "dart/constraint/WeldJointConstraint.h"
#include "dart/collision/CollisionDetector.h"
#include "dart/simulation/World.h"

#include "TrackOriTask.h"

#define EPS_DISTANCE 0.001

double gAngleX = 0.0;
double gAngleY = 1.0;
double gAngleZ = 0.0;
double gHandPosX = 0.0;
double gHandPosY = 0.0;
double gHandPosZ = 0.0;
double gObjPosX = 0.0;
double gObjPosY = 0.0;
double gObjPosZ = 0.0;

using namespace std;

using namespace Eigen;

using namespace dart;
using namespace collision;
using namespace constraint;
using namespace dynamics;
using namespace simulation;

//==============================================================================
Controller::Controller(World* _world,
                       Skeleton* _ground,
                       Skeleton* _shadowHand,
                       Skeleton* _object)
  : mWorld(_world),
    mGround(_ground),
    mHand(_shadowHand),
    mObjectSkel(_object)
{
  assert(_world);
  assert(_ground);
  assert(_shadowHand);
  assert(_object);

  //----------------------------------------------------------------------------
  // Initial states
  //----------------------------------------------------------------------------

  setInitialTransformation();
  backupInitialStates();

  //----------------------------------------------------------------------------
  // World setting
  //----------------------------------------------------------------------------

  mWorld->setGravity(Eigen::Vector3d(0, -9.81, 0));
  mTimestep = mWorld->getTimeStep();

  mConstratinSolver = mWorld->getConstraintSolver();
  mCollisionDetector = mConstratinSolver->getCollisionDetector();

  //----------------------------------------------------------------------------
  // Hand setting
  //----------------------------------------------------------------------------

  mHand->disableSelfCollision();
  mHandDof = mHand->getNumDofs();
  mFingerNum = 5;

  std::string palmName = "palm";
  mPalm = mHand->getBodyNode(palmName);
  assert(mPalm);

  mFingerNames.resize(mFingerNum);
  mFingerNames[0] = "thdistal";
  mFingerNames[1] = "ffdistal";
  mFingerNames[2] = "mfdistal";
  mFingerNames[3] = "rfdistal";
  mFingerNames[4] = "lfdistal";

  mFingerRootNames.resize(mFingerNum);
  mFingerRootNames[0] = "thbase";
  mFingerRootNames[1] = "ffknuckle";
  mFingerRootNames[2] = "mfknuckle";
  mFingerRootNames[3] = "rfknuckle";
  mFingerRootNames[4] = "lfknuckle";

//  mFingerTipIndices.resize(mFingerNum);
//	for (int i = 0; i < mFingerNum; ++i)
//  {
//		mFingerTipIndices[i] = mHand->getBodyNode(mFingerNames[i])->getSkelIndex();
//	}

  //----------------------------------------------------------------------------
  // Object setting
  //----------------------------------------------------------------------------

  mObject = mObjectSkel->getBodyNode(0);

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
  mEdges[0] = Eigen::Vector3d(0.0, -0.02,  0.02);
  mEdges[1] = Eigen::Vector3d(0.0,  0.02,  0.02);
  mEdges[2] = Eigen::Vector3d(0.0,  0.02, -0.02);
  mEdges[3] = Eigen::Vector3d(0.0, -0.02, -0.02);

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

  // TODO: Precomputed angles
  mAngles[0] = 0.740675;
  mAngles[1] = 0.087237;
  mAngles[2] = -0.358023;

//  mAngles[0] = 0.840675;
//  mAngles[1] = 0.437237;
//  mAngles[2] = -0.358023;

  mRollDir = 0;

  setHandAngle(mAngles[0]);

  mPreOri = mHand->getPositions().head<4>();
  mPreContactEdge = 0;

  mUpFace = 2;

  mPreHighCorners.resize(2);
  mCurHighCorners.resize(2);

  mPreHighCorners[0] = mPalm->getTransform().inverse()
                       * mObject->getTransform()
                       * mCorners[1][0];
  mPreHighCorners[1] = mPalm->getTransform().inverse()
                       * mObject->getTransform()
                       * mCorners[2][0];
  mCurHighCorners[0] = mPreHighCorners[0];
  mCurHighCorners[1] = mPreHighCorners[1];

  //----------------------------------------------------------------------------
  // Hand controller setting
  //----------------------------------------------------------------------------

  mDesiredDofs.resize(mHandDof);

//  float fingerRestPose[] = {0, -0.8000000119, 0, 0, 0, 0, 0, 0, -0.7428935766, 0, 0, 0, 0, 1.072659612, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.8000000119, 0, 0.200000003};
//  float fingerInterceptPose[] = {0, -0.8, 0, 0, 0, 0, 0, 0, 0, 0.4, 0, 0.4, 0.4, 0, 0.6, 0, 0.6, 0.6, 0, 0.4, 0.6, 0.4, 0.4, 0, 0.4, 0};
  for (int i = 0; i < mHandDof; i++)
  {
    mDesiredDofs[i] = mHand->getPosition(i);
//    mFingerRestPose[i] = fingerRestPose[i];
//    mFingerInterceptPose[i] = fingerInterceptPose[i];
  }

  // Set flags related to control
  mOriFlag = true;

  // Resize all the control forces
  mHandControlForce.setZero(mHandDof);
  mHandGravityCompensationForce.setZero(mHandDof);
  mHandObjControlForce.setZero(mHandDof);
  mHandOriForce.setZero(mHandDof);
  mHandMaintainForce.setZero(mHandDof);
  mHandConstraintForce.setZero(mHandDof);
  mHandSPDDampForce.setZero(mHandDof);
  mHandTrackForce.setZero(mHandDof);
  mHandTaskForce.setZero(mHandDof);

  // use three DoF at the wrist to control rolling motion
  mOriDofs.resize(3);
  mOriDofs(0) = 1;
  mOriDofs(1) = 2;
  mOriDofs(2) = 3;
//  mOriDofs(3) = 3;

  // Set spd gains
  mKp.setZero(mHandDof, mHandDof);
  mKd.setZero(mHandDof, mHandDof);
  // -- arm and wrist
  for (int i = 0; i < 4; i++)
  {
    mKp(i, i) = 50.0;
    mKd(i, i) = 20.0;
  }
  // -- hand
  for (int i = 4; i < mHandDof; i++)
  {
    mKp(i, i) = 15.0;
    mKd(i, i) = 2.0;
  }

  mPreOriTarget.setZero();
  mAccumulateOriError.setZero();

}

//==============================================================================
Controller::~Controller()
{
}


//==============================================================================
void Controller::setInitialTransformation()
{
  // Ground initial generalized positions
  int groundDof = mGround->getNumDofs();  // 6
  mGroundInitialPositions.setZero(groundDof);
  mGroundInitialPositions[4] = -0.5;  // y-axis

  // Hand initial generalized positions
  int handDof = mHand->getNumDofs();  // 6
  mHandInitialPositions.setZero(handDof);

//  mHandInitialPositions <<
//      0, -0.8000000119,          // lowerarm, handsupport
//      0, 0,                      // wrist(<-->, ^V)
//      0, 0, 0, 0,                // 2nd finger
//      -0.7428935766, 0, 0, 0, 0, // 5th finger
//      1.072659612, 0, 0, 0,      // 3rd finger
//      0, 0, 0, 0,                // 4th finger
//      0, 0, 0.8000000119, 0, 0.200000003 // 1st finger (thumb)
//      ;

  mHandInitialPositions <<
      0, -0.8000000119,          // lowerarm, handsupport
      0, 0,                      // wrist(<-->, ^V)
      0, 0, 0, 0,                // 2nd finger
      0, 0, 0, 0, 0,             // 5th finger
      0, 0, 0, 0,         // 3rd finger
      0, 0, 0, 0,                // 4th finger
      -0.7428935766, 1.072659612, 0, 0.8000000119, 0.200000003  // 1st finger (thumb)
      ;

  // Object initial generalized positions
  int objectDof = mObjectSkel->getNumDofs();
  mObjectInitialPositions.setZero(objectDof);
  mObjectInitialPositions
      << 0.0, 0.0, 0.0, 0.0775, -0.160, 0.23;
//  mObjectInitialPositions[4] = -0.155;
//  mObjectInitialPositions[3] = -10.155;

  mGround->setPositions(mGroundInitialPositions);
  mHand->setPositions(mHandInitialPositions);
  mObjectSkel->setPositions(mObjectInitialPositions);

  mGround->computeForwardKinematics(true, true, false);
  mHand->computeForwardKinematics(true, true, false);
  mObjectSkel->computeForwardKinematics(true, true, false);
}

//==============================================================================
void Controller::backupInitialStates()
{
  mHandStateBackup   = mHand->getState();
  mObjectStateBackup = mObjectSkel->getState();
}

//==============================================================================
void Controller::restoreInitialStates()
{
  mHand->setState(mHandStateBackup);
  mObjectSkel->setState(mObjectStateBackup);

  mHand->computeForwardKinematics(true, true, false);
  mObjectSkel->computeForwardKinematics(true, true, false);
}

//==============================================================================
void Controller::update(double /*_currentTime*/)
{
  //
  updateHandPose();


  // Compute target palm angles
  setPose();

  //
  updateContact();

  // Compute control forces for the target palm angles
  computeHandTotalControlForce();

  // Apply the control forces to the hand
  mHand->setForces(mHandControlForce);
}

//==============================================================================
void Controller::setPose()
{
  const bool isFirstRoll = mRollNum == 0 ? true : false;

  const Isometry3d objTransform = mObject->getTransform();

  //dynamics::BodyNode* wrist = mSkels[1]->getBodyNode(mPalmName);
  int contactEdgeIndex = evalContactEdge();

  if (contactEdgeIndex != -1)
    if (mPreContactEdge != contactEdgeIndex || contactEdgeIndex == 0)
      std::cout << "ContactEdge: " << contactEdgeIndex << std::endl;

  if (contactEdgeIndex == mPreContactEdge || contactEdgeIndex == -1)
  {
    //setHandTrans(mPreOri,mEdges[contactEdgeIndex]);
  }
  else if (contactEdgeIndex != -1)
  {
    mPreContactEdge = contactEdgeIndex;
  }

  // If we found contact edge, then set it as previous contact edge
//  if (contactEdgeIndex != -1)
//    mPreContactEdge = contactEdgeIndex;

  mPreOri = mHand->getPositions().head<4>();

  const bool isLastRoll = mRollNum == mN - 1 ? true : false;
  assert(mRollNum <= mN - 1);

  int index1 = (mRollNum                    ) % mEdges.size();
  int index2 = (mRollNum + mEdges.size() / 2) % mEdges.size();

//  std::cout << "index2: " << index2 << std::endl;


//  std::cout << "mRollNum: " << mRollNum << std::endl;

  if (isLastRoll)
  {
    static bool called = false;
    if (!called)
      std::cout << "----------------- Last roll detected. ----------------" << std::endl;
    called = true;
  }

  if (!isLastRoll)
  {
    if (contactEdgeIndex == mRollNum % (int)mEdges.size())
    {
      // contact edge position in world coordinate
      Vector3d contactPos = mObject->getTransform() * mEdges[contactEdgeIndex];

      // if change the roll direction, the condition will be changed
      // accordingly, related to roll direction
      // Assume the the rolling direction is algon with z-axis
      const double& comZ        = mObject->getWorldCOM()[2];
      const double& contactPosZ = contactPos[2];

      const bool& cond1 = comZ - contactPosZ > 0.005 ? true : false;
      const bool& cond2 = comZ - contactPosZ > 0.002 ? true : false;

//      std::cout << "contactEdgeIndex  : " << contactEdgeIndex << std::endl;
//      std::cout << "mRollNum          : " << mRollNum << std::endl;
//      std::cout << "comZ              : " << comZ << std::endl;
//      std::cout << "contactPosZ       : " << contactPosZ << std::endl;
//      std::cout << "comZ - contactPosZ: " << comZ - contactPosZ << std::endl;
//      std::cout << std::endl;

      //
      if ((cond1 && isFirstRoll) || (cond2 && !isFirstRoll))
      {
        const int index1 = (mRollNum + mEdges.size() - 1) % mEdges.size();
        const int index2 = (mRollNum + mEdges.size() + 1) % mEdges.size();
        const int index3 = (mRollNum                    ) % mEdges.size();

        const Vector3d& liftEdge    = objTransform * mEdges[index1];
        const Vector3d& dropEdge    = objTransform * mEdges[index2];
        const Vector3d& contactEdge = objTransform * mEdges[index3];

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
//          std::cout << "Target angel: " << mAngles[mRollNum] << std::endl;
          setHandAngle(mAngles[mRollNum]);
        }
      }
    }
  }
  else if (isLastRoll && (/*isEdgeInContact(index1) || */evalUpFace() == index2))
  {
//    if (isEdgeInContact(index1))
//      std::cout << "COND 1 !!";

//    if (evalUpFace() == index2)
//      std::cout << "COND 2 !!";

//    std::cout << "Target angel: " << 0.0 << std::endl;
    setHandAngle(0.0);
  }

  std::cout << "(evalUpFace, index2): " << evalUpFace() << ", " << index2 << std::endl;

  // calculate up face if know the high corners local coordinates
  if (evalUpFace() != mUpFace)
    mUpFace = evalUpFace();

//  std::cout << "RollNum: " << mRollNum << std::endl;

////  for (unsigned int i = 0; i < mSkels.size(); i++)
////  {
////    mSkels[i]->setPositions(mDofs[i]);
////    mSkels[i]->setVelocities(mDofVels[i]);
////    mSkels[i]->computeForwardKinematics(true, false, false);

////    if (mSkels[i]->isMobile())
////    {
////      // need to update first derivatives for collision
////      mSkels[i]->setGravity(mGravity);
////      mSkels[i]->setPositions(mDofs[i]);
////      mSkels[i]->setVelocities(mDofVels[i]);
////      mSkels[i]->computeForwardKinematics(true, true, false);
////    }
////    else
////    {
////      // need to update node transformation for collision
////      mSkels[i]->setPositions(mDofs[i]);
////      mSkels[i]->computeForwardKinematics(true, false, false);
////    }
  ////  }
}

//==============================================================================
void Controller::updateContact()
{

}

//==============================================================================
void Controller::setHandAngle(double _angle)
{
  // change the rotation axis to change the roll direction
  Vector3d palmRotateAxis = Vector3d::UnitX();
  Quaternion<double> q
      = (Quaternion<double>)AngleAxis<double>(_angle, palmRotateAxis);

  Vector3d palmInitOri = Vector3d::UnitY();
  Vector3d palmOri = q * palmInitOri;

  gAngleX = palmOri(0);
  gAngleY = palmOri(1);
  gAngleZ = palmOri(2);
}

//==============================================================================
int Controller::evalContactEdge()
{
  CollisionDetector* cd = mCollisionDetector;

  // Find closest edges with the contact points in YZ-plane
  std::vector<bool> inContactEdges(mEdges.size(), false);
  for (size_t i = 0; i < cd->getNumContacts(); ++i)
  {
    const Vector3d&   contactPos      = cd->getContact(i).point;
    const Isometry3d& cubeInvT        = mObject->getTransform().inverse();
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
  int contactEdgeIndex = -1;
  for (size_t i = 0; i < mEdges.size(); ++i)
  {
    if (inContactEdges[i] == true)
    {
      // If the contact edge is not single, then return -1
      for (size_t j = 0; j < mEdges.size(); ++j)
      {
        if (j != i && inContactEdges[j] == true)
          return -1;
      }

      // return the contact edge
      contactEdgeIndex = i;
    }
  }

  // If we couldn't find any contact edge, then return -1
  return contactEdgeIndex;
}

//==============================================================================
bool Controller::isEdgeInContact(int _edgeIndex)
{
  CollisionDetector* cd = mCollisionDetector;

  for (size_t i = 0; i < cd->getNumContacts(); ++i)
  {
    Vector3d contactPos      = cd->getContact(i).point;
    Vector3d contactLocalPos = mObject->getTransform().inverse() * contactPos;

    const Vector2d& contactLocalPosYZ = contactLocalPos.tail<2>();
    const Vector2d& edgeYZ            = mEdges[_edgeIndex].tail<2>();
    const double& distance            = (contactLocalPosYZ - edgeYZ).norm();

    if (distance < EPS_DISTANCE)
      return true;
  }

  return false;
}

//==============================================================================
Matrix3d Controller::evalObjOri()
{
  const BodyNode*   mObject = mObjectSkel->getBodyNode(0);
  const Isometry3d& objectT = mObject->getTransform();

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
int Controller::evalUpFace()
{
  const BodyNode*   palm     = mPalm;
  const Isometry3d& invT     = palm->getTransform().inverse();

  int    highVerIndex = -1;
  double height       = 10.0;

  const Matrix3d& rotMat = evalObjOri();
  Vector3d transVec = mObject->getTransform().translation();
//  Vector3d transVec(mDofs[2][0], mDofs[2][1], mDofs[2][2]);

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
void Controller::computeHandTotalControlForce()
{
  //----------------------------------------------------------------------------
  // Update mHandConstraintForce
  //
  computeHandConstraintForce();
  //----------------------------------------------------------------------------

  //----------------------------------------------------------------------------
  //
  //
  computeHandGravityCompensationForce();
  //----------------------------------------------------------------------------

  //----------------------------------------------------------------------------
  //
  //
  // computeHandObjControlForce();
  //----------------------------------------------------------------------------

  //----------------------------------------------------------------------------
  //
  //
  computeTrackForce();
  //----------------------------------------------------------------------------

  //----------------------------------------------------------------------------
  //
  //
  computeHandOriForce();
  //----------------------------------------------------------------------------


  //----------------------------------------------------------------------------
  //
  //
  // computeHandMaintainForce();
  //----------------------------------------------------------------------------


  //----------------------------------------------------------------------------
  //
  //
  // computeHandTaskForce();
  //----------------------------------------------------------------------------




  //----------------------------------------------------------------------------
  //
  //
//  computeHandDampForce();
  //----------------------------------------------------------------------------


  //----------------------------------------------------------------------------
  //
  //
  // computeHandMaintainForce();
  //----------------------------------------------------------------------------


  //----------------------------------------------------------------------------
  //
  //
  mHandControlForce = mHandGravityCompensationForce
//                      + mHandObjControlForce
                      + mHandSPDDampForce
                      + mHandTrackForce
//                      + mHandTaskForce
                      + mHandOriForce
                      ;
//  std::cout << "mHandOriForce: " << mHandOriForce.transpose() << std::endl;
  //----------------------------------------------------------------------------
}

//==============================================================================
void Controller::computeHandConstraintForce()
{
//  int dof = mHand->getNumDofs();
  mHandConstraintForce = mHand->getConstraintForces();
}

//==============================================================================
void Controller::computeHandTaskForce()
{
  int dof = mHand->getNumDofs();
  mHandTaskForce.setZero(dof);
}

//==============================================================================
void Controller::computeHandObjControlForce()
{
  int dof = mHand->getNumDofs();
  mHandObjControlForce.setZero(dof);
}

//==============================================================================
void Controller::computeTrackForce()
{
  int dof = mHand->getNumDofs();
  mHandTrackForce.setZero(dof);

  // other force
  Eigen::VectorXd otherForce
      = mHandGravityCompensationForce // force to be added by controller
        + mHandObjControlForce        // force to be added by controller
        + mHandOriForce               // force to be added by controller
        + mHandSPDDampForce           // force to be added by controller
        + mHandMaintainForce          // force to be added by controller
//        + mHand->getExternalForces()  // force to be added by simulator
        + mHandConstraintForce        // force to be added by simulator
        ;

  // track a pose using SPD
  VectorXd  q = mHand->getPositions();
  VectorXd dq = mHand->getVelocities();
  VectorXd Cg = mHand->getCoriolisAndGravityForces();
  MatrixXd invM = (mHand->getAugMassMatrix() + mKd * mTimestep).inverse();
  VectorXd p = -mKp * (q + dq * mTimestep - mDesiredDofs);
  VectorXd d = -mKd * dq;
  VectorXd qddot = invM * (-Cg + p + d + otherForce);

//  mHandTrackForce = p + d - mKd * qddot * mTimestep;
  mHandTrackForce = -10*mKp*(q - mDesiredDofs) - mKd * dq;

  if (mOriFlag)
  {
//    for (int i = 0; i < mOriDofs.size(); ++i)
//      mHandTrackForce(mOriDofs(i)) = 0.0;
    mHandTrackForce(3) = 0.0;
  }

  // TODO(JS): mControlFlags is always false for our rolling controller
//  if (mControlFlag)
//  {
//    // if use the tracking force as secondary force than not set the tracking force to be zero
//    for (int i = 0; i < taskDofIndex.size(); ++i)
//      mHandTrackForce(taskDofIndex[i]) = 0.0;
//  }

  // not using tracking force to prevent dropping
//  for (int i = 0; i < mFingerNum; ++i)
//  {
//    if (mContactFingers[i]
//        && !mInContactFingers[i]
//        && mControlFlag
//        && !mTrackFingers[i])
//    {
//      for (int j = 0; j < mFingerDofs[i].size(); ++j)
//        mHandTrackForce(mFingerDofs[i](j)) = 0.0;
//    }
//  }

//  std::cout << "mDesiredDofs: " << mDesiredDofs.transpose() << std::endl;
}

//==============================================================================
void Controller::computeTaskForce()
{
  int dof = mHand->getNumDofs();
  mHandTaskForce.setZero(dof);
}

//==============================================================================
void Controller::computeHandGravityCompensationForce()
{
  mHandGravityCompensationForce = mHand->getCoriolisAndGravityForces()
                              - mHand->getCoriolisForces();

  //  std::cout << "g: " << mGravityCompensationForce.transpose();
}

//===============================================================================
void Controller::computeHandDampForce()
{
  // other force
  Eigen::VectorXd otherForce;
  otherForce =
      mHandGravityCompensationForce
//      + mHand->getExternalForces()
      + mHandObjControlForce
      + mHandOriForce
      + mHandSPDDampForce
      + mHandMaintainForce
      + mHandConstraintForce;

  // damp using SPD
  VectorXd dq = mHand->getVelocities();
  VectorXd Cg = mHand->getCoriolisAndGravityForces();
  MatrixXd invM = (mHand->getAugMassMatrix() + mKd * mTimestep).inverse();
  VectorXd p = -mKp * (dq  * mTimestep);
  VectorXd d = -mKd * dq ;
  VectorXd qddot = invM * (-Cg + p + d + otherForce);

  mHandSPDDampForce = 0.05 * (p + d - mKd * qddot * mTimestep);
}

//==============================================================================
void Controller::computeHandOriForce()
{
  /////////////////////////
  ///
//  setHandAngle(-DART_PI / 12);
//  setHandAngle(0);
  ///
  /////////////////////////

  int dof = mHand->getNumDofs();

  if (mOriFlag == false)
  {
    mHandOriForce.setZero(dof);
    return;
  }

  tasks::TrackOriTask* trackPalmOri
      = new tasks::TrackOriTask("trackWristOri", mHand, mPalm);

  Eigen::Vector3d targetAngles(gAngleX, gAngleY, gAngleZ);
  targetAngles.normalize();
  trackPalmOri->setTarget(targetAngles);

  // state of hand
  Eigen::VectorXd  q = mHand->getPositions();
  Eigen::VectorXd dq = mHand->getVelocities();
  Eigen::VectorXd state = Eigen::VectorXd::Zero(q.size() + dq.size());
  state << q, dq;

  // other force
  Eigen::VectorXd otherForce = Eigen::VectorXd::Zero(mHandDof);
  otherForce
      = mHandGravityCompensationForce // force to be added by controller
        + mHandObjControlForce        // force to be added by controller
        + mHandOriForce               // force to be added by controller
        + mHandSPDDampForce           // force to be added by controller
        + mHandMaintainForce          // force to be added by controller
//        + mHand->getExternalForces()  // force to be added by simulator
        + mHandConstraintForce;       // force to be added by simulator

  trackPalmOri->updateTask(state, targetAngles, otherForce);

  Eigen::Vector3d diff = mPreOriTarget - trackPalmOri->getTarget();
  if (dart::math::isZero(diff.norm()))
  {
    mAccumulateOriError = mAccumulateOriError
                          + trackPalmOri->evalTaskError() * mTimestep;
  }
  else
  {
    mAccumulateOriError = Eigen::Vector3d::Zero();
    mPreOriTarget = trackPalmOri->getTarget();
    //getchar();
  }

  trackPalmOri->setAccumulateError(mAccumulateOriError);

  trackPalmOri->evalTorque();
  mHandOriForce = trackPalmOri->mTorque;

//  std::cout << "mHandOriForce: " << mHandOriForce.transpose() << std::endl;
}

//==============================================================================
void Controller::computeHandMaintainForce()
{
  int dof = mHand->getNumDofs();
  mHandMaintainForce.setZero(dof);

}

//==============================================================================
VectorXd Controller::computeOtherForces()
{
  Eigen::VectorXd otherForce;
  otherForce =
      mHandGravityCompensationForce
//      + mHand->getExternalForces()
      + mHandObjControlForce
      + mHandOriForce
      + mHandSPDDampForce
      + mHandMaintainForce
      + mHandConstraintForce;
}

//==============================================================================
void Controller::keyboard(unsigned char _key, int _x, int _y,
                          double _currentTime)
{
  switch (_key)
  {
    case 'i':  // Reset robot
      printDebugInfo();
      break;
    case 'r':  // Reset robot
      reset();
      break;

    default:
      break;
  }
}

//==============================================================================
void Controller::printDebugInfo() const
{
  std::cout << "[Shadow hand]"  << std::endl
            << " NUM NODES : " << mHand->getNumBodyNodes() << std::endl
            << " NUM DOF   : " << mHand->getNumDofs() << std::endl
            << " NUM JOINTS: " << mHand->getNumBodyNodes() << std::endl;

  for(size_t i = 0; i < mHand->getNumBodyNodes(); ++i)
  {
    Joint* joint = mHand->getJoint(i);
    BodyNode* body = mHand->getBodyNode(i);
    BodyNode* parentBody = mHand->getBodyNode(i)->getParentBodyNode();

    std::cout << "  Joint [" << i << "]: "
              << joint->getName()
              << " (" << joint->getNumDofs() << ")"
              << std::endl;
    if (parentBody != NULL)
    {
      std::cout << "    Parent body: " << parentBody->getName() << std::endl;
    }

    std::cout << "    Child body : " << body->getName() << std::endl;
  }
}

////==============================================================================
void Controller::reset()
{
  restoreInitialStates();

  dtmsg << "Robot is reset." << std::endl;
}


