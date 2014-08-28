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

#include "apps/handTiltingAngle/MyWindow.h"

#include "dart/math/Helpers.h"
#include "dart/gui/GLFuncs.h"
#include "dart/dynamics/BodyNode.h"
#include "dart/dynamics/Skeleton.h"
#include "dart/dynamics/FreeJoint.h"
#include "dart/dynamics/BoxShape.h"
#include "dart/constraint/ConstraintSolver.h"
#include "dart/simulation/World.h"

using namespace Eigen;

using namespace dart;
using namespace dynamics;

//==============================================================================
MyWindow::MyWindow(): SimWindow()
{
  mForce = Eigen::Vector3d::Zero();
  mRollNum = 0;
}

//==============================================================================
MyWindow::~MyWindow()
{
}

//==============================================================================
void MyWindow::timeStepping()
{
  mWorld->step();

  int contactEdgeIndex = evalContactEdge();
  if (contactEdgeIndex == mRollNum % mEdges.size())
  {
    // (Original Code) Eigen::Vector3d contactPos = mWorld->getSkeleton(1)->getBodyNode(0)->evalWorldPos(mEdges[contactEdgeIndex]); // contact edge position in world coordinate
    BodyNode* ground = mWorld->getSkeleton(0)->getBodyNode(0);
    BodyNode* cube = mWorld->getSkeleton(1)->getBodyNode(0);

    Isometry3d groundT = ground->getTransform();
    Isometry3d cubeT = cube->getTransform();

    // contact edge position in world coordinate
    Vector3d contactPos = cubeT * mEdges[contactEdgeIndex];

    //
    Vector3d contactLocalPos = groundT.inverse() * contactPos;

    //
    //if (mWorld->getSkeleton(1)->getWorldCOM()(0)-contactPos(0) < -0.0)
    {
      //
      if (mRollNum < mN - 1)
      {
        size_t index = 0;

        //
        index = (mRollNum + mEdges.size() - 1) % mEdges.size();
        Eigen::Vector3d liftEdge = cubeT * mEdges[index];

        //
        index = (mRollNum + mEdges.size() + 1) % mEdges.size();
        Eigen::Vector3d dropEdge = cubeT * mEdges[index];

        //
        index = (mRollNum) % mEdges.size();
        Eigen::Vector3d contactEdge = cubeT * mEdges[index];

        const double& liftLenX = liftEdge[0] - contactEdge[0];
        const double& liftLenY = liftEdge[1] - contactEdge[1];
        double liftAngle = atan(liftLenY / liftLenX);

        const double& dropLenX = contactEdge[0] - dropEdge[0];
        const double& dropLenY = dropEdge[1] - contactEdge[1];
        double dropAngle = atan(dropLenY / dropLenX);

        if (liftAngle > mAngles[mRollNum + 1] && dropAngle > -mAngles[mRollNum + 1])
        {
          mRollNum++;
          setGroundAngle(mAngles[mRollNum], contactLocalPos);
        }
      }
    }
  }
}

//==============================================================================
void MyWindow::drawSkels()
{
  glEnable(GL_LIGHTING);
  glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
  Vector4d color;
  color << 0.95, 0.95, 0.95, 1.0;
  mWorld->getSkeleton(0)->draw(mRI, color, false);
  color << 0.8, 0.3, 0.3, 1.0;
  mWorld->getSkeleton(1)->draw(mRI, color, false);
}

//==============================================================================
void MyWindow::keyboard(unsigned char key, int x, int y)
{
  Eigen::VectorXd pose = mWorld->getSkeleton(1)->getPositions();

  switch(key)
  {
    case ' ': // use space key to play or stop the motion
      mSimulating = !mSimulating;
      if(mSimulating)
      {
        mPlay = false;
        glutTimerFunc( mDisplayTimeout, refreshTimer, 0);
      }
      break;
    case 'p': // playBack
      mPlay = !mPlay;
      if (mPlay)
      {
        mSimulating = false;
        glutTimerFunc( mDisplayTimeout, refreshTimer, 0);
      }
      break;
    case '[': // step backward
      if (!mSimulating)
      {
        mPlayFrame--;
        if(mPlayFrame < 0)
          mPlayFrame = 0;
        glutPostRedisplay();
      }
      break;
    case ']': // step forwardward
      if (!mSimulating)
      {
        mPlayFrame++;
        if(mPlayFrame >= mWorld->getRecording()->getNumFrames())
          mPlayFrame = 0;
        glutPostRedisplay();
      }
      break;
    case 'v': // show or hide markers
      mShowMarkers = !mShowMarkers;
      break;
    case '1': // upper right force
      mForce[0] = -500;
      break;
    case '2': // upper right force
      mForce[0] = 500;
      break;
    case '3': // upper right force
      mForce[2] = -500;
      break;
    case '4': // upper right force
      mForce[2] = 500;
      break;
    case '=': // increase DoF value
      pose(0) = pose(0) + 0.1;
      mWorld->getSkeleton(1)->setPositions(pose);
      mWorld->getSkeleton(1)->computeForwardKinematics(true, true, false);
      std::cout << mWorld->getSkeleton(1)->getBodyNode(0)->getTransform().matrix() << std::endl;
      // 		std::cout << dart_math::xformHom(mWorld->getSkeleton(1)->getBodyNode(0)->getWorldInvTransform(), mEdges[0]).transpose() << std::endl;
      break;
    case '-': // decrease DoF value
      pose(0) = pose(0) - 0.1;
      mWorld->getSkeleton(1)->setPositions(pose);
      mWorld->getSkeleton(1)->computeForwardKinematics(true, true, false);
      std::cout << mWorld->getSkeleton(1)->getBodyNode(0)->getTransform().matrix() << std::endl;
      // 		std::cout << dart_math::xformHom(mWorld->getSkeleton(1)->getBodyNode(0)->getWorldInvTransform(), mEdges[0]).transpose() << std::endl;
      break;
    default:
      Win3D::keyboard(key,x,y);

  }
  glutPostRedisplay();
}

//==============================================================================
void MyWindow::setInitVel(Eigen::Vector3d _vel)
{
  mInitVel = _vel;
}

//==============================================================================
void MyWindow::evalN()
{
  mN = 3;
}

//==============================================================================
void MyWindow::evalGeometry()
{
  // the order is determined by the order of pivoting
  size_t numEdges = 4;
  mEdges.resize(numEdges);

  mEdges[0] = Vector3d(-0.02, -0.02, 0.0);
  mEdges[1] = Vector3d(-0.02,  0.02, 0.0);
  mEdges[2] = Vector3d( 0.02,  0.02, 0.0);
  mEdges[3] = Vector3d( 0.02, -0.02, 0.0);

  mCOM = Vector3d(0.0, 0.0, 0.0);

  // can define the faces, and calculate the alpha and phi
  mAlphas.resize(numEdges);
  mPhis.resize(numEdges);
  mRs.resize(numEdges);

  // Compute the angle between COM and pivoting edge with the previous face, and
  // the angle between COM and pivoting edge with the next face
  for (size_t i = 0; i < numEdges; ++i)
    mAlphas[i] = mPhis[i] = DART_PI / 4.0;

  // Compute distance between the pivoting egde and the COM
  for (size_t i = 0; i < numEdges; ++i)
    mRs[i] = (mEdges[i] - mCOM).norm();
}

//==============================================================================
void MyWindow::evalInertia()
{
  size_t numEdges = mEdges.size();
  mIs.resize(numEdges);

  //---- compute local inertia
//  double numerator   = 0.0;
//  double denominator = 0.0;

//  for (size_t i = 0; i < mEdges.size()-1; ++i)
//  {
//    Vector3d p = mEdges[i + 1].cross(mEdges[i]);

//    double p1 = mEdges[i + 1].dot(mEdges[i + 1]);
//    double p2 = mEdges[i + 1].dot(mEdges[i    ]);
//    double p3 = mEdges[i    ].dot(mEdges[i    ]);

//    denominator = denominator + p.norm();
//    numerator   = numerator   + p.norm() * (p1 + p2 + p3);
//  }

//  double I = mWorld->getSkeleton(1)->getBodyNode(0)->getMass()
//             * numerator / denominator / 6.0;

  // the order is determined by the order of pivoting
  for (size_t i = 0; i < numEdges; ++i)
  {
    //
    double Ixx, Iyy, Izz, Ixy, Ixz, Iyz;
    mWorld->getSkeleton(1)->getBodyNode(0)->getMomentOfInertia(Ixx, Iyy, Izz,
                                                               Ixy, Ixz, Iyz);

    //
    Eigen::Matrix3d I = Eigen::Matrix3d::Zero();

    //
    I(0, 0) = Ixx;
    I(1, 1) = Iyy;
    I(2, 2) = Izz;

    //
    I(0, 1) = I(1, 0) = Ixy;
    I(0, 2) = I(2, 0) = Ixz;
    I(1, 2) = I(2, 1) = Iyz;

    const double& mass = mWorld->getSkeleton(1)->getBodyNode(0)->getMass();
    const Eigen::Matrix3d& skew = math::makeSkewSymmetric(mCOM - mEdges[i]);

    mIs[i] = I - mass * skew * skew;
  }
}

//==============================================================================
void MyWindow::evalAngles()
{
  mAngles.resize(mN);
  mStartVels.resize(mN);
  mEndVels.resize(mN);

  // The beginning velocity magnitue for the first rolling cycle
  mStartVels[0] = mInitVel.norm();

  //---- rolling on plane

  // Empirical coefficient of kinetic energy dissipation due to collision
  // We used 0.5 in the paper but use 0.1 here...
  // JS: Is this because of inelastic collision?
  double eps = 0.1;

  // rolling on hand
  //mStartVels[0] = 0.05;
  //double p = 0.5;

  for (int i = 0; i < mN; ++i)
  {
    // current geometry index
    const int& curIndex = i % mEdges.size();

    // next geometry index
    const int& nextIndex = (i + 1) % mEdges.size();

    //
    double m = mWorld->getSkeleton(1)->getBodyNode(0)->getMass();

    // Gravity of y-axis component (planar motion)
    double g = abs(mWorld->getGravity()[1]);

    //---- calculate the minimum theta to roll

    // linear kinetic energy
    double linKe = 0.5 * m * mStartVels[i] * mStartVels[i];

    // rotational kinetic energy
    double rotKe = 0.5
                 * mIs[curIndex](2, 2)  // Izz
                 * mStartVels[i]
                 * mStartVels[i]
                 / (mRs[curIndex] * mRs[curIndex]);

    // the initial angle
    if (i == 0)
    {
      // Equation (8) in the paper
      mAngles[i] = asin(1 - (linKe + rotKe) / (m * g * mRs[curIndex])) - mAlphas[curIndex];

      // TODO(JS): Why 0.1 is added?
      mAngles[i] = mAngles[i] + 0.1;
    }

    if (i == mN - 1)
      break;

    // calculate the kinetic energy when COM is at the highest point
    double KE = linKe + rotKe - m * g * mRs[curIndex] * (1 - sin(mAlphas[curIndex] + mAngles[i]));
    double C = (-eps * KE - eps * m * g * mRs[curIndex] + m * g * mRs[nextIndex]) / (m * g);
    double A = mRs[nextIndex] * cos(mAlphas[nextIndex]) + eps * mRs[curIndex] * cos(mPhis[curIndex]);
    double B = mRs[nextIndex] * sin(mAlphas[nextIndex]) - eps * mRs[curIndex] * sin(mPhis[curIndex]);
    double k = sqrt(A * A + B * B);
    double theta = acos(A / k);

    if (abs(sin(theta) - (B / k)) > 0.001)
      theta = -theta;

    mAngles[i + 1] = asin(C / k) - theta;

    // rotate on plane
    if (i + 1 < mN - 1)
      mAngles[i + 1] += 0.1;
    else
      mAngles[i + 1] -= 0.1;

    // rotate on palm
    /*
      if (i+1 >= mN-1) {
        mAngles[i+1] -= 0.5;
      }
      */

    //---- calculate the end velocity before collision
    // potential energy
    double PE = m
                * g
                * mRs[curIndex]
                * (sin(mAlphas[curIndex] + mAngles[i])
                   - sin(mPhis[curIndex] - mAngles[i + 1]));
    mEndVels[i] = sqrt((PE + linKe + rotKe) / (0.5 * m + 0.5 * mIs[curIndex](2, 2)
                                           /(mRs[curIndex] * mRs[curIndex])));

    const double& currIzz = mIs[curIndex ](2, 2);
    const double& nextIzz = mIs[nextIndex](2, 2);
    const double& currPivotLength2 = mRs[curIndex ] * mRs[curIndex];
    const double& nextPivotLength2 = mRs[nextIndex] * mRs[nextIndex];
    const double& currE = 0.5 * m + 0.5 * currIzz / currPivotLength2;
    const double& nextE = 0.5 * m + 0.5 * nextIzz / nextPivotLength2;

    mStartVels[i + 1] = sqrt(eps * mEndVels[i] * mEndVels[i] * currE / nextE);
  }

  for (int i = 0; i < mN; ++i)
    std::cout << mAngles[i] << std::endl;
}

//==============================================================================
void MyWindow::setGroundAngle(double _angle, const Eigen::Vector3d& _axis)
{
  Eigen::Isometry3d preWorldTransformation = mWorld->getSkeleton(0)->getBodyNode(0)->getTransform();
  Eigen::Matrix3d preWorldOrientation = preWorldTransformation.linear();
  Eigen::Vector3d preWorldTranslation = preWorldTransformation.translation();
  VectorXd pose = mWorld->getSkeleton(0)->getPositions();
  pose[2] = _angle;
  mWorld->getSkeleton(0)->setPositions(pose);
  mWorld->getSkeleton(0)->computeForwardKinematics(true, true, true);
  Eigen::Isometry3d curWorldTransformation = mWorld->getSkeleton(0)->getBodyNode(0)->getTransform();
  Eigen::Matrix3d curWorldOrientation = curWorldTransformation.linear();
  Eigen::Vector3d curWorldTranslation = preWorldOrientation * _axis
                                        + preWorldTranslation
                                        - curWorldOrientation * _axis;
  Isometry3d cubeT = Isometry3d::Identity();
  cubeT.translation() = curWorldTranslation;
  cubeT.linear() = math::eulerXYZToMatrix(Vector3d(0.0, 0.0, _angle));
  mWorld->getSkeleton(0)->setPositions(math::logMap(cubeT));
  mWorld->getSkeleton(0)->computeForwardKinematics(true, true, true);
}

//==============================================================================
double MyWindow::getAngle(int _index)
{
  return mAngles[_index];
}

//==============================================================================
int MyWindow::evalContactEdge()
{
  //
  std::vector<bool> inContactEdges;

  //
  inContactEdges.resize(mEdges.size());

  //
  for (size_t i = 0; i < mEdges.size(); ++i)
    inContactEdges[i] = false;

  for (size_t i = 0; i < mWorld->getConstraintSolver()->getCollisionDetector()->getNumContacts(); ++i)
  {
    //
    Eigen::Vector3d contactPos
        = mWorld->getConstraintSolver()->getCollisionDetector()->getContact(i).point;

    //
    Vector3d contactLocalPos
        = mWorld->getSkeleton(1)->getBodyNode(0)->getTransform().inverse()
          * contactPos;

    //
    for (size_t j = 0; j < mEdges.size(); ++j)
    {
      if ((contactLocalPos.head(2)-mEdges[j].head(2)).norm() < 0.001)
        inContactEdges[j] = true;
    }
  }

  //
  int contactEdgeIndex = -1;
  for (size_t i = 0; i < mEdges.size(); ++i)
  {
    //
    if (inContactEdges[i] == true)
    {
      //
      for (size_t j = 0; j < mEdges.size(); ++j)
      {
        // there are more than one contact edges
        if (j != i && inContactEdges[j])
          return -1;
      }

      //
      contactEdgeIndex = i;
    }
  }

  return contactEdgeIndex;
}
