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
void MyWindow::timeStepping()
{
  mWorld->step();

  int contactEdgeIndex = evalContactEdge();
  if (contactEdgeIndex == mRollNum%mEdges.size())
  {
    // (Original Code) Eigen::Vector3d contactPos = mWorld->getSkeleton(1)->getBodyNode(0)->evalWorldPos(mEdges[contactEdgeIndex]); // contact edge position in world coordinate
    Eigen::Vector3d contactPos = mWorld->getSkeleton(1)->getBodyNode(0)->getTransform() * mEdges[contactEdgeIndex]; // contact edge position in world coordinate

    //
    Vector3d contactLocalPos
        = mWorld->getSkeleton(0)->getBodyNode(0)->getTransform().inverse()
          * contactPos;

    //
    if (mWorld->getSkeleton(1)->getWorldCOM()(0)-contactPos(0) < -0.0)
    {
      //
      if (mRollNum < mN - 1)
      {
        //
        Eigen::Vector3d liftEdge
            = mWorld->getSkeleton(1)->getBodyNode(0)->getTransform()
              * mEdges[(mRollNum+mEdges.size()-1) % mEdges.size()];

        //
        Eigen::Vector3d dropEdge
            = mWorld->getSkeleton(1)->getBodyNode(0)->getTransform()
              * mEdges[(mRollNum+mEdges.size()+1) % mEdges.size()];

        //
        Eigen::Vector3d contactEdge
            = mWorld->getSkeleton(1)->getBodyNode(0)->getTransform()
              * mEdges[(mRollNum)%mEdges.size()];

        double liftAngle
            = atan((liftEdge(1)-contactEdge(1))/(liftEdge(0)-contactEdge(0)));

        double dropAngle
            = atan((dropEdge(1)-contactEdge(1))/(contactEdge(0)-dropEdge(0)));

        if (liftAngle > mAngles[mRollNum+1] && dropAngle > -mAngles[mRollNum+1])
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
      // 		std::cout << dart_math::xformHom(mWorld->getSkeleton(1)->getNode(0)->getWorldInvTransform(), mEdges[0]).transpose() << std::endl;
      break;
    case '-': // decrease DoF value
      pose(0) = pose(0) - 0.1;
      mWorld->getSkeleton(1)->setPositions(pose);
      mWorld->getSkeleton(1)->computeForwardKinematics(true, true, false);
      std::cout << mWorld->getSkeleton(1)->getBodyNode(0)->getTransform().matrix() << std::endl;
      // 		std::cout << dart_math::xformHom(mWorld->getSkeleton(1)->getNode(0)->getWorldInvTransform(), mEdges[0]).transpose() << std::endl;
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
  int numEdges = 4;
  mEdges.resize(numEdges);
  // the order is determined by the order of pivoting

  mEdges[0] = Eigen::Vector3d(-0.02,-0.02,0.0);
  mEdges[1] = Eigen::Vector3d(-0.02,0.02,0.0);
  mEdges[2] = Eigen::Vector3d(0.02,0.02,0.0);
  mEdges[3] = Eigen::Vector3d(0.02,-0.02,0.0);

  mCOM = Eigen::Vector3d(0.0,0.0,0.0);

  // can define the faces, and calculate the alpha and phi
  mAlphas.resize(numEdges);
  mPhis.resize(numEdges);

  for (int i = 0; i < numEdges; ++i)
    mAlphas[i] = mPhis[i] = 0.785;

  mRs.resize(numEdges);
  for (int i = 0; i < numEdges; ++i)
    mRs[i] = (mEdges[i]-mCOM).norm();
}

//==============================================================================
void MyWindow::evalInertia()
{
  int numEdges = mEdges.size();
  mIs.resize(numEdges);

  //compute local inertia
  double numerator = 0.0;
  double denominator = 0.0;
  for (int i = 0; i < mEdges.size()-1; ++i)
  {
    denominator = denominator + (mEdges[i+1].cross(mEdges[i])).norm();
    numerator = numerator + (mEdges[i+1].cross(mEdges[i])).norm()*(mEdges[i+1].dot(mEdges[i+1])+mEdges[i+1].dot(mEdges[i])+mEdges[i].dot(mEdges[i]));
  }

  //
  double I = mWorld->getSkeleton(1)->getBodyNode(0)->getMass()
             * numerator / denominator / 6.0;

  // the order is determined by the order of pivoting
  for (int i = 0; i < numEdges; ++i)
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

    mIs[i] = I - mWorld->getSkeleton(1)->getBodyNode(0)->getMass()
                 * dart::math::makeSkewSymmetric(mCOM-mEdges[i])
                 * dart::math::makeSkewSymmetric(mCOM-mEdges[i]);
  }
}

//==============================================================================
void MyWindow::evalAngles()
{
  mAngles.resize(mN);
  mStartVels.resize(mN);
  mEndVels.resize(mN);
  mStartVels[0] = mInitVel.norm();
  // rolling on plane
  double p = 0.1;
  // rolling on hand
  //mStartVels[0] = 0.05;
  //double p = 0.5;
  for (int i = 0; i < mN; ++i)
  {
    // current geometry index
    int curIndex = i%mEdges.size();

    // next geometry index
    int nextIndex = (i+1)%mEdges.size();

    //
    double m = mWorld->getSkeleton(1)->getBodyNode(0)->getMass();

    //
    double g = abs(mWorld->getGravity()(1));

    // calculate the minimum theta to roll
    double KEL = 0.5*m*mStartVels[i]*mStartVels[i]; // linear kinetic energy
    double KER = 0.5*mIs[curIndex](2,2)*mStartVels[i]*mStartVels[i]/(mRs[curIndex]*mRs[curIndex]); // rotational kinetic energy

    // the initial angle
    if (i == 0)
    {
      mAngles[i] = asin(1-(KEL+KER)/(m*g*mRs[curIndex]))-mAlphas[curIndex];
      mAngles[i] = mAngles[i] + 0.1;
    }

    if (i == mN-1)
      break;

    // calculate the kinetic energy when COM is at the highest point
    double KE = KEL+KER-m*g*mRs[curIndex]*(1-sin(mAlphas[curIndex]+mAngles[i]));
    double C = (-p*KE-p*m*g*mRs[curIndex]+m*g*mRs[nextIndex])/(m*g);
    double A = mRs[nextIndex]*cos(mAlphas[nextIndex])+p*mRs[curIndex]*cos(mPhis[curIndex]);
    double B = mRs[nextIndex]*sin(mAlphas[nextIndex])-p*mRs[curIndex]*sin(mPhis[curIndex]);
    double k = sqrt(A*A+B*B);
    double theta = 0.0;
    theta = acos(A/k);

    if (abs(sin(theta)-(B/k))>0.001)
      theta = -theta;

    mAngles[i+1] = asin(C/k)-theta;

    // rotate on plane
    if (i+1 < mN-1)
      mAngles[i+1] += 0.1;
    else
      mAngles[i+1] -= 0.1;

    // rotate on palm
    /*
      if (i+1 >= mN-1) {
        mAngles[i+1] -= 0.5;
      }
      */

    // calculate the end velocity before collision
    double PE = m*g*mRs[curIndex]*(sin(mAlphas[curIndex]+mAngles[i])-sin(mPhis[curIndex]-mAngles[i+1])); // potential energy
    mEndVels[i] = sqrt((PE+KEL+KER)/(0.5*m+0.5*mIs[curIndex](2,2)/(mRs[curIndex]*mRs[curIndex])));
                       mStartVels[i+1] = sqrt(p*mEndVels[i]*mEndVels[i]*(0.5*m+0.5*mIs[curIndex](2,2)/(mRs[curIndex]*mRs[curIndex]))/(0.5*m+0.5*mIs[nextIndex](2,2)/(mRs[nextIndex]*mRs[nextIndex])));
  }

  for (int i = 0; i < mN; ++i) {
    std::cout << mAngles[i] << std::endl;
  }
}

//==============================================================================
void MyWindow::setGroundAngle(double _angle, Eigen::Vector3d _axis)
{
  Eigen::Isometry3d preWorldTransformation = mWorld->getSkeleton(0)->getBodyNode(0)->getTransform();
  Eigen::Matrix3d preWorldRotation = preWorldTransformation.linear();
  Eigen::Vector3d preWorldTranslation = preWorldTransformation.translation();
  VectorXd pose = mWorld->getSkeleton(0)->getPositions();
  pose(5) = _angle;
  mWorld->getSkeleton(0)->setPositions(pose);
  mWorld->getSkeleton(0)->computeForwardKinematics(true, true, true);
  Eigen::Isometry3d curWorldTransformation = mWorld->getSkeleton(0)->getBodyNode(0)->getTransform();
  Eigen::Matrix3d curWorldRotation = curWorldTransformation.linear();
  Eigen::Vector3d curWorldTranslation = preWorldRotation * _axis
                                        + preWorldTranslation
                                        - curWorldRotation * _axis;
  pose.head(3) = curWorldTranslation;
  mWorld->getSkeleton(0)->setPositions(pose);
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
  for (int i = 0; i < mEdges.size(); ++i)
    inContactEdges[i] = false;

  for (int i = 0; i < mWorld->getConstraintSolver()->getCollisionDetector()->getNumContacts(); ++i)
  {
    //
    Eigen::Vector3d contactPos
        = mWorld->getConstraintSolver()->getCollisionDetector()->getContact(i).point;

    //
    Vector3d contactLocalPos
        = mWorld->getSkeleton(1)->getBodyNode(0)->getTransform().inverse()
          * contactPos;

    //
    for (int j = 0; j < mEdges.size(); ++j)
    {
      if ((contactLocalPos.head(2)-mEdges[j].head(2)).norm() < 0.001)
        inContactEdges[j] = true;
    }
  }

  //
  int contactEdgeIndex = -1;
  for (int i = 0; i < mEdges.size(); ++i)
  {
    //
    if (inContactEdges[i] == true)
    {
      //
      for (int j = 0; j < mEdges.size(); ++j)
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
