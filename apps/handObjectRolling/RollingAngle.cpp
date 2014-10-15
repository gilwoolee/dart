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

#include "apps/handObjectRolling//RollingAngle.h"

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
RollingAngle::RollingAngle()
{
  mForce = Eigen::Vector3d::Zero();
  mRollNum = 0;
}

//==============================================================================
RollingAngle::RollingAngle(BodyNode* _bodyNode, const Vector3d& _gravity)
  : mBodyNode(_bodyNode),
    mGravity(_gravity)
{
  assert(_bodyNode != NULL);

  mForce = Eigen::Vector3d::Zero();
  mRollNum = 0;
}

//==============================================================================
RollingAngle::~RollingAngle()
{
}

//==============================================================================
void RollingAngle::setBodyNode(BodyNode* _body)
{
  assert(_body);
  mBodyNode = _body;
}

//==============================================================================
void RollingAngle::setGravity(const Vector3d& _gravity)
{
  mGravity = _gravity;
}

//==============================================================================
void RollingAngle::updateAngles()
{
  assert(mBodyNode);

  evalN();
  evalGeometry();
  evalInertia();
  setInitVel(Eigen::Vector3d::Zero());
  evalAngles();
}

//==============================================================================
int RollingAngle::getNumAngles() const
{
  return mN;
}

//==============================================================================
void RollingAngle::setInitVel(const Vector3d& _vel)
{
  mInitVel = _vel;
}

//==============================================================================
void RollingAngle::evalN()
{
  mN = 3;
}

//==============================================================================
void RollingAngle::evalGeometry()
{
  // the order is determined by the order of pivoting
  size_t numEdges = 4;
  mEdges.resize(numEdges);

  /// ASSUME THAT WE HAVE ONE BOX COLLISION SHAPE
  BoxShape* boxShape
      = (BoxShape*)mBodyNode->getCollisionShape(0);
  assert(boxShape);

  Eigen::Vector3d halfSize = boxShape->getSize()*0.5;

  std::cout << "halfSize: " << halfSize.transpose() << std::endl;

  mEdges[0] = Vector3d(0.0, -halfSize[1],  halfSize[2]);
  mEdges[1] = Vector3d(0.0,  halfSize[1],  halfSize[2]);
  mEdges[2] = Vector3d(0.0,  halfSize[1], -halfSize[2]);
  mEdges[3] = Vector3d(0.0, -halfSize[1], -halfSize[2]);

  mCOM = mBodyNode->getLocalCOM();
  std::cout << "COM: " << mCOM.transpose() << std::endl;

  // can define the faces, and calculate the alpha and phi
  mAlphas.resize(numEdges);
  mPhis.resize(numEdges);
  mRs.resize(numEdges);

  // Compute distance between the pivoting egde and the COM
  for (size_t i = 0; i < numEdges; ++i)
  {
    mRs[i] = (mEdges[i] - mCOM).norm();
//    std::cout << "mRs: " << mRs[i] << std::endl;
  }

  // Compute the angle between COM and pivoting edge with the previous face, and
  // the angle between COM and pivoting edge with the next face
//  for (size_t i = 0; i < numEdges; ++i)
//  {
//    mAlphas[i] = mPhis[i] = DART_PI / 4.0;

    mAlphas[0] = std::acos(std::fabs(mEdges[0][2] - mCOM[2]) / mRs[0]);
    mAlphas[1] = std::acos(std::fabs(mEdges[1][1] - mCOM[1]) / mRs[1]);
    mAlphas[2] = std::acos(std::fabs(mEdges[2][2] - mCOM[2]) / mRs[2]);
    mAlphas[3] = std::acos(std::fabs(mEdges[3][1] - mCOM[1]) / mRs[3]);
    mPhis[0] = DART_PI/2 - mAlphas[0];
    mPhis[1] = DART_PI/2 - mAlphas[1];
    mPhis[2] = DART_PI/2 - mAlphas[2];
    mPhis[3] = DART_PI/2 - mAlphas[3];

    for (size_t i = 0; i < numEdges; ++i)
    {
      std::cout << "mAlphas: " << mAlphas[i] << std::endl;
      std::cout << "mPhis: " << mPhis[i] << std::endl;
    }
//  }
}

//==============================================================================
void RollingAngle::evalInertia()
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
    mBodyNode->getMomentOfInertia(Ixx, Iyy, Izz, Ixy, Ixz, Iyz);

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

    const double& mass = mBodyNode->getMass();
    const Eigen::Matrix3d& skew = math::makeSkewSymmetric(mCOM - mEdges[i]);

    mIs[i] = I - mass * skew * skew;
  }
}

//==============================================================================
void RollingAngle::evalAngles()
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
    double m = mBodyNode->getMass();

    // Gravity of y-axis component (planar motion)
    double g = abs(mGravity[1]);

    //---- calculate the minimum theta to roll

    // linear kinetic energy
    double linKe = 0.5 * m * mStartVels[i] * mStartVels[i];

    // rotational kinetic energy
    double rotKe = 0.5
                 * mIs[curIndex](0, 0)  // Ixx
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
//    if (i + 1 < mN - 1)
//      mAngles[i + 1] += 0.1;
//    else
//      mAngles[i + 1] -= 0.1;

    // rotate on palm
    if (i + 1 >= mN - 1)
      mAngles[i + 1] -= 0.5;

//    mAngles[1] -= 0.1;

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
double RollingAngle::getAngle(int _index)
{
  return mAngles[_index];
}

//==============================================================================
const std::vector<Eigen::Vector3d> RollingAngle::getEdges() const
{
  return mEdges;
}

double RollingAngle::getHalfDepth() const
{
  return mEdges[0][0]/2;
}
