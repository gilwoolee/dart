/*
 * Copyright (c) 2014, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Jeongseok Lee <jslee02@gmail.com>
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

#include <iostream>

#include <Eigen/Dense>
#include <gtest/gtest.h>

#include "TestHelpers.h"

#include "dart/common/Console.h"
#include "dart/math/Geometry.h"
#include "dart/math/Helpers.h"
#include "dart/dynamics/BodyNode.h"
#include "dart/dynamics/Skeleton.h"
#include "dart/simulation/World.h"
#include "dart/utils/SkelParser.h"
#include "dart/utils/Paths.h"

using namespace Eigen;

using namespace dart;
using namespace math;
using namespace dynamics;

Eigen::Isometry3d integrate(Eigen::Vector6d& _V, double _dt)
{
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();

  T.translation() = _dt * _V.tail<3>();
  T.linear()      = math::expMapRot(_dt * _V.head<3>());

  return T;
}

Isometry3d expMap2(const Eigen::Vector6d& _S)
{
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();

  double theta = _S.head<3>().norm();
  Eigen::Vector3d w = _S.head<3>() / theta;
  Eigen::Matrix3d skew = math::makeSkewSymmetric(w.normalized());

  T.linear() = Eigen::Matrix3d::Identity()
               + std::sin(theta) * skew
               + (1.0 - std::cos(theta)) * skew * skew;

  return T;
}

class DiscreteBodyNode
{
public:
  DiscreteBodyNode()
  {
    mG = Eigen::Matrix6d::Identity();
    mG(0,0) = 0.80833333;
    mG(1,1) = 0.68333333;
    mG(2,2) = 0.14166667;
    mG(3,3) = 10.0;
    mG(4,4) = 10.0;
    mG(5,5) = 10.0;
    mW = Eigen::Isometry3d::Identity();
    mV << 1.5e-1, 0.0, 0.0, 0.1, 0.4, 0.9;
//    mV << 0.0, 0.0, 0.0, 0.1, 0.4, 0.9;
    mA.setZero();
    mTimeStep = 0.001;
  }

  virtual ~DiscreteBodyNode() {}

  void updateVelocity()
  {
    Eigen::Vector3d pWorldOld = mW.translation();

//    mV = mG.inverse() * math::dAdT(expMap2(/*mTimeStep * */mV), mG * mV);
    mW = mW * integrate(mV, 0.001);
    mV = mG.inverse() * math::dAdT(integrate(mV, 0.001), mG * mV);

    Eigen::Vector3d vWorld = mW.linear() * mV.tail<3>();

    Eigen::Vector3d pWorld = mW.translation();
    Eigen::Vector3d pWorld2 = mW.linear().inverse() * mW.translation();
    int a = 10;
  }

  void updateVelocity2()
  {
    Eigen::Vector3d pWorldOld = mW.translation();

    mW = mW * integrate(mV, 0.001);
//    mV = mG.inverse() * math::dAdT(expMap2(/*mTimeStep * */mV), mG * mV);
    //mV = mG.inverse() * math::dAdT(integrate(mV, 0.001), mG * mV);

    mV = mV + 0.001 * mG.inverse() * math::dad(mV, mG * mV);

    Eigen::Vector3d vWorld = mW.linear() * mV.tail<3>();

    Eigen::Vector3d pWorld = mW.translation();
    Eigen::Vector3d pWorld2 = mW.linear().inverse() * mW.translation();
    int a = 10;
  }

  void updateVelocity3()
  {
    Eigen::Vector3d pWorldOld = mW.translation();

//    mW = mW * integrate(mV, 0.001);

    mW.translation() = mW.translation() + 0.001 * mV.tail<3>();

    Eigen::Matrix3d I = mG.topLeftCorner<3,3>();
    Eigen::Vector3d w = mV.head<3>();
    //mV.head<3>() = mV.head<3>() - 0.001 * I * w.cross(I * w);

//    mV = mG.inverse() * math::dAdT(expMap2(/*mTimeStep * */mV), mG * mV);
    //mV = mG.inverse() * math::dAdT(integrate(mV, 0.001), mG * mV);

//    mV = mV + 0.001 * mG.inverse() * math::dad(mV, mG * mV);

    Eigen::Vector3d vWorld = mW.linear() * mV.tail<3>();

    Eigen::Vector3d pWorld = mW.translation();
    Eigen::Vector3d pWorld2 = mW.linear().inverse() * mW.translation();
    int a = 10;
  }

  void initCircular()
  {
    mG = Eigen::Matrix6d::Identity();
    mW = Eigen::Isometry3d::Identity();
    mV << 0.0, -DART_PI/5.0, 0.0, 0.0, 0.0, DART_PI/5.0;
//    mV << 0.0, 0.0, 0.0, 0.1, 0.4, 0.9;
    mA.setZero();
    mTimeStep = 0.001;
  }

  void updateVelocityCircular()
  {
    Eigen::Vector3d pWorldOld = mW.translation();

//    mV = mG.inverse() * math::dAdT(expMap2(/*mTimeStep * */mV), mG * mV);
    mV = mG.inverse() * math::dAdT(integrate(mV, 0.001), mG * mV);
    mW = mW * integrate(mV, 0.001);

    Eigen::Vector3d vWorld = mW.linear() * mV.tail<3>();

    Eigen::Vector3d pWorld = mW.translation();
    Eigen::Vector3d pWorld2 = mW.linear().inverse() * mW.translation();
    int a = 10;
  }

  Eigen::Matrix6d mG;

  Eigen::Isometry3d mW;

  Eigen::Vector6d mV;

  Eigen::Vector6d mA;

  double mTimeStep;

protected:

private:
};

//==============================================================================
void setPosition(Skeleton* _skel, const Vector3d& _pos);
void setLinearVelocity(Skeleton* _skel, const Vector3d& _linVel);
void setAngularVelocity(Skeleton* _skel, const Vector3d& _angVel);

//==============================================================================
void setPosition(Skeleton* _skel, const Vector3d& _pos)
{
  // Assume that the root joint is FreeJoint
  FreeJoint* freeJoint =
      dynamic_cast<FreeJoint*>(_skel->getBodyNode(0)->getParentJoint());
  assert(freeJoint);

  const Vector6d& S = freeJoint->getPositions();
  Isometry3d T = expMap(S);
  T.translation() = _pos;
  freeJoint->setPositions(logMap(T));
}

//==============================================================================
class AccuracyTest : public ::testing::Test
{
public:
  // Spawn a single box and record accuracy for momentum and energy conservation
  void Boxes(double _dt,
             int _iterations,
             int _boxCount,
             bool _gravity,
             bool _collision,
             bool _linear);

  // Spawn a single box and record accuracy for momentum and energy conservation
  void Boxes2(double _dt,
             int _iterations,
             int _boxCount,
             bool _gravity,
             bool _collision,
             bool _linear);
};

//==============================================================================
void AccuracyTest::Boxes(double _dt,
                         int _iterations,
                         int _boxCount,
                         bool _gravity,
                         bool _collision,
                         bool _linear)
{
  World* world = new World();

  // get gravity value
  if (_gravity)
    world->setGravity(Vector3d(0.0, 0.0, -9.81));
  else
    world->setGravity(Vector3d::Zero());

  Eigen::Vector3d g = world->getGravity();

  // Box size
  const double dx = 0.1;
  const double dy = 0.4;
  const double dz = 0.9;
  const double mass = 10.0;
  // inertia matrix, recompute if the above change
  const double Ixx = 0.80833333;
  const double Iyy = 0.68333333;
  const double Izz = 0.14166667;
  const Matrix3d I0 = Vector3d(Ixx, Iyy, Izz).asDiagonal();

  // Create box with inertia based on box of uniform density
  Skeleton* boxSkel = new Skeleton();
  BodyNode* box = new BodyNode();
  BoxShape* boxVizShape = new BoxShape(Vector3d(dx, dy, dz));
  BoxShape* boxColShape = new BoxShape(Vector3d(dx, dy, dz));
  FreeJoint* boxJoint = new FreeJoint();
  box->addVisualizationShape(boxVizShape);
  box->addCollisionShape(boxColShape);
  box->setParentJoint(boxJoint);
  box->setMomentOfInertia(Ixx, Iyy, Izz);
  box->setMass(mass);
  boxSkel->addBodyNode(box);
  world->addSkeleton(boxSkel);

  // initial linear velocity in global frame
  const Vector3d v0(0.1, 0.4, 0.9);

  // initial angular velocity in global frame
  Vector3d w0;

  // initial energy value
  double E0;

  if (_linear)
  {
    // Use angular velocity with one non-zero component
    // to ensure linear angular trajectory
    w0 << 1.5e-1, 0.0, 0.0;
    E0 = 4.9090937462499999;
  }
  else
  {
    // Since Ixx > Iyy > Izz,
    // angular velocity with large y component
    // will cause gyroscopic tumbling
    w0 << 1e-3, 1.5e0, 1.5e-2;
    E0 = 5.668765966704;
  }

  for (int i = 0; i < _boxCount; ++i)
  {
    Skeleton* skel = world->getSkeleton(i);
    BodyNode* bodyNode = skel->getBodyNode(0);

    // give models unique names
    skel->setName("model");

    // give models unique positions
    setPosition(skel, Vector3d(dz*2*i, 0.0, 0.0));

    // Set initial conditions

    skel->setVelocity(0, w0[0]);
    skel->setVelocity(1, w0[1]);
    skel->setVelocity(2, w0[2]);

    skel->setVelocity(3, v0[0]);
    skel->setVelocity(4, v0[1]);
    skel->setVelocity(5, v0[2]);

    skel->computeForwardKinematics(true, true, false);

    Vector3d linVel = bodyNode->getWorldLinearVelocity();
    Vector3d angVel = bodyNode->getWorldAngularVelocity();
    Vector3d wBody = box->getBodyAngularVelocity();

    double mag = linVel.norm();

    ASSERT_EQ(linVel, v0);
    ASSERT_EQ(angVel, w0);

//    ASSERT_EQ(v0, bodyNode->GetWorldCoGLinearVel());
//    ASSERT_EQ(w0, bodyNode->GetWorldAngularVel());
//    ASSERT_EQ(I0, bodyNode->GetInertial()->GetMOI());
//    ASSERT_NEAR(link->GetWorldEnergy(), E0, 1e-6);
  }

  const double simDuration = 10.0;
  const double t0 = 0.0;
  // initial linear position in global frame
  Vector3d p0 = box->getTransform().translation();
  int steps = ceil(simDuration / _dt);

  DiscreteBodyNode dbody;

  for (int i = 0; i < steps; ++i)
  {
    world->step();
    dbody.updateVelocity();

    // current time
    double t = world->getTime() - t0;

    // linear velocity error
    Vector3d v = box->getWorldLinearVelocity();
    Vector3d wBody = box->getBodyAngularVelocity();
    Vector3d wWorld = box->getWorldAngularVelocity();
    Vector3d linVelErr = v - (v0 + g*t);
//    linearVelocityError.InsertData(v - (v0 + g*t));
//    std::cout << "linVelErr: " << linVelErr.transpose() << std::endl;

    double mag2 = v.norm();
    double mag3 = box->getBodyVelocity().tail<3>().norm();

    // linear position error
    Vector3d p = box->getTransform().translation();
    Vector3d p2 = dbody.mW.translation();
    Vector3d v2World = dbody.mV.tail<3>();
    Vector3d posErr = p - (p0 + v0 * t + 0.5*g*t*t);
    Vector3d posErr2 = p2 - (p0 + v0 * t + 0.5*g*t*t);
//    linearPositionError.InsertData(p - (p0 + v0 * t + 0.5*g*t*t));
    std::cout << "posErr: " << posErr.transpose() << std::endl;
//    std::cout << "posErr2: " << posErr2.transpose() << std::endl;

    // angular momentum error
//    Vector3d H = link->GetWorldInertiaMatrix()*link->GetWorldAngularVel();
//    angularMomentumError.InsertData((H - H0) / H0mag);

    // energy error
//    energyError.InsertData((link->GetWorldEnergy() - E0) / E0);
  }

  delete world;
}

//==============================================================================
void AccuracyTest::Boxes2(double _dt,
                         int _iterations,
                         int _boxCount,
                         bool _gravity,
                         bool _collision,
                         bool _linear)
{
  World* world = new World();

  // get gravity value
  if (_gravity)
    world->setGravity(Vector3d(0.0, 0.0, -9.81));
  else
    world->setGravity(Vector3d::Zero());

  Eigen::Vector3d g = world->getGravity();

  // Box size
  const double dx = 0.1;
  const double dy = 0.4;
  const double dz = 0.9;
  const double mass = 10.0;
  // inertia matrix, recompute if the above change
  const double Ixx = 0.80833333;
  const double Iyy = 0.68333333;
  const double Izz = 0.14166667;
  const Matrix3d I0 = Vector3d(Ixx, Iyy, Izz).asDiagonal();

  // Create box with inertia based on box of uniform density
  Skeleton* boxSkel = new Skeleton();
  BodyNode* box = new BodyNode();
  BoxShape* boxVizShape = new BoxShape(Vector3d(dx, dy, dz));
  BoxShape* boxColShape = new BoxShape(Vector3d(dx, dy, dz));
  FreeJoint* boxJoint = new FreeJoint();
  box->addVisualizationShape(boxVizShape);
  box->addCollisionShape(boxColShape);
  box->setParentJoint(boxJoint);
  box->setMomentOfInertia(Ixx, Iyy, Izz);
  box->setMass(mass);
  boxSkel->addBodyNode(box);
  world->addSkeleton(boxSkel);

  // initial linear velocity in global frame
  const Vector3d v0(0.0, -1.0, 0.0);

  // initial angular velocity in global frame
  Vector3d w0(0.0, 0.0, DART_PI/5.0);

  // initial energy value
  double E0;


  for (int i = 0; i < 1; ++i)
  {
    Skeleton* skel = world->getSkeleton(i);
    BodyNode* bodyNode = skel->getBodyNode(0);

    // give models unique names
    skel->setName("model");

    // give models unique positions
    setPosition(skel, Vector3d(1.0, 0.0, 0.0));

    // Set initial conditions

    skel->setVelocity(0, w0[0]);
    skel->setVelocity(1, w0[1]);
    skel->setVelocity(2, w0[2]);

    skel->setVelocity(3, v0[0]);
    skel->setVelocity(4, v0[1]);
    skel->setVelocity(5, v0[2]);

    skel->computeForwardKinematics(true, true, false);

    Vector3d linVel = bodyNode->getWorldLinearVelocity();
    Vector3d angVel = bodyNode->getWorldAngularVelocity();
    Vector3d wBody = box->getBodyAngularVelocity();

    double mag = linVel.norm();

    ASSERT_EQ(linVel, v0);
    ASSERT_EQ(angVel, w0);

//    ASSERT_EQ(v0, bodyNode->GetWorldCoGLinearVel());
//    ASSERT_EQ(w0, bodyNode->GetWorldAngularVel());
//    ASSERT_EQ(I0, bodyNode->GetInertial()->GetMOI());
//    ASSERT_NEAR(link->GetWorldEnergy(), E0, 1e-6);
  }

  const double simDuration = 10.0;
  const double t0 = 0.0;
  // initial linear position in global frame
  Vector3d p0 = box->getTransform().translation();
  int steps = ceil(simDuration / _dt);

  DiscreteBodyNode dbody;
  dbody.initCircular();

  for (int i = 0; i < steps; ++i)
  {
    double t_prev = world->getTime() - t0;

    box->setExtForce(
          (DART_PI/5.0)
          *(DART_PI/5.0)
          *Vector3d(cos(DART_PI/5.0*t_prev), -sin(DART_PI/5.0*t_prev), 0.0));
    world->step();
//    dbody.updateVelocity();
    dbody.updateVelocityCircular();

    // current time
    double t = world->getTime() - t0;

    // linear velocity error
    Vector3d v = box->getWorldLinearVelocity();
    Vector3d wBody = box->getBodyAngularVelocity();
    Vector3d wWorld = box->getWorldAngularVelocity();
    Vector3d linVelErr = v - (v0 + g*t);
//    linearVelocityError.InsertData(v - (v0 + g*t));
//    std::cout << "linVelErr: " << linVelErr.transpose() << std::endl;

    double mag2 = v.norm();
    double mag3 = box->getBodyVelocity().tail<3>().norm();

    // linear position error
    Vector3d p = box->getTransform().translation();
    Vector3d p2 = dbody.mW.translation();
    Vector3d v2World = dbody.mV.tail<3>();
    Vector3d exactP(cos(DART_PI/5.0*t), -sin(DART_PI/5.0*t), 0.0);
    Vector3d posErr = p - exactP;
    Vector3d posErr2 = p2 - exactP;
//    linearPositionError.InsertData(p - (p0 + v0 * t + 0.5*g*t*t));
    std::cout << "posErr: " << posErr.transpose() << std::endl;
    std::cout << "posErr2: " << posErr2.transpose() << std::endl;

    // angular momentum error
//    Vector3d H = link->GetWorldInertiaMatrix()*link->GetWorldAngularVel();
//    angularMomentumError.InsertData((H - H0) / H0mag);

    // energy error
//    energyError.InsertData((link->GetWorldEnergy() - E0) / E0);
  }

  delete world;
}

//==============================================================================
TEST_F(AccuracyTest, testBoxes)
{
  Boxes(0.001, 50, 1, false, false, true);
//  Boxes2(0.001, 50, 1, false, false, true);
}

//==============================================================================
int main(int argc, char* argv[])
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}


