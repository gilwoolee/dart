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
 *   * This code incorporates portions of Open Dynamics Engine
 *     (Copyright (c) 2001-2004, Russell L. Smith. All rights
 *     reserved.) and portions of FCL (Copyright (c) 2011, Willow
 *     Garage, Inc. All rights reserved.), which were released under
 *     the same BSD license as below
 *
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

#include <iostream>

#include "dart/dynamics/SoftBodyNode.h"
#include "dart/dynamics/Skeleton.h"
#include "dart/dynamics/PointMass.h"
#include "dart/gui/GLFuncs.h"
#include "dart/simulation/World.h"

#define FORCE_ON_RIGIDBODY (25.0)
#define FORCE_ON_VERTEX (1.00)
#define DESIRED_VELOCITY (25)

MyWindow::MyWindow()
  : SoftSimWindow()
{
  mForceOnRigidBody = Eigen::Vector3d::Zero();
  mForceOnVertex = Eigen::Vector3d::Zero();
  mImpulseDuration = 0.0;
  mDesiredQ = 0.0;
  mDesiredIzz = 0.05;
  mDesiredW = 0.43;
}

MyWindow::~MyWindow()
{
}

void MyWindow::timeStepping()
{
//  dart::dynamics::Skeleton* Skeleton =
//      static_cast<dart::dynamics::Skeleton*>(mWorld->getSkeleton(1));
//  dart::dynamics::SoftBodyNode* softBodyNode = Skeleton->getSoftBodyNode(0);
//  softBodyNode->addExtForce(mForceOnRigidBody);

  double q   = mWorld->getSkeleton(0)->getJoint(1)->getPosition(0);
  double dq  = mWorld->getSkeleton(0)->getJoint(1)->getVelocity(0);
  double Izz = mWorld->getSkeleton(0)->getTotalSpatialInertiaTensorRoot()(2,2);

  double w = mWorld->getSkeleton(0)->getBodyNode(0)->getWorldAngularVelocity()[2];
  double H = Izz * w;

  mDesiredIzz = mH / mDesiredW;
  mDesiredQ   = mWorld->getSkeleton(0)->setDesiredIzz(mDesiredIzz);
  double tau  = -20.0*(q - mDesiredQ) - 1.0*dq;

  mWorld->getSkeleton(0)->getJoint(1)->setForce(0, tau);

  std::cout << "DesiredW: " << mDesiredW << ", q: " << q << ", w: " << w
            << ", I: " << Izz << ", H: " << H << std::endl;

  mWorld->step();

  // for perturbation test
  mImpulseDuration--;
  if (mImpulseDuration <= 0)
  {
    mImpulseDuration = 0;
    mForceOnRigidBody.setZero();
  }

  mForceOnVertex /= 2.0;
}

void MyWindow::drawSkels()
{
  glEnable(GL_LIGHTING);
  //glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
  Eigen::Vector4d color;
  color << 0.5, 0.8, 0.6, 1.0;
  mWorld->getSkeleton(0)->draw(mRI, color, false);

  // draw arrow
  if (mImpulseDuration > 0)
  {
    dart::dynamics::Skeleton* Skeleton =
        static_cast<dart::dynamics::Skeleton*>(mWorld->getSkeleton(1));
    dart::dynamics::SoftBodyNode* softBodyNode = Skeleton->getSoftBodyNode(0);
    softBodyNode->addExtForce(mForceOnRigidBody);
    Eigen::Vector3d poa
        = softBodyNode->getTransform() * Eigen::Vector3d(0.0, 0.0, 0.0);
    Eigen::Vector3d start = poa - mForceOnRigidBody / 25.0;
    double len = mForceOnRigidBody.norm() / 25.0;
    dart::gui::drawArrow3D(start, mForceOnRigidBody, len, 0.025, 0.05);
  }

  SimWindow::drawSkels();
}

void MyWindow::init()
{
//  double q   = mWorld->getSkeleton(0)->getJoint(1)->getPosition(0);
//  double dq  = mWorld->getSkeleton(0)->getJoint(1)->getVelocity(0);
  double Izz = mWorld->getSkeleton(0)->getTotalSpatialInertiaTensorRoot()(2,2);

  double w = mWorld->getSkeleton(0)->getBodyNode(0)->getWorldAngularVelocity()[2];
  mH = Izz * w;
}

void MyWindow::keyboard(unsigned char key, int x, int y)
{
  switch (key)
  {
  case ' ':  // use space key to play or stop the motion
    mSimulating = !mSimulating;
    if (mSimulating)
    {
      mPlay = false;
      glutTimerFunc(mDisplayTimeout, refreshTimer, 0);
    }
    break;
  case 'p':  // playBack
    mPlay = !mPlay;
    if (mPlay)
      {
        mSimulating = false;
        glutTimerFunc(mDisplayTimeout, refreshTimer, 0);
      }
      break;
    case '[':  // step backward
      if (!mSimulating)
      {
        mPlayFrame--;
        if (mPlayFrame < 0)
          mPlayFrame = 0;
        glutPostRedisplay();
      }
      break;
    case ']':  // step forwardward
      if (!mSimulating)
      {
        mPlayFrame++;
        if (mPlayFrame >= mWorld->getRecording()->getNumFrames())
          mPlayFrame = 0;
        glutPostRedisplay();
      }
      break;
    case 'v':  // show or hide markers
      mShowMarkers = !mShowMarkers;
      break;
    case 'n':
      mShowPointMasses = !mShowPointMasses;
      break;
    case 'm':
      mShowMeshs = !mShowMeshs;
      break;
    case 'i':
    case 'I':
      if (mWorld->getSkeleton(0)->mShowTotalInertia)
        mWorld->getSkeleton(0)->mShowTotalInertia = false;
      else
        mWorld->getSkeleton(0)->mShowTotalInertia = true;
      break;
    case 'q':
    case 'Q':
    {
      mDesiredW -= 0.01;
      break;
    }
    case 'w':
    case 'W':
    {
      mDesiredW += 0.01;
      break;
    }
    case '1':  // upper right force
      mForceOnRigidBody[0] = -FORCE_ON_RIGIDBODY;
      mImpulseDuration = 100;
      break;
    case '2':  // upper right force
      mForceOnRigidBody[0] = FORCE_ON_RIGIDBODY;
      mImpulseDuration = 100;
      break;
    case '3':  // upper right force
      mForceOnRigidBody[1] = -FORCE_ON_RIGIDBODY;
      mImpulseDuration = 100;
      break;
    case '4':  // upper right force
      mForceOnRigidBody[1] = FORCE_ON_RIGIDBODY;
      mImpulseDuration = 100;
      break;
    case '5':  // upper right force
      mForceOnRigidBody[2] = -FORCE_ON_RIGIDBODY;
      mImpulseDuration = 100;
      break;
    case '6':  // upper right force
      mForceOnRigidBody[2] = FORCE_ON_RIGIDBODY;
      mImpulseDuration = 100;
      break;
//    case 'q':
//    case 'Q':
//      mController->setTorqueRotorX(-DESIRED_VELOCITY);
//      mController->setTorqueRotorY(0);
//      mController->setTorqueRotorZ(0);
//      break;
//    case 'w':
//    case 'W':
//      mController->setTorqueRotorX(DESIRED_VELOCITY);
//      mController->setTorqueRotorY(0);
//      mController->setTorqueRotorZ(0);
//      break;
//    case 'a':
//    case 'A':
//      mController->setTorqueRotorX(0);
//      mController->setTorqueRotorY(-DESIRED_VELOCITY);
//      mController->setTorqueRotorZ(0);
//      break;
//    case 's':
//    case 'S':
//      mController->setTorqueRotorX(0);
//      mController->setTorqueRotorY(DESIRED_VELOCITY);
//      mController->setTorqueRotorZ(0);
//      break;
//    case 'z':
//    case 'Z':
//      mController->setTorqueRotorX(0);
//      mController->setTorqueRotorY(0);
//      mController->setTorqueRotorZ(-DESIRED_VELOCITY);
//      break;
//    case 'x':
//    case 'X':
//      mController->setTorqueRotorX(0);
//      mController->setTorqueRotorY(0);
//      mController->setTorqueRotorZ(DESIRED_VELOCITY);
//      break;
    default:
      Win3D::keyboard(key, x, y);
  }
  glutPostRedisplay();
}

//void MyWindow::setController(Controller* _controller)
//{
//  mController = _controller;
//}

