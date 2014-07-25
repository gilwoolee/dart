/*
 * Copyright (c) 2014, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Jeffrey T. Bingham <bingjeff@gmail.com>,
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

#include "apps/inertiaBot/MyWindow.h"

#include <iostream>
#include <fstream>

//#include <unsupported/Eigen/Splines>

#include "dart/common/Console.h"
#include "dart/math/Helpers.h"
#include "dart/simulation/World.h"
#include "dart/dynamics/BodyNode.h"
#include "dart/dynamics/Skeleton.h"
#include "dart/dynamics/FreeJoint.h"
#include "dart/dynamics/BoxShape.h"
#include "dart/collision/CollisionDetector.h"
#include "dart/constraint/ConstraintSolver.h"
#include "dart/gui/GLFuncs.h"
#include "dart/utils/Paths.h"

using namespace std;

using namespace Eigen;
using namespace tinyxml2;

using namespace dart;
//  using namespace math;
using namespace collision;
using namespace constraint;
using namespace dynamics;
using namespace simulation;
using namespace gui;
//using namespace utils;

MyWindow::MyWindow()
  : SimWindow()
{
  mVisibleCollisionShape = false;
  mVisibleInertiaEllipsoid = false;
  mVisibleCOM = true;
  mVisibleAngularMomentum = false;
  mVisibleLinearMomentum = false;
  mSave = false;

  // Initial camera setting
//  mRI->getCamera()->slide();
  mTrans[1] = -000.f;

  mEye[1] = 0.0f;
}

MyWindow::~MyWindow()
{

}

void MyWindow::setController(CompositeController* _controller)
{
  mController = _controller;

  if (mController)
    mController->prestep(mWorld->getTime());
}

void MyWindow::timeStepping()
{
  //    typedef Spline<double,1> Spline2d;

  //    const double pi = std::acos(-1.0);
  //    const VectorXd xvals = (VectorXd(6) <<  0.0, 0.2, 0.4, 0.6, 0.8, 0.8).finished();
  //    //const VectorXd yvals = (VectorXd(6) << -0.5*pi,0.25*pi,0.0,0.25*pi,0.5*pi,0.5*pi).finished();
  //    const VectorXd yvals = (VectorXd(6) << -1.0, 0.5, 0.0, 0.5, 1.0, 1.0).finished();
  //    //const Spline2d splA = SplineFitting<Spline2d>::Interpolate(mCmdServoA.col(1).transpose(), 3, mCmdServoA.col(0).transpose());
  //    //const Spline2d splB = SplineFitting<Spline2d>::Interpolate(mCmdServoB.col(1).transpose(), 3, mCmdServoB.col(0).transpose());
  //    const Spline2d splA = SplineFitting<Spline2d>::Interpolate(yvals.transpose(), 3, xvals.transpose());

  //    double t = mWorld->getTime();
  //    mWorld->step();
  //    mController->setDesiredDof(0, splA(t)[0]);
  //    if (t < 0.8)
  //    {
  //      if ( abs(t  - floor(t / 0.01) * 0.01) < 1.e-3 )
  //      {
  //        std::cout << t << " "<< splA(t) << endl;
  //                  //<< " " << splB(t) << std::endl;
  //      }
  //    }
  //    else
  //    {
  //        mSimulating = false;
  //    }
  //*
  this->mCapture = false;
  if (int(floor(100*mWorld->getTime())) % 3 == 0)
  {
    if (this->mSave)
      this->mCapture = true;
  }
  // add damping
  VectorXd damping = computeDamping();
  // add control force


  mController->update(mWorld->getTime());
  mWorld->getSkeleton(0)->setForces(mController->getTorques());
  // simulate one step
  mWorld->step();
  //*/
}

void MyWindow::drawSkels()
{
  // Used for building inertia ellipsoids
  SelfAdjointEigenSolver<Matrix3d> es;
  Vector3d inertiaAxis;
  Isometry3d inertiaLocation;
  double s11, s22, s33, m;

  for (unsigned int i = 0; i < mWorld->getNumSkeletons(); ++i)
  {
    Skeleton* skel = mWorld->getSkeleton(i);

    // Hide/show render mesh and show/hide inertia ellipsoid
    if (mVisibleInertiaEllipsoid)
    {
      for (unsigned int j = 0; j < skel->getNumBodyNodes(); ++j)
      {
        BodyNode* bodyNode = skel->getBodyNode(j);

        // TODO(JS): Double check this out. Why do we need this?
        //bodyNode->updateTransform();

        // Compute inertia w.r.t. world frame
        Matrix3d R  = bodyNode->getTransform().linear();
        Matrix3d Ic = bodyNode->getSpatialInertia().topLeftCorner<3,3>();
        Matrix3d Iw = R * Ic * R.transpose();

        // Debug code to verify if Iw is correct
//        Matrix3d R2  = bodyNode->getWorldTransform().linear();
//        Matrix3d Ic2 = Matrix3d::Zero();
//        Ic2(0, 0) = bodyNode->mIxx;
//        Ic2(1, 1) = bodyNode->mIyy;
//        Ic2(2, 2) = bodyNode->mIzz;
//        Ic2(0, 1) = bodyNode->mIxy;
//        Ic2(0, 2) = bodyNode->mIxz;
//        Ic2(1, 2) = bodyNode->mIyz;
//        Ic2(1, 0) = Ic2(0, 1);
//        Ic2(2, 0) = Ic2(0, 2);
//        Ic2(2, 1) = Ic2(1, 2);
//        double mass = bodyNode->getMass();
//        Vector3d p = bodyNode->getLocalCOM();
//        Matrix3d skewP = math::makeSkewSymmetric(p);
//        std::cout << "skewP: " << std::endl << skewP << std::endl;
//        Matrix3d Iw2 = R2 * Ic2 * R2.transpose() + mass * skewP.transpose() * skewP;

        es.compute(Iw);
        s11 = es.eigenvalues()[0];
        s22 = es.eigenvalues()[1];
        s33 = es.eigenvalues()[2];
        m = bodyNode->getMass();
        inertiaAxis[0] = sqrt( 2.5 / m * (-s11 + s22 + s33 ) );
        inertiaAxis[1] = sqrt( 2.5 / m * ( s11 - s22 + s33 ) );
        inertiaAxis[2] = sqrt( 2.5 / m * ( s11 + s22 - s33 ) );

        mRI->pushMatrix();
        mRI->setPenColor(Vector3d(0.8, 0.2, 0.2));
        inertiaLocation.setIdentity();
        inertiaLocation.translate(bodyNode->getWorldCOM());
        inertiaLocation.rotate(es.eigenvectors());
        mRI->transform(inertiaLocation);
        mRI->drawEllipsoid(inertiaAxis);
        mRI->popMatrix();
      }
    }
    else
    {
//      glLineWidth(2.0f);
//      glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
//      skel->draw(mRI);
      glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
      skel->draw(mRI);
    }
    // Hide/show collision volume
    if (mVisibleCollisionShape)
    {
      Shape* collisionShape;
      for (unsigned int j = 0; j < skel->getNumBodyNodes(); ++j)
      {
        mRI->pushMatrix();
        mRI->transform(skel->getBodyNode(j)->getTransform());

        for (unsigned int k = 0; k < skel->getBodyNode(j)->getNumCollisionShapes(); ++k)
        {
          mRI->pushMatrix();

          glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
          collisionShape = skel->getBodyNode(j)->getCollisionShape(k);
          collisionShape->draw(mRI, Vector4d(0.2, 0.8, 0.2, 1.0), false);

          mRI->popMatrix();
        }

        mRI->popMatrix();
      }
    }

    if (mVisibleCOM)
    {
      Skeleton* skel = mWorld->getSkeleton(0);
      Vector3d com   = skel->getWorldCOM();

      mRI->pushMatrix();
      mRI->setPenColor(Vector3d(0.2, 0.8, 0.2));
      mRI->translate(com);
      mRI->drawEllipsoid(Vector3d(0.025, 0.025, 0.025));
      mRI->popMatrix();
    }

//    if (mVisibleAngularMomentum)
//    {
//      Skeleton* skel = mWorld->getSkeleton(0);
//      Vector3d com   = skel->getWorldCOM();
//      Vector3d angMomentum = skel->getMomentum(skel->getWorldCOM()).head<3>();

//      mRI->pushMatrix();
//      mRI->setPenColor(Vector3d(0.2, 0.2, 0.8));
//      drawArrow3D(com, angMomentum.normalized(), angMomentum.norm() * 10.0, 0.005);
//      mRI->popMatrix();
//    }

//    if (mVisibleLinearMomentum)
//    {
//      Skeleton* skel = mWorld->getSkeleton(0);
//      Vector3d com   = skel->getWorldCOM();
//      Vector3d linMomentum = skel->getMomentum(skel->getWorldCOM()).tail<3>();

//      mRI->pushMatrix();
//      mRI->setPenColor(Vector3d(0.8, 0.2, 0.2));
//      drawArrow3D(com, linMomentum.normalized(), linMomentum.norm() * 10.0, 0.005);
//      mRI->popMatrix();
//    }
  }
}

void MyWindow::keyboard(unsigned char _key, int _x, int _y)
{
  const double pi = DART_PI;
  switch(_key)
  {
    case ' ':  // use space key to play or stop the motion
      mSimulating = !mSimulating;
      if(mSimulating)
      {
        mPlay = false;
        glutTimerFunc(mDisplayTimeout, refreshTimer, 0);
      }
      break;
    case ',':  // use b key to visualize the center of mass
      mVisibleCOM = !mVisibleCOM;
      break;
    case '.':  // use b key to visualize the angular momentum
      mVisibleAngularMomentum = !mVisibleAngularMomentum;
      break;
    case '/':  // use b key to visualize the linear momentum
      mVisibleLinearMomentum = !mVisibleLinearMomentum;
      break;
    case 'n':  // use n key to visualize the collision shapes
      mVisibleCollisionShape = !mVisibleCollisionShape;
      break;
    case 'm':  // use m key to visualize the collision shapes
      mVisibleInertiaEllipsoid = !mVisibleInertiaEllipsoid;
      break;
    case 'l':  // use l key to load a script
      loadXmlScript(DART_DATA_PATH"urdf/inertiabot.xml");
      break;
    case 'i':  // use i-key to print out debug information
      for(int c = 0; c < mWorld->getNumSkeletons(); ++c)
      {
        Skeleton* skel = mWorld->getSkeleton(c);
        cout << "SKELETON: "    << c << endl
             << " NUM NODES : " << skel->getNumBodyNodes() << endl
             << " NUM DOF   : " << skel->getNumDofs() << endl
                // The number of body nodes and joints are same because every
                // single body node has its parent joint.
             << " NUM JOINTS: " << skel->getNumBodyNodes() << endl;
        for(int j = 0; j < mWorld->getSkeleton(c)->getNumBodyNodes(); ++j)
        {
          cout << "  Joint [" << j << "]: "
               << mWorld->getSkeleton(c)->getJoint(j)->getName() << endl;
        }
      }
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
        if (mPlayFrame >= mWorld->getRecording()->getNumFrames())
          mPlayFrame = 0;
        glutPostRedisplay();
      }
      break;
    case 'g':
      mController->changeControlMode(CompositeController::CM_AUTOMATIC);
      break;
    case 'h':
      mController->changeControlMode(CompositeController::CM_MANUAL);
      break;
    case 'b': // TODO(JS): need better key-mapping
      if (this->mSave == false)
      {
        dtmsg << "Capturing started." << std::endl;
        this->mSave = true;
      }
      else
      {
        dtmsg << "Capturing finished." << std::endl;
        this->mSave = false;
      }
      break;
    case '0':
      dtmsg << "States saving..." << std::endl;
      printResult();
      dtmsg << "States saved!\n" << std::endl;
    default:
      Win3D::keyboard(_key, _x, _y);
  }

  mController->keyboard(_key);

  glutPostRedisplay();
}

void MyWindow::printResult()
{
  //---------------------- Simulated Path ----------------------------------
  ofstream file;
  file.open(DART_DATA_PATH"states.txt");

  int nFrames = mWorld->getRecording()->getNumFrames();

//  for (int i = 0; i < nFrames; ++i)
//  {
//    file
//        << mBakedStates[i][mBakedStates[i].size() - 1]
//        << " "
//        << mBakedStates[i].head<10>().transpose()
//        << " "
//        << std::endl;
//  }

  file.close();

  //---------------------- Key Configurations ----------------------------------
//  file.open(DART_DATA_PATH"keyConfig.txt");

//  int nKeys = mController->mPlannedMotion.getNumKeyConfig();

//  for (int i = 0; i < nKeys; ++i)
//  {
//    file
//        << mController->mPlannedMotion.getKeyTime(i)
//        << " "
//        << mController->mPlannedMotion.getKeyConfig(i).transpose()
//        << std::endl;
//  }

//  file.close();
}

//void MyWindow::bake()
//{
//  CollisionDetector* cd
//      = mWorld->getConstraintSolver()->getCollisionDetector();
//  int nContacts  = cd->getNumContacts();
//  int nSkeletons = mWorld->getNumSkeletons();

//  Eigen::VectorXd state(
//        mWorld->getIndex(nSkeletons)  // States of skeletons
//        + 6 * nContacts               // Contact info
//        + 1);                         // Time

//  for (unsigned int i = 0; i < mWorld->getNumSkeletons(); i++)
//  {
//    state.segment(mWorld->getIndex(i),
//                  mWorld->getSkeleton(i)->getNumDofs())
//        = mWorld->getSkeleton(i)->getPositions();
//  }

//  for (int i = 0; i < nContacts; i++)
//  {
//    int begin = mWorld->getIndex(nSkeletons) + i * 6;

//    state.segment(begin, 3)     = cd->getContact(i).point;
//    state.segment(begin + 3, 3) = cd->getContact(i).force;
//  }

//  // Time
//  state[state.size() - 1]     = mWorld->getTime();

//  mBakedStates.push_back(state);
//}

void MyWindow::loadXmlScript(const char* _xmlFileName)
{
  XMLDocument xmldoc;
  XMLElement* rootElem = NULL;
  XMLElement* scriptElem = NULL;
  XMLElement* curElem = NULL;
  int numRepeats = 0;

  // Clear out the previous commands and seed with current time
  for (int c = 0; c < 2; ++c)
  {
    mCommands[2*c].clear();
    mCommands[2*c].push_back(mWorld->getTime());
    mCommands[2*c+1].clear();
    mCommands[2*c+1].push_back(0.0);
  }
  xmldoc.LoadFile(_xmlFileName);
  rootElem = xmldoc.FirstChildElement("RobotProgram");
  if (rootElem)
  {
    scriptElem = rootElem->FirstChildElement("script");
    if (scriptElem)
    {
      curElem = scriptElem->FirstChildElement("run");
      while (curElem)
      {
        // Find out if the command is repeated
        const char* strAttribute = curElem->Attribute("repeat");
        numRepeats = strAttribute ? atoi(strAttribute) : 0;
        // Search for command to add
        const char* strCmdName = curElem->GetText();
        if( strCmdName && numRepeats > 0 )
        {
          for(int c=0; c < numRepeats; ++c)
          {
            loadXmlCommand(rootElem, strCmdName, mCommands);
          }
          while(mCommands[0].size() > 0)
          {
            for (int c = 0; c < 4; ++c)
            {
              if (mCommands[c].size() > 0)
              {
                cout << mCommands[c].front() << " ";
                mCommands[c].pop_front();
              }
              else
              {
                cout << "- ";
              }
            }
            cout << endl;
          }
          //TODO: write script parsing
        }
        curElem = curElem->NextSiblingElement("run");
      }
    }
  }
}

void MyWindow::loadXmlCommand(tinyxml2::XMLElement* _rootElem,
                              const char* _strCmdName,
                              std::list<double> (&_vecCmd)[4])
{
  XMLElement* cmdElem = NULL;
  XMLElement* servoElem = NULL;
  XMLElement* curElem = NULL;
  list<double> lstTime, lstPosition;
  int numServo = -1;
  double tlast = 0;
  double degtorad = asin(1.0) / 90.0;

  cmdElem = _rootElem->FirstChildElement("command");
  while( cmdElem )
  {
    if( cmdElem->Attribute("name", _strCmdName) )
    {
      servoElem = cmdElem->FirstChildElement("servo");
      while( servoElem )
      {
        numServo = -1;
        if( servoElem->Attribute("name", "A") )
          numServo = 0;
        else if( servoElem->Attribute("name", "B") )
          numServo = 1;
        if( numServo >= 0 )
        {
          curElem = servoElem->FirstChildElement("time");
          lstTime = parseTextVector( curElem->GetText() );
          curElem = servoElem->FirstChildElement("position");
          lstPosition = parseTextVector( curElem->GetText() );
          if( lstTime.size() == lstPosition.size() )
          {
            // If there is already a time vector started we will add previous time
            tlast = _vecCmd[2*numServo].size() > 0 ? _vecCmd[2*numServo].back() : 0;
            for(list<double>::iterator i=lstTime.begin(); i!=lstTime.end(); i++)
            {
              _vecCmd[2*numServo].push_back( *i + tlast );
            }
            // Add the points to the command
            for(list<double>::iterator i=lstPosition.begin(); i!=lstPosition.end(); i++)
            {
              _vecCmd[2*numServo+1].push_back( (*i) * degtorad );
            }
          }
          else
          {
            cout << "ERROR: Command[" << _strCmdName << "] Servo["
                 << numServo << "]time and position vectors are different length." << endl;
          }
        }
        servoElem = servoElem->NextSiblingElement("servo");
      }
    }
    cmdElem = curElem->NextSiblingElement("command");
  }
}

std::list<double> MyWindow::parseTextVector(const char* _txtVector)
{
  list<double> lstVector;
  char* strVal;
  strVal = new char[ strlen( _txtVector ) ];
  strcpy( strVal, _txtVector );
  if( strlen(strVal) > 0 )
  {
    while( strlen(strVal) > 0 )
    {
      lstVector.push_back( strtod( strVal, &strVal ) );
    }
  }

  return lstVector;
}

Eigen::VectorXd MyWindow::computeDamping()
{
  int nDof = mWorld->getSkeleton(0)->getNumDofs();
  VectorXd damping = VectorXd::Zero(nDof);
  // add damping to each joint
  damping = -0.1 * mWorld->getSkeleton(0)->getPositions();
  return damping;
}
