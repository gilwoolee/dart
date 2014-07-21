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

#ifndef _MYWINDOW_
#define _MYWINDOW_
#ifdef WIN32
#include <gl/glew.h>
#endif
#include <cstdarg>

#include "dart/gui/Win3D.h"
#include "dart/integration/EulerIntegrator.h"
#include "dart/integration/RK4Integrator.h"
#include "dart/dynamics/Skeleton.h"

#include "Task.h"
#include "MaintainTask.h"
#include "TrackOriTask.h"
#include "TrackPoseTask.h"
#include "TrackEETask.h"
#ifdef WIN32
#include <nvGlutWidgets.h>
#include <nvGlutManipulators.h>
#endif
#include <string>
#include <fstream>
#include <cstring>

extern double extTorque;

namespace dart {
namespace constraint {
class ConstraintSolver;
}  // namespace constraint
namespace dynamics {
class ContactDynamics;
}  // namespace dynamics
namespace optimizer {
class ObjectiveBox;
class Var;
}  // namespace optimizer
}  // namespace dart

class Controller;

class MyWindow : public dart::gui::Win3D
                 //public dart::integration::IntegrableSystem
{
public:
  MyWindow(dart::dynamics::Skeleton* _mList = 0, ...): Win3D()
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
    mEdges.resize(numEdges);
    // the order is determined by the order of pivoting, related to roll direction
    mEdges[3] = Eigen::Vector3d(0.0,-0.02,-0.02);
    mEdges[2] = Eigen::Vector3d(0.0,0.02,-0.02);
    mEdges[1] = Eigen::Vector3d(0.0,0.02,0.02);
    mEdges[0] = Eigen::Vector3d(0.0,-0.02,0.02);
    mCorners.resize(numEdges);
    for (int i = 0; i < numEdges; ++i) {
      mCorners[i].resize(2);
      mCorners[i][0] = mCorners[i][1] = mEdges[i];
      mCorners[i][0](0) = 0.02;
      mCorners[i][1](0) = -0.02;
    }
    mN = 3;
    mBackN = 0;
    mRollNum = 0;
    mRollBackNum = 0;
    mRollBackIndex = 0;
    mAngles.resize(mN);

    mAngles[0] = 0.740675;
    mAngles[1] = 0.137237;
    mAngles[2] = -0.358023;

    if (_mList) {
      mSkels.push_back(_mList);
      va_list ap;
      va_start(ap, _mList);
      while (true) {
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
    for (unsigned int i = 0; i < mSkels.size(); i++) {
      int nDofs = mSkels[i]->getNumDofs();
      sumNDofs += nDofs;
      mIndices.push_back(sumNDofs);
    }
    initDyn();
  }

  virtual void draw();
  virtual void keyboard(unsigned char key, int x, int y);
  virtual void displayTimer(int _val);
  virtual void resize(int w, int h);
  virtual void click(int button, int state, int x, int y);
  virtual void drag(int x, int y);

  // Needed for integration
//  virtual Eigen::VectorXd getState();
//  virtual Eigen::VectorXd evalDeriv();
//  virtual void setState(const Eigen::VectorXd& state);

  // Update forward dynamics
  void step();

protected:
  // the name for all the output file
  static const char* const mFileName;

  int mSimFrame;
  bool mSim;
  int mPlayFrame;
  bool mPlay;
  bool mShowMarkers;
  bool mDrawOri;
  bool mDrawIK;
  bool mDrawModel;
  bool mDrawTarget;
  bool mDrawBoundEllipsoid;
  bool mDrawContactForce;

  int mDofNum;
  int mActiveDofIndex;
  int mObjDofNum;
  int mObjActiveDofIndex;

  std::vector<dart::dynamics::Skeleton*> mSkels;
  dart::integration::EulerIntegrator mIntegrator;

  dart::constraint::ConstraintSolver* mConstraintSolver;
  Controller* mController;
  std::vector<tasks::Task*> mTasks;

  std::vector<Eigen::VectorXd> mDofAccs;
  std::vector<Eigen::VectorXd> mDofVels;
  std::vector<Eigen::VectorXd> mDofs;

  // baked states
  std::vector<Eigen::VectorXd> mBakedStates;
  std::vector<Eigen::VectorXd> mBakedTanRelVels;
  std::vector<std::vector<bool> > mInContactVec; // the first dimension is the finger index, the second dimension is the frame
  std::vector<std::vector<Eigen::Vector3d> > mContactForceVec; // the first dimension is the finger index, the second dimension is the frame
  std::vector<std::vector<Eigen::Vector3d> > mFingerTargetContactForceVec; // the first dimension is the finger index, the second dimension is the frame
  std::vector<std::vector<Eigen::Vector3d> > mFingerContactPointVec; // the first dimension is the finger index, the second dimension is the frame
  std::vector<Eigen::Vector3d> mRootTrans;

  int mFingerNum;
  double mTimeStep;
  int mFrictionBasis;
  Eigen::Vector3d mGravity;
  Eigen::Vector3d mForce; // user input force for object
  std::vector<int> mContactIndices; // the length is determined by the number of contacts
  std::vector<Eigen::VectorXd> mContactPointsBallStart; // the length is determined by the number of fingers
  std::vector<Eigen::VectorXd> mContactPointsBallEnd; // the length is determined by the number of fingers
  std::vector<Eigen::Vector3d> mTargetContactPoints; // the length is determined by the number of fingers
  std::vector<Eigen::Vector3d> mContactPoints; // the average of actual contact points, the length is determined by the number of contacts
  std::vector<Eigen::Vector3d> mFingerContactPoints; // the average of actual contact points, the length is determined by the number of fingers
  std::vector<Eigen::Vector3d> mTargetContactForces; // the length is determined by the number of contacts
  std::vector<Eigen::Vector3d> mFingerTargetContactForces; // the length is determined by the number of fingers
  std::vector<Eigen::Vector3d> mContactForces; // the sum of actual contact force, the length is determined by the number of fingers
  std::vector<Eigen::Vector3d> mContactNormals; // the length is determined by the number of fingers

  std::string mPalmName;
  std::vector<std::string> mFingerNames;
  std::vector<std::string> mFingerRootNames;
  //std::vector<int> mFingerTipIndices;

  std::vector<int> mIndices;
  Eigen::Vector3d mBoundEllipsoidR;

  bool liftFlag;
  Eigen::Vector4d mAxisAngle;
  Eigen::Vector4d mTargetAxisAngle;

  // test, control with target
  Eigen::Vector3d mFirstInitDir;
  Eigen::Vector3d mFirstInitPoint;
  Eigen::Vector3d mSecondInitDir;
  Eigen::Vector3d mSecondInitPoint;
  Eigen::Vector3d mInitPosition;

  Eigen::Vector3d mTargetOri;
  Eigen::Vector3d mObjOri;

  double mPalmObjAngle;

  double mObjRadius; // the approximation of the object radius, if it is a sphere, then it is the radius of the sphere
  std::vector<Eigen::Vector3d> mEdges; // the edges of the polygon (pivoting edges) in the local coordinate, use the represent point in the 2D plane
  std::vector<std::vector<Eigen::Vector3d> > mCorners; // the corners of the polygon in the local coodinate, the first dimension is corresponding to edges, the second dimension is two corners for an edge, the first one has positive coordinate (close to little finger), the second one has negative coordinate
  int mRollNum;
  int mRollBackNum;
  int mN;
  int mBackN;
  int mRollBackIndex;
  std::vector<double> mAngles;
  int mPreContactEdge;
  Eigen::Vector3d mPreOri;
  int mUpFace;
  std::vector<Eigen::Vector3d> mPreHighCorners;
  std::vector<Eigen::Vector3d> mCurHighCorners;
  int mRollDir;

  void initDyn();
  void setPose();
  void bake();

  void saveBake();
  bool loadBake();
  std::string mBakeFileName;

  void doUI();
  //void solveBoundEllipsoid();
  Eigen::Vector3d ballToCartesian(Eigen::VectorXd _ball);
  Eigen::Vector3d ballToCartesian(Eigen::VectorXd _ball, double _radius);
  Eigen::Vector2d cartesianToBall(Eigen::Vector3d _cartesian);
  Eigen::Vector2d cartesianToBall(Eigen::Vector3d _cartesian, double _radius);
  Eigen::Vector3d objToEllipsoid(Eigen::Vector3d _obj);
  void setHandInitPose();
  void saveHandInitPose();
  void setObjInitPose();
  void updateTaskArray();
  void updateTaskTarget();
  void updateHandPose();
  void updateContact();
  void updateIntForce();
  void updateExtForce();
  Eigen::VectorXd updateForceTorqueConstraint();
  void updateContactPoint();
  Eigen::Vector3d evalFingerTipTraj(Eigen::VectorXd _start, Eigen::VectorXd _end, int _curFrame, int _totalFrame);
  Eigen::Matrix4d evalHandTransform();
  Eigen::Matrix4d evalHandTransformInv();
  Eigen::Vector3d applyHandTransform(Eigen::Vector3d _x);
  Eigen::Vector3d applyHandTransformDir(Eigen::Vector3d _v);
  Eigen::Vector3d applyHandTransformInv(Eigen::Vector3d _x);
  Eigen::Vector3d applyHandTransformInvDir(Eigen::Vector3d _v);
  Eigen::Vector4d matrixToAxisAngle(Eigen::Matrix3d& _m);
  Eigen::Matrix3d axisAngleToMatrix(Eigen::Vector4d& _v);
  Eigen::Matrix3d applyAxisAngle(Eigen::Vector4d& _v1, Eigen::Vector4d& _v2);
  Eigen::Vector3d evalVelOnObj(Eigen::Vector3d& _l);
  bool forceAchievable(int _index, Eigen::Vector3d _point, Eigen::Vector3d _force);
  void setHandAngle(double _angle);
  void setHandTrans(Eigen::Vector3d _preOri, Eigen::Vector3d _axis);
  int evalContactEdge();
  bool evalEdgeInContact(int _edgeIndex);
  Eigen::Matrix3d evalObjOri();
  int evalUpFace();
  void evalHighCorners();
  bool evalCornerChange();
  void evalRollDir();
};

#endif
