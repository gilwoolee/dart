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

#include "apps/inertiaBot/AutomaticController.h"

#include <omp.h>

#include <iostream>
#include <fstream>

#include "dart/common/Console.h"
#include "dart/dynamics/Skeleton.h"
#include "dart/dynamics/BodyNode.h"
#include "dart/dynamics/Joint.h"
#include "dart/constraint/ConstraintSolver.h"
#include "dart/collision/CollisionDetector.h"
#include "dart/utils/Paths.h"

using namespace std;

using namespace Eigen;

using namespace dart;
using namespace dynamics;

AutomaticController::AutomaticController(Skeleton* _skel,
    constraint::ConstraintSolver* _constDyn)
  : Controller(_skel, _constDyn),
    mControlPhase(CP_FALLING),
    mIsOnFallingController(true),
    mIsOnRollingController(true),
    mImpactPose(Vector3d::Zero()),
    mUseSmallTrianglePath(false),
    mReplanningThreshold(DART_RADIAN * 5.0),
    mPlannedMotion(mSkel),
    mStationaryCasePlanningTime(15.0),
    mRollingJointAngle(DART_RADIAN * 45.0)
{
  int nDof = 2;

  mTorques.resize(mSkel->getNumDofs());
  mTorques.fill(0);
  mKp = MatrixXd::Zero(nDof, nDof);
  mKd = MatrixXd::Zero(nDof, nDof);
  for (int i = 0; i < nDof; i++)
  {
//    mKp(i, i) = 0.1;
//    mKd(i, i) = 0.025;
        mKp(i, i) = 0.2;
        mKd(i, i) = 0.05;
  }
  mPreOffset = 0.0;

  // K
  double allowedX = 0.05;  // Need to be specified manually by the user
//  double allowedX = 0.05;  // Need to be specified manually by the user
  double mass     = mSkel->getMass();
  double g        = mSkel->getGravity()[1];
  mComK = -mass * g / allowedX;
  std::cout << "allowedX: " << allowedX << std::endl;
  std::cout << "mass    : " << mass     << std::endl;
  std::cout << "g       : " << g        << std::endl;
  std::cout << "mComK   : " << mComK    << std::endl;

  // B
//  double v0;
//  mComB = std::fabs(mass * g / v0);
  mComB = std::sqrt(4.0 * mComK * mass);
  std::cout << "mComB   : " << mComB    << std::endl;

  mZ0 = 0.0;

  _prepareToSolveRollingTorqueVer1();
}

AutomaticController::~AutomaticController()
{
  _finishToSolveRollingTorqueVer1();
}

void AutomaticController::prestep(double _currentTime)
{
  Controller::prestep(_currentTime);

  doMotionPlanning(_currentTime);
}

void AutomaticController::activate(double _currentTime)
{
  dtmsg << "Automatic controller is activated." << std::endl;

  doMotionPlanning(_currentTime);
}

void AutomaticController::deactivate(double _currentTime)
{

}

void AutomaticController::update(double _time)
{
  Controller::update(_time);

  if (mControlPhase == CP_FALLING)
  {
//    std::cout << "ddq  : " << mSkel->get_ddq().transpose() << std::endl;
//    std::cout << "ddq2 : " << computeDDQ(mSkel, mSkel->get_tau()).transpose() << std::endl;

    if (isHitAnything(_time))\
    {
      mZ0 = mSkel->getWorldCOM()[1];
      std::cout << "COM at impact: " << mSkel->getWorldCOM().transpose()
                << std::endl;
      std::cout << "Impact time: " << _time << std::endl;
      printState("fallingStates.txt");
      changeControlPhase(CP_ROLLING);
    }
    else
    {
      if (mIsOnFallingController)
      {
        if (isReplaningNeeded(_time, mReplanningThreshold))
        {
          dtmsg << "Replanning..." << std::endl;
          doMotionPlanning(_time);
        }

        evalTorquesFallingPD();
//        evalTorquesFallingAPD();
//         evalTorquesFallingSPD();
      }
      else
        mTorques.setZero();
    }
  }

  if (mControlPhase == CP_ROLLING)
  {
    evalTorquesRollingVer1();
  }

  // Record current info
  static int count = 0;
  count++;
  if (count % 1000)
    bake();

  mSkel->clearExternalForceVector();
  mSkel->clearInternalForceVector();
  mSkel->setConstraintForceVector(VectorXd::Zero(mSkel->getNumDofs()));
}

const VectorXd& AutomaticController::getTorques() const
{
  return mTorques;
}

double AutomaticController::getTorque(int _index) const
{
  return mTorques[_index];
}

void AutomaticController::keyboard(unsigned char _key)
{
  Controller::keyboard(_key);

  switch(_key)
  {
    case 'r':  // Replanning
      doMotionPlanning(mCurrentTime);
      break;
    case 'a':  // On-off rolling controller
      if (mControlPhase == CP_ROLLING)
      {
        if (mIsOnRollingController)
        {
          dtmsg << "Rolling controller: [on] -> [off]" << std::endl;
          mIsOnRollingController = false;
        }
        else
        {
          dtmsg << "Rolling controller: [off] -> [on]" << std::endl;
          mIsOnRollingController = true;
        }
      }
    case '0':
      dtmsg << "AutomaticController: printFallingState save!" << std::endl;
      printState("wholeStates.txt");
      break;
  }
}

void AutomaticController::setImpactPose(const Vector3d& _pos)
{
  mImpactPose = _pos;
}

void AutomaticController::setUseSmallTrianglePath(bool _v)
{
  mUseSmallTrianglePath = _v;
}

void AutomaticController::onOffFallingController(bool _v)
{
  mIsOnFallingController = _v;
}

void AutomaticController::onOffRollingController(bool _v)
{
  mIsOnRollingController = _v;
}

void AutomaticController::setReplanningThreshold(double _v)
{
  assert(_v > 0);
  mReplanningThreshold = _v;
}

double AutomaticController::getReplanningThreshold() const
{
  return mReplanningThreshold;
}

void AutomaticController::setStationaryCasePlanningTime(double _v)
{
  assert(_v > 0.0);
  mStationaryCasePlanningTime = _v;
}

double AutomaticController::getStationaryCasePlanningTime() const
{
  return mStationaryCasePlanningTime;
}

void AutomaticController::setRollingJointAngle(double _v)
{
  mRollingJointAngle = _v;
}

double AutomaticController::getRollingJointAngle() const
{
  return mRollingJointAngle;
}

void AutomaticController::doMotionPlanning(double _currentTime)
{
  dtmsg << "Planning started..." << std::endl;

  double begin = omp_get_wtime();

  double timeBudget = 0.0;
  if (mSkel->getGravity()[1] == 0)
    timeBudget = mStationaryCasePlanningTime - _currentTime;
  else
    timeBudget = getPredictedFallTime();

  dtmsg << "Time budget: " << timeBudget << std::endl;

  // Current configuration
  VectorXd q = mSkel->get_q();

  // Initial configuration
  Vector3d qi(q[0], q[3], q[4]);

  // Plan key configurations
  MatrixXd qpath;
  if (mUseSmallTrianglePath)
  {
  qpath
      = threelinkOrientPlanSmallTriangle(
          qi, mImpactPose, Vector3d::Zero(), _currentTime, timeBudget,
          DART_PI_HALF, 10.0);
  }
  else
  {
    qpath
        = threelinkOrientPlanBigTriangle(
            qi, mImpactPose, Vector3d::Zero(), _currentTime, timeBudget,
            DART_PI_HALF, 10.0);
  }

  mPlannedMotion.setKeyConfig(
        qpath.col(0), qpath.col(1), qpath.col(2), qpath.col(3));


  // Print planned path information
  dtmsg << "[ Planned Motion Information ]" << std::endl;
  dtmsg << "- Time: [" << qpath(0, 3) << "] ~ [" << qpath(qpath.rows() - 1, 3)
        << "]" << std::endl;
  dtmsg << "- Initial configuration: " << qi.transpose() << std::endl;
  dtmsg << "- Final configuration  : " << mImpactPose.transpose() << std::endl;
//  dtmsg << "- qpath: \n" << qpath << std::endl;

  dtmsg << "Planning finished." << std::endl;

  double end = omp_get_wtime();

  dtmsg << "Planning time was." << end - begin << std::endl;
}

bool AutomaticController::isReplaningNeeded(double _time, double _tol)
{
//  if (getPredictedFallTime() < 0.0)
//    return false;

  //////////////////////////////////////////////////////////////////////////////
  // TODO(JS): Hack
//  if (mPlannedMotion.getPositionsAtTime(_time)[0] < DART_RADIAN * -5.0
//      || mPlannedMotion.getPositionsAtTime(_time)[0] > DART_RADIAN * 5.0
//      || mPlannedMotion.getPositionsAtTime(_time)[1] < DART_RADIAN * -5.0
//      || mPlannedMotion.getPositionsAtTime(_time)[1] > DART_RADIAN * 5.0)
//    return false;

  //////////////////////////////////////////////////////////////////////////////

  Vector2d desiredQ = mPlannedMotion.getPositionsAtTime(_time);
  if (std::fabs(desiredQ[0] - desiredQ[1]) > DART_RADIAN * 10.0)
    return false;

  // Current configuration
  VectorXd q = mSkel->get_q();
  double q0 = q[0];
  double q0_des = mPlannedMotion.getPredictedOrientation(_time);

  if (std::fabs(q0 - q0_des) < _tol)
    return false;

  return true;
}

bool AutomaticController::isHitAnything(double _time)
{
  dart::collision::CollisionDetector* cd
      = mConstraintDynamics->getCollisionDetector();
  int nContacts = cd->getNumContacts();

  for (int i = 0; i < nContacts; ++i)
  {
    dart::collision::Contact contact = cd->getContact(i);
    if (mSkel == contact.collisionNode1->getBodyNode()->getSkeleton())
      return true;
    if (mSkel == contact.collisionNode2->getBodyNode()->getSkeleton())
      return true;
  }

  return false;
}

void AutomaticController::changeControlPhase(
    AutomaticController::ControlPhase _cp)
{
  // If current control pahse is same with _cp do nothing
  if (mControlPhase == _cp)
    return;

  ControlPhase oldCP = mControlPhase;

  // Set current control phase to _cp
  mControlPhase = _cp;

  // Print message
  dtmsg << "AutomaticController (t = "
        << mCurrentTime
        << "): phase changed ["
        << getControlPhaseString(oldCP)
        << "] -> ["
        << getControlPhaseString(_cp)
        << "]"
        << std::endl;
}

const MatrixXd& AutomaticController::getKp() const
{
  return mKp;
}

const MatrixXd& AutomaticController::getKd() const
{
  return mKd;
}

void AutomaticController::evalTorquesFallingPD()
{
  Vector2d q  = mSkel->get_q().segment<2>(3);
  Vector2d dq = mSkel->get_dq().segment<2>(3);
  Vector2d desiredQ  = mPlannedMotion.getPositionsAtTime(mCurrentTime);
  Vector2d desiredDQ = mPlannedMotion.getVelocityAtTime(mCurrentTime);

  // Solve for the appropriate joint torques
  mTorques.setZero();
  mTorques.segment<2>(3) = -mKp * (q  - desiredQ) -mKd * (dq - desiredDQ);
}

void AutomaticController::evalTorquesFallingAPD()
{
  int dof = mSkel->getNumDofs();

  // TODO(JS): We don't consider any external forces here.
  const MatrixXd& M = mSkel->getAugMassMatrix();
  const MatrixXd& C = mSkel->getCoriolisMatrix();
  const VectorXd& g = mSkel->getGravityForceVector();

  VectorXd q   = mSkel->get_q();
  VectorXd dq  = mSkel->get_dq();
  VectorXd ddq = mSkel->get_ddq();

  VectorXd desiredQ   = q;
  VectorXd desiredDQ  = dq;
  VectorXd desiredDDQ = ddq;

  desiredQ.segment<2>(3)   = mPlannedMotion.getPositionsAtTime(mCurrentTime);
  desiredDQ.segment<2>(3)  = mPlannedMotion.getVelocityAtTime(mCurrentTime);
  desiredDDQ.segment<2>(3) = mPlannedMotion.getAccelerationAtTime(mCurrentTime);

  VectorXd tau_pd = VectorXd::Zero(dof);
  tau_pd.segment<2>(3) = - mKp * (q.segment<2>(3)  - desiredQ.segment<2>(3))
                         - mKd * (dq.segment<2>(3) - desiredDQ.segment<2>(3));

  VectorXd tau = M * desiredDDQ + C * desiredDQ + g + tau_pd;

  mTorques.setZero();
  mTorques.segment<2>(3) = tau.segment<2>(3);
}

void AutomaticController::evalTorquesFallingSPD()
{
  int    n  = mSkel->getNumDofs();
  double dt = mSkel->getTimeStep();

  VectorXd _dof    = mSkel->get_q();
  VectorXd _dofVel = mSkel->get_dq();
  VectorXd _dofAcc = mSkel->get_ddq();

  // TODO(JS):
  VectorXd constrForces = mConstraintDynamics->getContactForce(0);

  VectorXd desiredQ  = mPlannedMotion.getPositionsAtTime(mCurrentTime);
  VectorXd desiredDQ = mPlannedMotion.getVelocityAtTime(mCurrentTime);

  Eigen::MatrixXd invM = (mSkel->getAugMassMatrix() + mKd * dt).inverse();
  Eigen::VectorXd p = -mKp * (_dof    + _dofVel * dt - desiredQ);
  Eigen::VectorXd d = -mKd * (_dofVel/* + _dofAcc * dt - desiredDQ*/);
  // TODO(JS):
  Eigen::VectorXd qddot
      = invM * (-mSkel->getCombinedVector() + p + d + constrForces);

  mTorques.setZero();
  mTorques.segment<2>(3) = p + d - mKd * qddot * dt;
  mTorques.segment<2>(3) = p + d;

  // Just to make sure no illegal torque is used
//  for (int i = 0; i < 3; i++)
//    mTorques[i] = 0.0;
}

void AutomaticController::evalTorquesRollingVer1()
{
  if (mIsOnRollingController)
  {
    if (mConstraintDynamics->getCollisionDetector()->getNumContacts() > 0)
      _solveRollingTorqueVer1();
    else
      mTorques = computeTauPD(mSkel);
  }
  else
  {
    mTorques.setZero();
  }

//  std::cout << "AutomaticController::evalTorquesRollingVer1(): "
//            << mTorques.transpose() << std::endl;
}

void AutomaticController::_prepareToSolveRollingTorqueVer1()
{
  double lb[2] = { -10.0, -10.0 }; /* lower bounds */
  double ub[2] = {  10.0,  10.0 }; /* upper bounds */

  opt = nlopt_create(NLOPT_LD_SLSQP, 2); /* algorithm and dimensionality */
//  opt = nlopt_create(NLOPT_LN_COBYLA, 2); /* algorithm and dimensionality */
//  opt = nlopt_create(NLOPT_GN_ISRES, 2); /* algorithm and dimensionality */
  nlopt_set_maxeval(opt, 1000);
  nlopt_set_lower_bounds(opt, lb);
  nlopt_set_upper_bounds(opt, ub);
  nlopt_set_min_objective(
        opt, AutomaticController::_objectFuncRollingTorqueVer1, this);
//  nlopt_set_min_objective(
//        opt, AutomaticController::_constraintFuncRollingTorqueVer1_JS, this);

//  nlopt_add_equality_constraint(
//        opt, AutomaticController::_constraintFuncRollingTorqueVer1, this, 1e-3);
//  nlopt_add_equality_constraint(
//        opt, AutomaticController::_constraintFuncRollingTorqueVer1_JS, this, 1e-3);

//  double tol[8] = {1e-3, 1e-3, 1e-3, 1e-3, 1e-3, 1e-3, 1e-3, 1e-3};
//  nlopt_add_inequality_mconstraint(
//        opt, 8,
//        AutomaticController::_constraintFuncRollingTorqueVer1_KinematicLimits,
//        this, tol);

//  double tol[4] = {1e-3, 1e-3, 1e-3, 1e-3};
//  nlopt_add_inequality_mconstraint(
//        opt, 4,
//        AutomaticController::_constraintFuncRollingTorqueVer1_KinematicLimits4,
//        this, tol);

  nlopt_set_xtol_rel(opt, 1e-4);
}

void AutomaticController::_solveRollingTorqueVer1()
{
  double x[2];// = { 1.234, 5.678 };  /* some initial guess */
  x[0] = mTorques[3];
  x[1] = mTorques[4];
//  x[0] = 0.0;
//  x[1] = 0.0;

  static int count = 0;
  static double duration = 0.0;
  count++;

  double minf = 0.0; /* the minimum objective value, upon return */

//  double b = ac->mComB;
//  double k = ac->mComK;
//  mZ0 = 0.05;
//  double k = 50000.0;
//  double b = 20.0;
//  Vector3d  COM = mSkel->getWorldCOM();
//  Vector3d dCOM = mSkel->getWorldCOMVelocity();
//  double f_v_z = -b * dCOM[1] - k * (COM[1] - mZ0);

//  std::cout << "(COM - Z0): " << (COM[1] - mZ0) << std::endl;
//  std::cout << "dCOM[1]   : " << dCOM[1] << std::endl;
//  std::cout << "f_v_z     : " << f_v_z << std::endl;

  double begin = omp_get_wtime();

  if (nlopt_optimize(opt, x, &minf) < 0)
    std::cout << "nlopt failed!\n";
//  else
//    std::cout << "found minimum: " << minf << std::endl;

//  mSkel->getBodyNode(0)->addExtForce(Vector3d(0.0, f_v_z, 0.0));

  duration += omp_get_wtime() - begin;

  if (count == 1000)
  {
    std::cout << "Average time: " << duration / count << std::endl;
    duration = 0.0;
    count = 0;
  }

//  double end = omp_get_wtime();

//  std::cout << "duration: " << end - begin << std::endl;

  int dof = mSkel->getNumDofs();
  VectorXd tau_pd = VectorXd::Zero(dof);
  tau_pd = computeTauPD(mSkel);

//  std::cout << "x: " << x[0] << ", " << x[1] << std::endl;

  mTorques.setZero();
  mTorques[3] = x[0] + tau_pd[3];
  mTorques[4] = x[1] + tau_pd[4];
}

double AutomaticController::_objectFuncRollingTorqueVer1(
    unsigned n, const double* x, double* grad, void* my_func_data)
{
  AutomaticController* ac = static_cast<AutomaticController*>(my_func_data);
  Skeleton* skel = ac->getSkeleton();
  int dof = skel->getNumDofs();
  double dt = skel->getTimeStep();

  Vector3d  COM = skel->getWorldCOM();
  Vector3d dCOM = skel->getWorldCOMVelocity();
  MatrixXd   Jc = skel->getWorldCOMJacobian();
  MatrixXd  dJc = skel->getWorldCOMJacobianTimeDeriv();
  double b = ac->mComB;
  double k = ac->mComK;
//  ac->mZ0 = 0.05;
//  k = 1000.0;
//  b = 10.0;

  // Tau_c
  VectorXd tau_c = VectorXd::Zero(dof);
  tau_c[3] = x[0];
  tau_c[4] = x[1];

  // Tau_pd
  // TODO(JS): PD controller for tracking rolling pose
  VectorXd tau_pd = VectorXd::Zero(dof);
  tau_pd = ac->computeTauPD(skel);

//  std::cout << ac->mZ0 << std::endl;

  // Tau_v
  double f_v_z = -b * dCOM[1] - k * (COM[1] - ac->mZ0);
//  std::cout << "f_v_z: " << f_v_z << std::endl;
  Vector3d f_v(0.0, f_v_z, 0.0);
  VectorXd tau_v = Jc.transpose() * f_v;
//  std::cout << "tau_v: " << tau_v << std::endl;

  VectorXd dq    = skel->get_dq();
  VectorXd ddq   = ac->computeDDQ(skel, tau_c + tau_pd);
  VectorXd ddq_o = ac->computeDDQ(skel, tau_v);

//  std::cout << "q: " << skel->get_q().transpose() << std::endl;
//  std::cout << "dq: " << skel->get_dq().transpose() << std::endl;
//  std::cout << "tau_c: " << tau_c.transpose() << std::endl;
//  std::cout << "tau_pd: " << tau_pd.transpose() << std::endl;
//  std::cout << "tau_v: " << tau_v.transpose() << std::endl;
//  std::cout << "ddq: " << ddq.transpose() << std::endl;
//  std::cout << "ddq_o: " << ddq_o.transpose() << std::endl;
//  std::cout << "diff: " << (ddq - ddq_o).transpose() << std::endl;
//  std::cout << std::endl;

  double ddCOMz   = (dJc.row(1) * dq + Jc.row(1) * ddq).squaredNorm();
  double ddCOMz_o = (dJc.row(1) * dq + Jc.row(1) * ddq_o).squaredNorm();

  double dCOMz   = dCOM[1] + ddCOMz   * dt;
  double dCOMz_o = dCOM[1] + ddCOMz_o * dt;

  double f1 = 0.5 * (ddq - ddq_o).dot(ddq - ddq_o);

  double f2 = 0.5 * (ddCOMz - ddCOMz_o) * (ddCOMz - ddCOMz_o);

  double f3 = 0.5 * (dCOMz - dCOMz_o) * (dCOMz - dCOMz_o);

//  f += 100.0 * (dJc.row(0) * dq + Jc.row(0) * ddq).squaredNorm();

//  if (grad)
//  {
//    MatrixXd invM = skel->getInvAugMassMatrix();
//    Vector2d EigenGrad = invM.rightCols<2>().transpose() * (ddq - ddq_o);
//    grad[0] = EigenGrad[0];
//    grad[1] = EigenGrad[1];
//  }

//  if (grad)
//  {
//    MatrixXd invM = skel->getInvAugMassMatrix();
//    Vector2d EigenGrad = (ddCOMz - ddCOMz_o) * Jc.row(1) * invM.rightCols<2>();
//    grad[0] = EigenGrad[0];
//    grad[1] = EigenGrad[1];
//  }

  if (grad)
  {
    MatrixXd invM = skel->getInvAugMassMatrix();
    Vector2d EigenGrad = (dCOMz - dCOMz_o) * Jc.row(1) * invM.rightCols<2>();
    grad[0] = dt*EigenGrad[0];
    grad[1] = dt*EigenGrad[1];
  }

  return f3;
}

VectorXd AutomaticController::computeTauPD(Skeleton* _skel)
{
  int dof = _skel->getNumDofs();
  VectorXd tau_pd = VectorXd::Zero(dof);
  Vector2d _q  = _skel->get_q().segment<2>(3);
  Vector2d _dq = _skel->get_dq().segment<2>(3);

  // These desired values should be modified.
  Vector2d desiredQ(DART_RADIAN * 45.0, DART_RADIAN * -45.0);
  Vector2d desiredDQ(0.0, 0.0);

  double q0 = _skel->get_q()[0];

  // Calculate number of max cycles
  int nRot = std::floor(std::fabs(q0) / DART_2PI);

  // Calculate remaining cycle rotation
  double rotRemains = std::fabs(q0) - nRot * DART_2PI;

//  if (-90.0 * DART_RADIAN < rotRemains && rotRemains < 90.0 * DART_RADIAN)
//    desiredQ << DART_RADIAN * +45.0, DART_RADIAN * -45.0;
//  else
//    desiredQ << DART_RADIAN * -45.0, DART_RADIAN * +45.0;

//  if (-90.0 * DART_RADIAN < rotRemains && rotRemains < 90.0 * DART_RADIAN)
    desiredQ << mRollingJointAngle, -mRollingJointAngle;
//  else
//    desiredQ << -mRollingJointAngle, mRollingJointAngle;

  tau_pd.segment<2>(3) = -getKp() * (_q  - desiredQ)
                         - getKd() * (_dq - desiredDQ);

  return tau_pd * 0.00002;
}

double AutomaticController::_constraintFuncRollingTorqueVer1(
    unsigned n, const double* x, double* grad, void* data)
{
  AutomaticController* ac = static_cast<AutomaticController*>(data);
  Skeleton* skel = ac->getSkeleton();
  int dof = skel->getNumDofs();
  double m = skel->getMass();
//  double b = ac->mComB;
  double b = 0.01;
//  double k = ac->mComK;
  double k = 0.01;
  double g = skel->getGravity()[1];
//  double z0 = ac->mZ0;
  double z0 = 0.00;
  VectorXd tau  = VectorXd::Zero(dof);
  tau[3] = x[0];
  tau[4] = x[1];

  VectorXd dq  = skel->get_dq();
  VectorXd ddq = ac->computeDDQ(skel, tau);

//  Vector3d c = skel->getWorldCOM();
  double cz = skel->getWorldCOM()[1];

  MatrixXd J = skel->getWorldCOMJacobian();

  MatrixXd dJ = skel->getWorldCOMJacobianTimeDeriv();

  double f = 0.0;
  f = (J.row(1) * ddq
      + (dJ.row(1) + (b/m)*J.row(1)) * dq).squaredNorm()
      + (k / m) * cz
      + (g - (k/m) * z0);

//  if (grad)
//  {
//    MatrixXd invM = skel->getInvAugMassMatrix();
//    VectorXd EigenGrad = J.row(1) * invM.rightCols<2>();
//    grad[0] = EigenGrad[0];
//    grad[1] = EigenGrad[1];
//  }

//  return f;
  return std::fabs(f);
}

double AutomaticController::_constraintFuncRollingTorqueVer1_JS(
    unsigned n, const double* x, double* grad, void* data)
{
  AutomaticController* ac = static_cast<AutomaticController*>(data);
  Skeleton* skel = ac->getSkeleton();
  double dt = skel->getTimeStep();
  int dof = skel->getNumDofs();
  double m = skel->getMass();
//  double b = ac->mComB;
  double b = 1.0;
//  double k = ac->mComK;
  double k = 10;
  double g = skel->getGravity()[1];
//  double z0 = ac->mZ0;
  double z0 = 0.01;
  VectorXd tau  = VectorXd::Zero(dof);
  tau[3] = x[0];
  tau[4] = x[1];

  VectorXd q   = skel->get_dq();
  VectorXd dq  = skel->get_dq();
  VectorXd ddq = ac->computeDDQ(skel, tau);

  VectorXd dqNext = dq + dt * ddq;
  VectorXd qNext = q + dq * dqNext;

//  Vector3d c = skel->getWorldCOM();
//  double cz = skel->getWorldCOM()[0];

  MatrixXd J = skel->getWorldCOMJacobian();

  MatrixXd dJ = skel->getWorldCOMJacobianTimeDeriv();

  Vector3d COMAcc = dJ * dq + J * ddq;

//  double f = COMAcc[1];
  double f = ddq[3] - ddq[4];

//  if (grad)
//  {
//    MatrixXd invM = skel->getInvAugMassMatrix();
//    VectorXd EigenGrad = J.row(0) * invM.rightCols<2>();
//    grad[0] = EigenGrad[0];
//    grad[1] = EigenGrad[1];
//  }

  return -f;
}

void AutomaticController::_constraintFuncRollingTorqueVer1_KinematicLimits(
    unsigned m,
    double* result,
    unsigned n,
    const double* x,
    double* gradient,
    void* func_data)
{
  AutomaticController* ac = static_cast<AutomaticController*>(func_data);
  Skeleton* skel = ac->getSkeleton();
  double dt = skel->getTimeStep();
  int dof = skel->getNumDofs();

  // Tau_c
  VectorXd tau_c = VectorXd::Zero(dof);
  tau_c[3] = x[0];
  tau_c[4] = x[1];

  // Tau_pd
  // TODO(JS): PD controller for tracking rolling pose
  VectorXd tau_pd = VectorXd::Zero(dof);
  tau_pd = ac->computeTauPD(skel);

  VectorXd ddq   = ac->computeDDQ(skel, tau_c + tau_pd);
  VectorXd dq_next = skel->get_dq() + dt * ddq;
  VectorXd  q_next = skel->get_dq() + dt * dq_next;


  double pb = DART_RADIAN * 90;
  double vb = DART_RADIAN * 10;

  result[0] =   q_next[3] - pb;
  result[1] =  -q_next[3] + pb;
  result[2] =   q_next[4] - pb;
  result[3] =  -q_next[4] + pb;
  result[4] =  dq_next[3] - vb;
  result[5] = -dq_next[3] + vb;
  result[6] =  dq_next[4] - vb;
  result[7] = -dq_next[4] + vb;

//  if (gradient)
//  {
//    MatrixXd invM = skel->getInvAugMassMatrix();
//    Vector2d EigenGrad = (ddCOMz - ddCOMz_o) * Jc.row(1) * invM.rightCols<2>();
//    grad[0] = EigenGrad[0];
//    grad[1] = EigenGrad[1];
//  }
}

void AutomaticController::_constraintFuncRollingTorqueVer1_KinematicLimits4(
    unsigned m,
    double* result,
    unsigned n,
    const double* x,
    double* gradient,
    void* func_data)
{
  AutomaticController* ac = static_cast<AutomaticController*>(func_data);
  Skeleton* skel = ac->getSkeleton();
  double dt = skel->getTimeStep();
  int dof = skel->getNumDofs();

  // Tau_c
  VectorXd tau_c = VectorXd::Zero(dof);
  tau_c[3] = x[0];
  tau_c[4] = x[1];

  // Tau_pd
  // TODO(JS): PD controller for tracking rolling pose
  VectorXd tau_pd = VectorXd::Zero(dof);
  tau_pd = ac->computeTauPD(skel);

  VectorXd ddq   = ac->computeDDQ(skel, tau_c + tau_pd);
  VectorXd dq_next = skel->get_dq() + dt * ddq;
  VectorXd  q_next = skel->get_dq() + dt * dq_next;


  double pb = DART_RADIAN * 90.0;
  double vb = DART_RADIAN * 10.0;
  double ab = DART_RADIAN * 0.01;

//  result[0] =   q_next[3] - pb;
//  result[1] =  -q_next[3] + pb;
//  result[2] =   q_next[4] - pb;
//  result[3] =  -q_next[4] + pb;

//  result[1] =  dq_next[3] - vb;
//  result[2] = -dq_next[3] + vb;
//  result[3] =  dq_next[4] - vb;
//  result[4] = -dq_next[4] + vb;

  result[1] =  ddq[3] - ab;
  result[2] = -ddq[3] + ab;
  result[3] =  ddq[4] - ab;
  result[4] = -ddq[4] + ab;


//  if (gradient)
//  {
//    MatrixXd invM = skel->getInvAugMassMatrix();
//    Vector2d EigenGrad = (ddCOMz - ddCOMz_o) * Jc.row(1) * invM.rightCols<2>();
//    grad[0] = EigenGrad[0];
//    grad[1] = EigenGrad[1];
  //  }
}

void AutomaticController::_finishToSolveRollingTorqueVer1()
{
  nlopt_destroy(opt);
}

VectorXd AutomaticController::computeDDQ(Skeleton* _skel,
                                         const Eigen::VectorXd& _tau)
{
  int dof = _skel->getNumDofs();
  VectorXd ddq = VectorXd::Zero(dof);

  MatrixXd M = _skel->getAugMassMatrix();
  MatrixXd invM = _skel->getInvAugMassMatrix();
  VectorXd Cg   = _skel->getCombinedVector();
  VectorXd tauConstraint = _skel->getConstraintForceVector();
//  VectorXd tauConstraint = VectorXd::Zero(dof);
  VectorXd tauContact = VectorXd::Zero(dof);
  VectorXd damp = _skel->getDampingForceVector();
  int nBodies = _skel->getNumBodyNodes();
  for (int i = 0; i < nBodies; ++i)
  {
    BodyNode* body = _skel->getBodyNode(i);
    int nContacts = body->getNumContactForces();
    int nDepGenCoords = body->getNumDependentGenCoords();
    MatrixXd Jt = body->getBodyJacobian().transpose();
    VectorXd force = VectorXd::Zero(nDepGenCoords);
    for (int j = 0; j < nContacts; ++j)
    {
      Vector6d bodyContactForce = body->getContactForce(j);
      force += Jt * bodyContactForce;
    }
    for (int j = 0; j < nDepGenCoords; ++j)
    {
      int idx = body->getDependentGenCoord(j);
      tauContact[idx] += force[j];
    }
  }

  tauContact = _skel->getExternalForceVector();

//  std::cout << "tauConstraint" << tauConstraint.transpose() << std::endl;

  ddq = invM * (_tau + tauConstraint + tauContact + damp - Cg);
//  ddq = M.inverse() * (_tau + tauConstraint + tauContact + damp - Cg);

  return ddq;
}

void AutomaticController::evalTorquesRollingVer2()
{
  // TODO(JS): Not implemented yet.
}

MatrixXd AutomaticController::threelinkOrientPlan_VER1(const Vector3d& _qi,
                                              const Vector3d& _qf,
                                              const Vector3d& _wo,
                                              double _totalTime,
                                              double _jointPosLimit,
                                              double _jointVelLimit)
{
  // For explanation sake, explode variables
  double q0i = _qi[0];
  double q1i = _qi[1];
  double q2i = _qi[2];

  double q0f = _qf[0];
  double q1f = _qf[1];
  double q2f = _qf[2];

  // First define the total amount of rotation of the root body desired
  double dq = q0f - q0i;

  // The smallest deviation to the root body angle should occur by selecting a
  // path that drives the joint angles to the "q1==q2 diagonal", and the
  // smallest individual joint angle is the path that produces a perpendicular
  // path to the q1==q2 diagonal. So find these paths first.
  //   Need to solve:
  //                 q1 - a = b
  //                 q2 + a = b
  //                          b = (q1 + q2) / 2
  //   Then:
  //         q1new == q2new = (q1 + q2) / 2
  //
  // This will give a starting point, qs, and ending point, qe, that the cycle
  // rotations can be computed with respect to.
  double qs = 0.5 * (q1i + q2i);
  double qe = 0.5 * (q1f + q2f);

  // We have three moves that we know that we will always take, qi->qs,
  // stuff in the middle, and qe->qf. Depending on whether there is initial
  // angular momentum will dictate how this is accomplished.
  // We define some postures
//  Vector2d po(q1i, q2i);  // Initial
//  Vector2d ps(qs, qs);    // Initial diag
//  Vector2d pe(qe, qe);    // Final diag
//  Vector2d pf(q1f, q2f);  // Final

  // Define the min/max postures
//  Vector2d pmin(_jointPosLimit, -_jointPosLimit);
//  Vector2d pmax(0.0, 0.0);

  // Calculate initial angular momentum and max/min possible angular velocity
//  MatrixXd Hdq0o = evalMomemtumJacobian0(po(1),   po(2));
//  MatrixXd Hdq0n = evalMomemtumJacobian0(pmin(1), pmin(2));
//  MatrixXd Hdq0x = evalMomemtumJacobian0(pmax(1), pmax(2));
//  MatrixXd Hdq0f = evalMomemtumJacobian0(pf(1),   pf(2));

  // Initial angular momentum
//  Vector3d Ho = Hdq0o * _wo;

  // Angular velocity at minimum configuration (max vel)
//  Vector3d wn = Hdq0n.inverse() * Ho;

  // Angular velocity at maximum configuration (min vel)
//  Vector3d wx = Hdq0x.inverse() * Ho;

  // Angular velocity at final configuration
//  Vector3d wf = Hdq0f.inverse() * Ho;

  // Calculate how much rotation will be caused by the beginning and ending
  // moves
  double dq0s = lineRotation(mSkel, Vector2d(q1i, q2i), Vector2d(qs,qs));
  double dq0e = lineRotation(mSkel, Vector2d(qe, qe), Vector2d(q1f,q2f));
//  [dq0os, dqFos, dqLos] = lineRotation(po,ps);
//  [dq0ef, dqFef, dqLef] = lineRotation(pe,pf);

  // Since the cycle will start and end at qs, also calculate the angle change
  // from qs to qe.
  double dq0se = lineRotation(mSkel, Vector2d(qs, qs), Vector2d(qe, qe));

  // Calculate the starting and ending angles of the cycling for the root body
  double q0s = q0i + dq0s;
  double q0e = q0f - dq0e - dq0se;

  // Calculate total angle required to cycle
  double dq0c = q0e - q0s;

  // Calculate the maximum cycle rotation
  double dqmax = cycleRotationBigTriangle(mSkel, _jointPosLimit);

  // Calculate number of max cycles
  int nmax = std::floor(std::fabs(dq0c) / dqmax);

  // Calculate remaining cycle rotation
  double dq0r = std::fabs(dq0c) - nmax * dqmax;

  // Solve for scaled cycle
  double scldqmax = solveCycleScaleBigTriangle(mSkel, dq0r,
                                               _jointPosLimit, Vector2d(0, 1));

  // Now build the cycle.
  // First build the basic cycle block
  MatrixXd cycle = MatrixXd::Zero(3, 3);
  if (dq0c > 0.0)
  {
    cycle << 0,  1,  1,
             0, -1,  1,
             0, -1, -1;
  }
  else
  {
    cycle << 0, -1, -1,
             0,  1, -1,
             0,  1,  1;
  }
  cycle *= _jointPosLimit;

  // Next build the full path
  MatrixXd qpath;

  if (scldqmax > 0)
  {
    int n = 2 + (nmax + 1) * 3 + 2;
    qpath = MatrixXd::Zero(n ,3);

    qpath(0, 0) = q0i;
    qpath(0, 1) = q1i;
    qpath(0, 2) = q2i;

    qpath(1, 0) = q0s;
    qpath(1, 1) = qs;
    qpath(1, 2) = qs;

    for (int i = 0; i < nmax; ++i)
      qpath.block<3, 3>(2 + i * 3, 0) = cycle;

    qpath.block<3, 3>(2 + nmax * 3, 0) = scldqmax * cycle;

    qpath(1, 0) = q0s;
    qpath(1, 1) = qs;
    qpath(1, 2) = qs;

    qpath(n - 2, 0) = q0e;
    qpath(n - 2, 1) = qe;
    qpath(n - 2, 2) = qe;

    qpath(n - 1, 0) = q0f;
    qpath(n - 1, 1) = q1f;
    qpath(n - 1, 2) = q2f;
  }
  else
  {
    int n = 2 + nmax * 3 + 2;
    qpath = MatrixXd::Zero(n ,3);

    qpath(0, 0) = q0i;
    qpath(0, 1) = q1i;
    qpath(0, 2) = q2i;

    qpath(1, 0) = q0s;
    qpath(1, 1) = qs;
    qpath(1, 2) = qs;

    for (int i = 0; i < nmax; ++i)
      qpath.block<3, 3>(2 + i * 3, 0) = cycle;

    qpath(1, 0) = q0s;
    qpath(1, 1) = qs;
    qpath(1, 2) = qs;

    qpath(n - 2, 0) = q0e;
    qpath(n - 2, 1) = qe;
    qpath(n - 2, 2) = qe;

    qpath(n - 1, 0) = q0f;
    qpath(n - 1, 1) = q1f;
    qpath(n - 1, 2) = q2f;
  }
  assert(scldqmax >= 0.0);

  // Finally insert the expected root body rotations
  for (int i = 1; i < qpath.rows(); ++i)
    qpath(i, 0) = qpath(i - 1, 0) + lineRotation(mSkel,
                    Vector2d(qpath(i - 1, 1), qpath(i - 1, 2)),
                    Vector2d(qpath(    i, 1), qpath(    i, 2)));

  return qpath;
}

MatrixXd AutomaticController::threelinkOrientPlanBigTriangle(
    const Eigen::Vector3d& _qi,
    const Eigen::Vector3d& _qf,
    const Eigen::Vector3d& _wo,
    double _startingTime,
    double _totalTime,
    double _jointPosLimit,
    double _jointVelLimit)
{
  // For explanation sake, explode variables
  double q0i = _qi[0];
  double q1i = _qi[1];
  double q2i = _qi[2];

  double q0f = _qf[0];
  double q1f = _qf[1];
  double q2f = _qf[2];

  // First define the total amount of rotation of the root body desired
  double dq = q0f - q0i;

  // The smallest deviation to the root body angle should occur by selecting a
  // path that drives the joint angles to the "q1==q2 diagonal", and the
  // smallest individual joint angle is the path that produces a perpendicular
  // path to the q1==q2 diagonal. So find these paths first.
  //   Need to solve:
  //                 q1 - a = b
  //                 q2 + a = b
  //                          b = (q1 + q2) / 2
  //   Then:
  //         q1new == q2new = (q1 + q2) / 2
  //
  // This will give a starting point, qs, and ending point, qe, that the cycle
  // rotations can be computed with respect to.
  double qs = 0.5 * (q1i + q2i);
  double qe = 0.5 * (q1f + q2f);

  // Calculate how much rotation will be caused by the beginning and ending
  // moves
  double dq0s = lineRotation(mSkel, Vector2d(q1i, q2i), Vector2d(qs,qs));
  double dq0e = lineRotation(mSkel, Vector2d(qe, qe), Vector2d(q1f,q2f));

  // Since the cycle will start and end at qs, also calculate the angle change
  // from qs to qe.
  double dq0se = lineRotation(mSkel, Vector2d(qs, qs), Vector2d(qe, qe));

  // Calculate the starting and ending angles of the cycling for the root body
  double q0s = q0i + dq0s;
  double q0e = q0f - dq0e - dq0se;

  // Calculate total angle required to cycle
  double dq0c = q0e - q0s;

  // Calculate the maximum cycle rotation
  double dqmax = cycleRotationBigTriangle(mSkel, _jointPosLimit);

  std::cout << "Max rotation: " << dqmax << std::endl;

  // Calculate number of max cycles
  int nmax = std::floor(std::fabs(dq0c) / dqmax);

  // Calculate remaining cycle rotation
  double dq0r = std::fabs(dq0c) - nmax * dqmax;

  // Solve for scaled cycle
  double scldqmax = solveCycleScaleBigTriangle(
                      mSkel, dq0r, _jointPosLimit, Vector2d(0, 1));

  // Now build the cycle.
  // First build the basic cycle block
  MatrixXd cycle = MatrixXd::Zero(3, 3);
  if (dq0c > 0.0)
  {
    cycle << 0,  1,  1,
             0, -1,  1,
             0, -1, -1;
  }
  else
  {
    cycle << 0, -1, -1,
             0,  1, -1,
             0,  1,  1;
  }
  cycle *= _jointPosLimit;

  // Next build the full path
  MatrixXd qpath;

  if (scldqmax > 0)
  {
    int n = 2 + (nmax + 1) * 3 + 2;
    qpath = MatrixXd::Zero(n ,4);

    qpath(0, 0) = q0i;
    qpath(0, 1) = q1i;
    qpath(0, 2) = q2i;

    qpath(1, 0) = q0s;
    qpath(1, 1) = qs;
    qpath(1, 2) = qs;

    for (int i = 0; i < nmax; ++i)
      qpath.block<3, 3>(2 + i * 3, 0) = cycle;

    qpath.block<3, 3>(2 + nmax * 3, 0) = scldqmax * cycle;

    qpath(1, 0) = q0s;
    qpath(1, 1) = qs;
    qpath(1, 2) = qs;

    qpath(n - 2, 0) = q0e;
    qpath(n - 2, 1) = qe;
    qpath(n - 2, 2) = qe;

    qpath(n - 1, 0) = q0f;
    qpath(n - 1, 1) = q1f;
    qpath(n - 1, 2) = q2f;
  }
  else
  {
    int n = 2 + nmax * 3 + 2;
    qpath = MatrixXd::Zero(n ,4);

    qpath(0, 0) = q0i;
    qpath(0, 1) = q1i;
    qpath(0, 2) = q2i;

    qpath(1, 0) = q0s;
    qpath(1, 1) = qs;
    qpath(1, 2) = qs;

    for (int i = 0; i < nmax; ++i)
      qpath.block<3, 3>(2 + i * 3, 0) = cycle;

    qpath(1, 0) = q0s;
    qpath(1, 1) = qs;
    qpath(1, 2) = qs;

    qpath(n - 2, 0) = q0e;
    qpath(n - 2, 1) = qe;
    qpath(n - 2, 2) = qe;

    qpath(n - 1, 0) = q0f;
    qpath(n - 1, 1) = q1f;
    qpath(n - 1, 2) = q2f;
  }
  assert(scldqmax >= 0.0);

  // Finally insert the expected root body rotations
  for (int i = 1; i < qpath.rows(); ++i)
    qpath(i, 0) = qpath(i - 1, 0) + lineRotation(mSkel,
                    Vector2d(qpath(i - 1, 1), qpath(i - 1, 2)),
                    Vector2d(qpath(    i, 1), qpath(    i, 2)));

//  dtmsg << "- qpath: \n" << qpath << std::endl;

  double totalPathLength = 0.0;
  for (int i = 0; i < qpath.rows() - 1; ++i)
  {
    Vector2d q1 = qpath.block<1,2>(    i, 1);
    Vector2d q2 = qpath.block<1,2>(i + 1, 1);
    double lineLength = (q2 - q1).norm();

    qpath(i + 1, 3)  = lineLength;
    totalPathLength += lineLength;
  }

//  std::cout << "_totalTime: " << _totalTime << std::endl;
//  std::cout << "totalPathLength: " << totalPathLength << std::endl;
//  dtmsg << "- qpath: \n" << qpath << std::endl;

  qpath(0, 3) = _startingTime;
  for (int i = 1; i < qpath.rows(); ++i)
    qpath(i, 3) = qpath(i - 1, 3) + qpath(i, 3) / totalPathLength * _totalTime;

  return qpath;
}

MatrixXd AutomaticController::threelinkOrientPlanSmallTriangle(
    const Eigen::Vector3d& _qi,
    const Eigen::Vector3d& _qf,
    const Eigen::Vector3d& _wo,
    double _startingTime,
    double _totalTime,
    double _jointPosLimit,
    double _jointVelLimit)
{
  // For explanation sake, explode variables
  double q0i = _qi[0];
  double q1i = _qi[1];
  double q2i = _qi[2];

  double q0f = _qf[0];
  double q1f = _qf[1];
  double q2f = _qf[2];

  // First define the total amount of rotation of the root body desired
  double dq = q0f - q0i;

  // The smallest deviation to the root body angle should occur by selecting a
  // path that drives the joint angles to the "q1==q2 diagonal", and the
  // smallest individual joint angle is the path that produces a perpendicular
  // path to the q1==q2 diagonal. So find these paths first.
  //   Need to solve:
  //                 q1 - a = b
  //                 q2 + a = b
  //                          b = (q1 + q2) / 2
  //   Then:
  //         q1new == q2new = (q1 + q2) / 2
  //
  // This will give a starting point, qs, and ending point, qe, that the cycle
  // rotations can be computed with respect to.
  double qs = 0.5 * (q1i + q2i);
  double qe = 0.5 * (q1f + q2f);

  // Calculate how much rotation will be caused by the beginning and ending
  // moves
  double dq0s = lineRotation(mSkel, Vector2d(q1i, q2i), Vector2d(qs,qs));
  double dq0e = lineRotation(mSkel, Vector2d(qe, qe), Vector2d(q1f,q2f));

  // Since the cycle will start and end at qs, also calculate the angle change
  // from qs to qe.
  double dq0se = lineRotation(mSkel, Vector2d(qs, qs), Vector2d(qe, qe));

  // Calculate the starting and ending angles of the cycling for the root body
  double q0s = q0i + dq0s;
  double q0e = q0f - dq0e - dq0se;

  // Calculate total angle required to cycle
  double dq0c = q0e - q0s;

  // Calculate the maximum cycle rotation
  double dqmax = cycleRotationSmallTriangle(mSkel, _jointPosLimit);
  std::cout << "Max rotation: " << dqmax << std::endl;

  // Calculate number of max cycles
  int nmax = std::floor(std::fabs(dq0c) / dqmax);

  // Calculate remaining cycle rotation
  double dq0r = std::fabs(dq0c) - nmax * dqmax;

  // Solve for scaled cycle
  double scldqmax = solveCycleScaleSmallTriangle(
                      mSkel, dq0r, _jointPosLimit, Vector2d(0, 1));

  // Now build the cycle.
  // First build the basic cycle block
  MatrixXd cycle = MatrixXd::Zero(3, 3);
  if (dq0c > 0.0)
  {
    cycle << 0,  0,  0,
             0,  1, -1,
             0,  0, -1;
  }
  else
  {
    cycle << 0,  0,  0,
             0,  0, -1,
             0,  1, -1;
  }
  cycle *= _jointPosLimit;

  // Next build the full path
  MatrixXd qpath;

  if (scldqmax > 0)
  {
    int n = 2 + (nmax + 1) * 3 + 2;
    qpath = MatrixXd::Zero(n ,4);

    qpath(0, 0) = q0i;
    qpath(0, 1) = q1i;
    qpath(0, 2) = q2i;

    qpath(1, 0) = q0s;
    qpath(1, 1) = qs;
    qpath(1, 2) = qs;

    for (int i = 0; i < nmax; ++i)
      qpath.block<3, 3>(2 + i * 3, 0) = cycle;

    qpath.block<3, 3>(2 + nmax * 3, 0) = scldqmax * cycle;

    qpath(1, 0) = q0s;
    qpath(1, 1) = qs;
    qpath(1, 2) = qs;

    qpath(n - 2, 0) = q0e;
    qpath(n - 2, 1) = qe;
    qpath(n - 2, 2) = qe;

    qpath(n - 1, 0) = q0f;
    qpath(n - 1, 1) = q1f;
    qpath(n - 1, 2) = q2f;
  }
  else
  {
    int n = 2 + nmax * 3 + 2;
    qpath = MatrixXd::Zero(n ,4);

    qpath(0, 0) = q0i;
    qpath(0, 1) = q1i;
    qpath(0, 2) = q2i;

    qpath(1, 0) = q0s;
    qpath(1, 1) = qs;
    qpath(1, 2) = qs;

    for (int i = 0; i < nmax; ++i)
      qpath.block<3, 3>(2 + i * 3, 0) = cycle;

    qpath(1, 0) = q0s;
    qpath(1, 1) = qs;
    qpath(1, 2) = qs;

    qpath(n - 2, 0) = q0e;
    qpath(n - 2, 1) = qe;
    qpath(n - 2, 2) = qe;

    qpath(n - 1, 0) = q0f;
    qpath(n - 1, 1) = q1f;
    qpath(n - 1, 2) = q2f;
  }
  assert(scldqmax >= 0.0);

  // Finally insert the expected root body rotations
  for (int i = 1; i < qpath.rows(); ++i)
    qpath(i, 0) = qpath(i - 1, 0) + lineRotation(mSkel,
                    Vector2d(qpath(i - 1, 1), qpath(i - 1, 2)),
                    Vector2d(qpath(    i, 1), qpath(    i, 2)));

//  dtmsg << "- qpath: \n" << qpath << std::endl;

  double totalPathLength = 0.0;
  for (int i = 0; i < qpath.rows() - 1; ++i)
  {
    Vector2d q1 = qpath.block<1,2>(    i, 1);
    Vector2d q2 = qpath.block<1,2>(i + 1, 1);
    double lineLength = (q2 - q1).norm();

    qpath(i + 1, 3)  = lineLength;
    totalPathLength += lineLength;
  }

//  std::cout << "_totalTime: " << _totalTime << std::endl;
//  std::cout << "totalPathLength: " << totalPathLength << std::endl;
//  dtmsg << "- qpath: \n" << qpath << std::endl;

  qpath(0, 3) = _startingTime;
  for (int i = 1; i < qpath.rows(); ++i)
    qpath(i, 3) = qpath(i - 1, 3) + qpath(i, 3) / totalPathLength * _totalTime;

  return qpath;
}

MatrixXd AutomaticController::evalMomemtumJacobian0(Skeleton* _skel,
                                                    double _q1, double _q2)
{
  // Prepare variables
  MatrixXd J0 = MatrixXd::Zero(6, 6);

  BodyNode* root = _skel->getBodyNode(0);
  BodyNode* legA = _skel->getBodyNode(1);
  BodyNode* legB = _skel->getBodyNode(2);

  assert(root->getName() == "body");
  assert(legA->getName() == "leg_A");
  assert(legB->getName() == "leg_B");

  // Backup q1, q2
  std::vector<int> id;
  id.push_back(3);
  id.push_back(4);
  Vector2d oldQ = _skel->getPositions(id);

  // Set _q1, _q2 to the skeleton
  Vector2d newQ(_q1, _q2);
  _skel->setConfig(id, newQ);

  // Compute Jacobian of momentum
  MatrixXd G0 = root->getInertia();
  MatrixXd G1 = legA->getInertia();
  MatrixXd G2 = legB->getInertia();

//  std::cout << "G0:\n" << G0 << std::endl << std::endl;
//  std::cout << "G1:\n" << G1 << std::endl << std::endl;
//  std::cout << "G2:\n" << G2 << std::endl << std::endl;

  MatrixXd   S0  = root->getParentJoint()->getLocalJacobian();
  Isometry3d T01 = legA->getParentJoint()->getLocalTransform();
  Isometry3d T02 = legB->getParentJoint()->getLocalTransform();

  Isometry3d Tsc = Isometry3d::Identity();
  Tsc.translation() = _skel->getWorldCOM();
  Isometry3d Tco = Tsc.inverse() * root->getWorldTransform();

  J0 = G0
       + math::transformInertia(T01.inverse(), G1)
       + math::transformInertia(T02.inverse(), G2);

  J0 = J0 * S0;

  J0 = math::dAdInvTJac(Tco, J0);

  // Restore q1, q2
  _skel->setConfig(id, oldQ);

//  std::cout << "J11:\n" << J0.block<3, 3>(0, 0) << std::endl << std::endl;
//  std::cout << "J12:\n" << J0.block<3, 3>(0, 3) << std::endl << std::endl;
//  std::cout << "J21:\n" << J0.block<3, 3>(3, 0) << std::endl << std::endl;
//  std::cout << "J22:\n" << J0.block<3, 3>(3, 3) << std::endl << std::endl;

  for (int i = 0; i < 3; ++i)
  {
    for (int j = 0; j < 2; ++j)
      assert((J0.block<3, 2>(0, 1))(i, j) < 1e-6);
  }

  return J0.topLeftCorner<3, 1>();
}

Vector3d AutomaticController::evalMomemtumJacobian1(Skeleton* _skel,
                                                    double _q1, double _q2)
{
  // Prepare variables
  Vector6d J1 = Vector6d::Zero(6);

  BodyNode* root = _skel->getBodyNode(0);
  BodyNode* legA = _skel->getBodyNode(1);

  assert(root->getName() == "body");
  assert(legA->getName() == "leg_A");

  // Backup q1, q2
  std::vector<int> id;
  id.push_back(3);
  id.push_back(4);
  Vector2d oldQ = _skel->getPositions(id);

  // Set _q1, _q2 to the skeleton
  Vector2d newQ(_q1, _q2);
  _skel->setConfig(id, newQ);

  // Compute Jacobian of momentum
  MatrixXd G1 = legA->getInertia();

//  std::cout << "G1:\n" << G1 << std::endl << std::endl;

  Isometry3d T01 = legA->getParentJoint()->getLocalTransform();
  MatrixXd   S1  = legA->getParentJoint()->getLocalJacobian();

  Isometry3d Tsc = Isometry3d::Identity();
  Tsc.translation() = _skel->getWorldCOM();
  Isometry3d Tco = Tsc.inverse() * root->getWorldTransform();

  J1 = math::dAdInvT(T01, G1 * S1);
  J1 = math::dAdInvT(Tco, J1);

  // Restore q1, q2
  _skel->setConfig(id, oldQ);

  return J1.head<3>();
}

Vector3d AutomaticController::evalMomemtumJacobian2(Skeleton* _skel,
                                                    double _q1, double _q2)
{
  // Prepare variables
  Vector6d J2 = Vector6d::Zero(6);

  BodyNode* root = _skel->getBodyNode(0);
  BodyNode* legB = _skel->getBodyNode(2);

  assert(root->getName() == "body");
  assert(legB->getName() == "leg_B");

  // Backup q1, q2
  std::vector<int> id;
  id.push_back(3);
  id.push_back(4);
  Vector2d oldQ = _skel->getPositions(id);

  // Set _q1, _q2 to the skeleton
  Vector2d newQ(_q1, _q2);
  _skel->setConfig(id, newQ);

  // Compute Jacobian of momentum
  MatrixXd G2 = legB->getInertia();

//  std::cout << "G1:\n" << G1 << std::endl << std::endl;

  Isometry3d T02 = legB->getParentJoint()->getLocalTransform();
  MatrixXd   S2  = legB->getParentJoint()->getLocalJacobian();

  Isometry3d Tsc = Isometry3d::Identity();
  Tsc.translation() = _skel->getWorldCOM();
  Isometry3d Tco = Tsc.inverse() * root->getWorldTransform();

  J2 = math::dAdInvT(T02, G2 * S2);
  J2 = math::dAdInvT(Tco, J2);

  // Restore q1, q2
  _skel->setConfig(id, oldQ);

  return J2.head<3>();
}

dart::math::Jacobian AutomaticController::evalMomentumJacobian(
    const VectorXd& _q, const Eigen::Vector3d& _position)
{
  assert(_q.size()  == 8);

  dart::math::Jacobian J = math::Jacobian::Zero(6, 8);

  Eigen::Isometry3d Tso(Eigen::AngleAxisd() * Eigen::Translation3d(_position));

  assert(Tso.translation() == _position);
  assert(Tso.linear()      == Eigen::Matrix3d::Identity());

  // Backup configuration
  VectorXd oldQ = mSkel->getPositions();

  // Compute momentum Jacobian of which configuration, _q
  mSkel->setConfig(_q);
  for (int i = 0; i < mSkel->getNumBodyNodes(); ++i)
  {
    BodyNode* body = mSkel->getBodyNode(i);
    Eigen::MatrixXd localJ
        = math::dAdInvTJac(Tso.inverse() * body->getWorldTransform(),
                           body->getInertia() * body->getBodyJacobian());

    for (int j = 0; j < body->getNumDependentGenCoords(); ++j)
    {
      int idx = body->getDependentGenCoord(j);
      J.col(idx) += localJ.col(j);
    }
  }

  // Restore configuration
  mSkel->setConfig(oldQ);

  return J;
}

math::Jacobian AutomaticController::evalMomentumJacobianAtRootCOM(const VectorXd& _q)
{
  BodyNode* body = mSkel->getRootBodyNode();
  Vector3d rootCOM = body->getWorldCOM();

  return evalMomentumJacobian(_q, rootCOM);
}

Vector6d AutomaticController::evalMomemtum(const VectorXd& _q,
                                  const VectorXd& _dq,
                                  const Vector3d& _position)
{
  assert(_q.size()  == 8);
  assert(_dq.size() == 8);

  // Backup state
  VectorXd oldX = mSkel->getState();

  // Compute momentum of which state, (_q, _dq)
  VectorXd X = VectorXd::Zero(_q.size() * 2);
  X << _q, _dq;
  mSkel->setState(X);
  Vector6d P = mSkel->getMomentum(_position);

  // Restore state
  mSkel->setState(oldX);

  return P;
}

Vector6d AutomaticController::evalMomemtumAtRootCOM(const VectorXd& _q,
                                           const VectorXd& _dq)
{
  BodyNode* body = mSkel->getRootBodyNode();
  Vector3d rootCOM = body->getWorldCOM();

  return evalMomemtum(_q, _dq, rootCOM);
}

double AutomaticController::oneStepRotation(
    Skeleton* _skel,
    double _q1, double _q2,
    double _dq1, double _dq2,
    const Vector3d& _Ho)
{
  double rotation = 0.0;

  //
  MatrixXd J0 = evalMomemtumJacobian0(_skel, _q1, _q2);
  MatrixXd J1 = evalMomemtumJacobian1(_skel, _q1, _q2);
  MatrixXd J2 = evalMomemtumJacobian2(_skel, _q1, _q2);

  assert(J0.rows() == 3);
  assert(J0.cols() == 1);

  assert(J2.rows() == 3);
  assert(J2.cols() == 1);

  assert(J2.rows() == 3);
  assert(J2.cols() == 1);

  double J0_z = J0(2, 0);
//  Vector3d dqo = J0.inverse() * (-J1 * _dq1 - J2 * _dq2 + _Ho);
  Vector3d dqo = (1.0/J0_z) * (-J1 * _dq1 - J2 * _dq2 + _Ho);

//  assert(std::fabs(dqo[0]) < 1e-6);
//  assert(std::fabs(dqo[1]) < 1e-6);

  // TODO(JS): We should other x and y component. Here both are ignored.
  rotation = dqo[2];

//  std::cout << "dqo: " << dqo.transpose() << std::endl;

//  Matrix3d R = math::expMapRot(dqo);

//  std::cout << "R: \n" << R << std::endl;

  return rotation;
}

double AutomaticController::lineRotation(
    Skeleton* _skel,
    const Vector2d& _qi, const Vector2d& _qf, int _resolution)
{
  assert(_resolution > 1);

  double dq = 0.0;

  // Compute linearly spaced vectors
  VectorXd q1 = linspace(_qi[0], _qf[0], _resolution);
  VectorXd q2 = linspace(_qi[1], _qf[1], _resolution);

  // Compute differences
  VectorXd dq1 = diff(q1);
  VectorXd dq2 = diff(q2);

  // Add line segments
  for (int i = 0; i < _resolution - 1; ++i)
    dq += oneStepRotation(_skel, q1[i+1], q2[i+1], dq1[i], dq2[i], Vector3d::Zero());

  return dq;
}

double AutomaticController::cycleRotationBigTriangle(Skeleton* _skel, double _b)
{
  double dq = 0.0;

  dq += lineRotation(_skel, Vector2d(-_b, -_b), Vector2d( _b,  _b));
  dq += lineRotation(_skel, Vector2d( _b,  _b), Vector2d(-_b,  _b));
  dq += lineRotation(_skel, Vector2d(-_b,  _b), Vector2d(-_b, -_b));

  return dq;
}

double AutomaticController::cycleRotationSmallTriangle(Skeleton* _skel, double _b)
{
  double dq = 0.0;

  dq += lineRotation(_skel, Vector2d(0.0, 0.0), Vector2d( _b, -_b));
  dq += lineRotation(_skel, Vector2d( _b, -_b), Vector2d(0.0, -_b));
  dq += lineRotation(_skel, Vector2d(0.0, -_b), Vector2d(0.0, 0.0));

  return dq;
}

double AutomaticController::solveCycleScaleBigTriangle(
    Skeleton* _skel,
    double _dq, double _b, const Vector2d& _bound, double _tol)
{
  double scale = _bound.mean();
  double error = cycleRotationBigTriangle(_skel, scale * _b) - _dq;

  if (std::fabs(error) > _tol)
  {
    if (error > 0.0)
      solveCycleScaleBigTriangle(_skel, _dq, _b, Vector2d(_bound[0], scale));
    else
      solveCycleScaleBigTriangle(_skel, _dq, _b, Vector2d(scale, _bound[1]));
  }
  else
  {
    return scale;
  }
}

double AutomaticController::solveCycleScaleSmallTriangle(
    Skeleton* _skel,
    double _dq, double _b, const Vector2d& _bound, double _tol)
{
  double scale = _bound.mean();
  double error = cycleRotationSmallTriangle(_skel, scale * _b) - _dq;

  if (std::fabs(error) > _tol)
  {
    if (error > 0.0)
      solveCycleScaleSmallTriangle(_skel, _dq, _b, Vector2d(_bound[0], scale));
    else
      solveCycleScaleSmallTriangle(_skel, _dq, _b, Vector2d(scale, _bound[1]));
  }
  else
  {
    return scale;
  }
}

Vector2d AutomaticController::computeMomentumJacobian(const Vector2d& _q)
{
  Vector2d Hdq = Vector2d::Zero();



  return Hdq;
}

double AutomaticController::computeAngleDisplacement(const Vector2d& _q,
                                            const Vector2d& _dq)
{
  // Equation (6) in the paper
  // (H0 - Hdq1 * dq1 - Hdq2 * dq2) / Hdq0

}

VectorXd AutomaticController::linspace(double _val1, double _val2, int _numPoints)
{
  assert(_numPoints > 1);

  VectorXd res = VectorXd::Zero(_numPoints);

  double delta = (_val2 - _val1) / static_cast<double>(_numPoints);

  res[0] = _val1;
  for (int i = 1; i < _numPoints - 1; ++i)
    res[i] = res[i - 1] + delta;
  res[_numPoints - 1] = _val2;

  return res;
}

VectorXd AutomaticController::diff(const VectorXd& _val)
{
  assert(_val.size() > 1);

  VectorXd res = VectorXd::Zero(_val.size() - 1);

  for (int i = 0; i < _val.size() - 1; ++i)
    res[i] = _val[i + 1] - _val[i];

  return res;
}

string AutomaticController::getControlPhaseString(
    AutomaticController::ControlPhase _cp)
{
  if (_cp == CP_FALLING)
    return string("Falling");
  else if (_cp == CP_ROLLING)
    return string("Rolling");
  else
    return string("Unknown");
}

double AutomaticController::getPredictedFallTime()
{
  // Assume that there is no external force so the COM of the inertiaBot is free
  // falling by gravity force only. Then the remaining to colliding is:
  //     arg f(t) = 0.5*a*t*t + v*t + x = 0
  double y0  = mSkel->getWorldCOM()[1];
  y0 += -0.2;  // Margin
  double dy0 = mSkel->getWorldCOMVelocity()[1];
  double ddy = mSkel->getGravity()[1];
  double t   = 0.1;
  double tol = 1e-6;

  while (std::fabs(getPredictedFallTimeCalcF(ddy, dy0, y0, t)) > tol)
  {
    t = t - getPredictedFallTimeCalcF(ddy, dy0, y0, t)
            / getPredictedFallTimeCalcDF(ddy, dy0, t);
  }

  return t;
}

double AutomaticController::getPredictedFallTimeCalcF(double _a, double _v,
                                                      double _x, double _t)
{
  return 0.5 * _a * _t * _t + _v * _t + _x;
}

double AutomaticController::getPredictedFallTimeCalcDF(double _a, double _v,
                                                       double _t)
{
  assert(std::fabs(_a * _t + _v) > 0);
  return _a * _t + _v;
}

void AutomaticController::bake()
{
  collision::CollisionDetector* cd
      = mConstraintDynamics->getCollisionDetector();

  int dof       = mSkel->getNumDofs();
  int nContacts = cd->getNumContacts();

  Eigen::VectorXd state(
        1                // time
        + dof * 2        // state
        + 2              // control force
        + 3              // center of mass
        + 3              // angular momentum
        + 3              // linear moementum
        + 6 * nContacts  // contact (contact point(3) and force(3))
        );

  // Time
  state[0] = mCurrentTime;

  // States
  state.segment(1      , dof) = mSkel->get_q();
  state.segment(1 + dof, dof) = mSkel->get_dq();

  // Torques
//  state.segment(1 + dof * 2, 2) = mSkel->get_tau().tail<2>();
  state.segment(1 + dof * 2, 2) = mTorques.segment<2>(3);

  // Center of mass
  int comBegin = 1 + dof * 2 + 2;
  state[comBegin    ] = mSkel->getWorldCOM()[1];
  state[comBegin + 1] = mSkel->getWorldCOMVelocity()[1];
  state[comBegin + 2] = mSkel->getWorldCOMAcceleration()[1];

  // Momentums
  int momBegin = 1 + dof * 2 + 2 + 3;
  Vector6d P = mSkel->getMomentum(mSkel->getWorldCOM());
  state.segment(momBegin    , 3) = P.head<3>();
  state.segment(momBegin + 3, 3) = P.tail<3>();

  // Contacts
  for (int i = 0; i < nContacts; i++)
  {
    int begin = 1 + 2 * dof + 2 + 3 + 6 + i * 6;
    state.segment(begin    , 3) = cd->getContact(i).point;
    state.segment(begin + 3, 3) = cd->getContact(i).force;
  }

  mBakedStates.push_back(state);
}

void AutomaticController::printState(const std::string& _fileName)
{
  ofstream file;
  std::string fullName = std::string(DART_DATA_PATH) + _fileName;
  file.open(fullName.c_str());

  int nFrames = mBakedStates.size();

  for (int i = 0; i < nFrames; ++i)
  {
    VectorXd states = mBakedStates[i];

    file << states.transpose() << std::endl;
  }

  file.close();

  std::string fullNameWithPlannedMotion = fullName + std::string("Key");
  file.open(fullNameWithPlannedMotion.c_str());

  int nKeys = mPlannedMotion.getNumKeyConfig();
  for (int i = 0; i < nKeys; ++i)
  {
    file << mPlannedMotion.getKeyTime(i)
         << " " << mPlannedMotion.getKeyConfig(i).transpose()
         << " " << mPlannedMotion.getKeyPredictedOrientation(i)
         << std::endl;
  }

  file.close();
}

