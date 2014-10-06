#include "Controller.h"

#include <Eigen/Geometry>

#include "dart/dynamics/Skeleton.h"
#include "dart/dynamics/BodyNode.h"
#include "dart/math/Helpers.h"
//#include "optimizer/ObjectiveBox.h"
//#include "optimizer/snopt/SnoptSolver.h"
//#include "optimizer/Var.h"
//#include "ControlTorqueProblem.h"
//#include "TorqueProjConstraint.h"
//#include "TorqueMagniConstraint.h"
//#include "IKProblem.h"
//#include "PositionConstraint.h"

using namespace Eigen;
using namespace dart;
using namespace math;
using namespace dynamics;
using namespace tasks;
using namespace optimizer;

double angleX = 0.0;
double angleY = 1.0;
double angleZ = 0.0;
double handPosX = 0.0;
double handPosY = 0.0;
double handPosZ = 0.0;
double objPosX = 0.0;
double objPosY = 0.0;
double objPosZ = 0.0;

//==============================================================================
Controller::Controller(Skeleton* _skel,
                       double _t,
                       std::vector<Task*>& _tasks,
                       int _fingerNum,
                       std::vector<std::string>& _fingerRootNames,
                       std::vector<std::string>& _fingerTipNames,
                       std::string _palmName)
  : mSkel(_skel),
    mTimestep(_t),
    mTasks(_tasks),
    mFingerNum(_fingerNum),
    mFingerRootNames(_fingerRootNames),
    mFingerTipNames(_fingerTipNames),
    mPalmName(_palmName)
{
  int nDof = mSkel->getNumDofs();
  mKp = MatrixXd::Identity(nDof, nDof);
  mKd = MatrixXd::Identity(nDof, nDof);;

  mTorques.resize(nDof);
  mGravityCompensationForce.resize(nDof);
  mObjControlForce.resize(nDof);
  mTrackForce.resize(nDof);
  mDampForce.resize(nDof);
  mOriForce.resize(nDof);
  mMaintainForce.resize(nDof);
  mTaskForce.resize(nDof);
  mConstraintForce.resize(nDof);
  mFingerEndPose.resize(nDof);
  mFingerRestPose.resize(nDof);
  mFingerInterceptPose.resize(nDof);
  mDesiredDofs.resize(nDof);

  mTorques.setZero();
  mGravityCompensationForce.setZero();
  mObjControlForce.setZero();
  mTrackForce.setZero();
  mDampForce.setZero();
  mOriForce.setZero();
  mMaintainForce.setZero();
  mTaskForce.setZero();
  mConstraintForce.setZero();
  mFingerEndPose.setZero();

  float fingerRestPose[] = {-0.8000000119, 0, 0, 0, 0, 0, 0, -0.7428935766, 0, 0, 0, 0, 1.072659612, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.8000000119, 0, 0.200000003};
  float fingerInterceptPose[] = {-0.8, 0, 0, 0, 0, 0, 0, 0, 0.4, 0, 0.4, 0.4, 0, 0.6, 0, 0.6, 0.6, 0, 0.4, 0.6, 0.4, 0.4, 0, 0.4, 0};
  for (int i = 0; i < nDof; i++)
  {
    mDesiredDofs[i] = mSkel->getPosition(i);
    mFingerRestPose[i] = fingerRestPose[i];
    mFingerInterceptPose[i] = fingerInterceptPose[i];
  }

  mActiveNumFrame.resize(mFingerNum);
  mActiveSimFrame.resize(mFingerNum);
  mTrackNumFrame.resize(mFingerNum);
  mTrackSimFrame.resize(mFingerNum);
  mInContactFrame.resize(mFingerNum);
  mActiveFingers.resize(mFingerNum);
  mContactFingers.resize(mFingerNum);
  mInContactFingers.resize(mFingerNum);
  mRestFingers.resize(mFingerNum);
  mTrackFingers.resize(mFingerNum);
  mFingerDofs.resize(mFingerNum);

  for (int i = 0; i < mFingerNum; ++i) {
    mActiveNumFrame[i] = 150;
    mActiveSimFrame[i] = 0;
    mTrackNumFrame[i] = 300;
    mTrackSimFrame[i] = 0;
    mInContactFrame[i] = 0;
    mActiveFingers[i] = false;
    mContactFingers[i] = false;
    mInContactFingers[i] = false;
    mRestFingers[i] = true;
    mTrackFingers[i] = false;
  }

  mOriFlag = false;
  mOriSimFrame = 0;
  mOriNumFrame = 500;
  mMaintainFlag = false;
  mControlFlag = false;
  mTaskFlag = false;
  mOnPalmFlag = false;

  mFingerDofs[0].resize(5);
  mFingerDofs[1].resize(4);
  mFingerDofs[2].resize(4);
  mFingerDofs[3].resize(4);
  mFingerDofs[4].resize(5);
  mFingerDofs[0](0) = 7;
  mFingerDofs[0](1) = 12;
  mFingerDofs[0](2) = 17;
  mFingerDofs[0](3) = 22;
  mFingerDofs[0](4) = 24;
  mFingerDofs[1](0) = 3;
  mFingerDofs[1](1) = 8;
  mFingerDofs[1](2) = 13;
  mFingerDofs[1](3) = 18;
  mFingerDofs[2](0) = 5;
  mFingerDofs[2](1) = 10;
  mFingerDofs[2](2) = 15;
  mFingerDofs[2](3) = 20;
  mFingerDofs[3](0) = 6;
  mFingerDofs[3](1) = 11;
  mFingerDofs[3](2) = 16;
  mFingerDofs[3](3) = 21;
  mFingerDofs[4](0) = 4;
  mFingerDofs[4](1) = 9;
  mFingerDofs[4](2) = 14;
  mFingerDofs[4](3) = 19;
  mFingerDofs[4](4) = 23;

  // use three DoF at the wrist to control rolling motion
  mOriDofs.resize(3);
  mOriDofs(0) = 0;
  mOriDofs(1) = 1;
  mOriDofs(2) = 2;

  mExtNodeNum = 5;
  mExtDofNum = 3;

  mKp(0, 0) = 50;
  mKp(1, 1) = 50;
  mKp(2, 2) = 50;
  mKd(0, 0) = 20;
  mKd(1, 1) = 20;
  mKd(2, 2) = 20;

  for (int i = 3; i < nDof; i++) {
    mKp(i, i) = 15.0;
    mKd(i, i) = 2.0;
  }
  mFrame = 0;

  mPreOriTarget = Vector3d::Zero();
  mAccumulateOriError = Vector3d::Zero();
}

//==============================================================================
Controller::~Controller()
{
}

//void Controller::resetController(std::vector<tasks::Task*>& _tasks)
//{
//  mTasks = _tasks;
//}

//void Controller::evalTargetOri(const Eigen::VectorXd& _dof, const Eigen::VectorXd& _dofVel, const Eigen::VectorXd& _objDof, const Eigen::VectorXd& _objVel, const Eigen::Vector3d& _offset, const Eigen::Vector3d& _targetOri, const Eigen::Vector3d& _objOri)
//{
//  Vector3d palmOri;
//  // based on the object position with respect to palm
//  if (mOnPalmFlag && _objVel(2) < -0.1) {
//    palmOri = Vector3d(0.0,1.0,1.0).normalized();
//  }

//  else {
//    // based on the object orientation
//    Vector3d oriDir = _objOri.cross(_targetOri); // two vectors are all normalized
//    double angle = acos(_objOri.dot(_targetOri)/(_objOri.norm()*_targetOri.norm()));

//    double kp = 0.3;
//    double kd = 0.2;
//    Eigen::Quaternion<double> q;
//    q = (Eigen::Quaternion<double>)Eigen::AngleAxis<double>(kp*angle+kd*(5.0-abs(_objVel(3))), oriDir);

//    Vector3d palmInitOri = Vector3d(0.0,1.0,0.0).normalized();
//    palmOri = q*palmInitOri;
//  }

//  angleX = palmOri(0);
//  angleY = palmOri(1);
//  angleZ = palmOri(2);

//}

//==============================================================================
void Controller::evalTargetPose(const Eigen::VectorXd& _dof,
                                const Eigen::VectorXd& _dofVel,
                                const Eigen::VectorXd& _objDof,
                                const Eigen::VectorXd& _objVel,
                                const Eigen::Vector3d& _offset,
                                const Eigen::Vector3d& _targetOri,
                                const Eigen::Vector3d& _objOri)
{
  // set the desired angle for wrist and elbow to be rest
  for (int i = 0; i < mSkel->getNumDofs(); ++i)
    setDesiredDof(i,mFingerRestPose(i));

  // set pose for contact finger and active finger
  for (int i = 0; i < mFingerNum; ++i)
  {
    for (int j = 0; j < mFingerDofs[i].size(); ++j)
      setDesiredDof(mFingerDofs[i](j), mFingerEndPose[mFingerDofs[i](j)]);
  }

  // set pose for rest finger
  for (int i = 0; i < mFingerNum; ++i)
  {
    if (mRestFingers[i])
    {
      for (int j = 0; j < mFingerDofs[i].size(); ++j)
        setDesiredDof(mFingerDofs[i](j), mFingerRestPose(mFingerDofs[i](j)));
    }
  }

}

//Eigen::MatrixXd Controller::evalTaskNullSpace()
//{
//  int numRows = 0;
//  int rowIndex = 0;
//  Eigen::MatrixXd rowSpaceMatrix,nullSpaceMatrix;

//  for (int i = 0; i < mTasks.size(); ++i) {
//    numRows += mTasks.at(i)->mOmega.rows();
//  }
//  rowSpaceMatrix = MatrixXd::Zero(numRows, mSkel->getNumDofs());
//  for (int i = 0; i < mTasks.size(); ++i) {
//    for (int j = 0; j < mTasks.at(i)->mOmega.rows(); ++j) {
//      rowSpaceMatrix.row(rowIndex) = mTasks.at(i)->mOmega.row(j);
//      rowIndex ++;
//    }
//  }

//  FullPivLU<MatrixXd> lu_decomp(rowSpaceMatrix);
//  nullSpaceMatrix = lu_decomp.kernel();
//  return nullSpaceMatrix;
//}

//==============================================================================
VectorXd Controller::getTorques()
{
  return mTorques;
}

//==============================================================================
void Controller::setDesiredDof(int _index, double _val)
{
  mDesiredDofs[_index] = _val;
}

//==============================================================================
void Controller::computeTorques(const VectorXd& _dof,
                                const VectorXd& _dofVel,
                                const VectorXd& _dofAcc,
                                const VectorXd& _objDof,
                                const VectorXd& _objVel,
                                std::vector<Vector3d>& _contactPoints,
                                std::vector<Vector3d>& _contactForces,
                                std::vector<std::string>& _contactBodyNames,
                                const Vector3d& _targetOri,
                                const Vector3d& _objOri)
{
  mTorques.setZero();

  // plan related
  Eigen::Vector3d onHandOffset(0.0, -0.01, 0.04);

  std::vector<int> taskDofIndex;
  for (int i = 0; i < mFingerNum; ++i)
  {
    if (mInContactFingers[i])
    {
      for (int j = 0; j < mFingerDofs[i].size(); ++j)
        taskDofIndex.push_back(mFingerDofs[i](j));
    }
  }

  // target
  evalTargetPose(_dof, _dofVel, _objDof, _objVel,
                 onHandOffset, _targetOri, _objOri);
  // evalTargetOri(_dof,_dofVel,_objDof,_objVel,onHandOffset,_targetOri,_objOri);

  // add virtual force to prevent dropping
  Vector3d objDofVec(_objDof(0), _objDof(1), _objDof(2));
  for (int i = 0; i < mFingerNum; ++i)
  {
    if (mContactFingers[i] && !mInContactFingers[i] && !mTrackFingers[i])
    {
      BodyNode* fingerTip    = mSkel->getBodyNode(mFingerTipNames[i]);
      Vector3d  fingerTipCom = fingerTip->getWorldCOM();
      Vector3d  direction    = (objDofVec - fingerTipCom).normalized();

      _contactPoints.push_back(fingerTipCom);
      _contactForces.push_back(1.0*direction);
      _contactBodyNames.push_back(mFingerTipNames[i]);
    }
  }

  // TODO(JS): We just control palm.
  // evaluate force
//  if (mControlFlag)
//  {
//    evalObjControlForce(_contactPoints, _contactForces, _contactBodyNames);
//  }
//  else
  {
    mObjControlForce = Eigen::VectorXd::Zero(mSkel->getNumDofs());
  }

  evalTrackForce(_dof,_dofVel);

  if (mOriFlag)
  {
    for (int i = 0; i < mOriDofs.size(); ++i)
      mTrackForce(mOriDofs(i)) = 0.0;
  }

  if (mControlFlag)
  {
    // if use the tracking force as secondary force than not set the tracking force to be zero
    for (int i = 0; i < taskDofIndex.size(); ++i)
      mTrackForce(taskDofIndex[i]) = 0.0;
  }

  // not using tracking force to prevent dropping
  for (int i = 0; i < mFingerNum; ++i)
  {
    if (mContactFingers[i]
        && !mInContactFingers[i]
        && mControlFlag
        && !mTrackFingers[i])
    {
      for (int j = 0; j < mFingerDofs[i].size(); ++j)
        mTrackForce(mFingerDofs[i](j)) = 0.0;
    }
  }

  // TODO(JS): Commented out since mTaskFlag is always false for rolling control
//  if (mTaskFlag)
//  {
//    evalTaskForce();
//  }
//  else
  {
    mTaskForce = Eigen::VectorXd::Zero(mSkel->getNumDofs());
  }

  // gravity compensation and damp force
  evalGravityCompensationForce();
  evalDampForce(_dofVel);

  if (mOriFlag)
    evalOriForce(_dof,_dofVel);
  else
    mOriForce = Eigen::VectorXd::Zero(mSkel->getNumDofs());

  mTorques = mGravityCompensationForce + mObjControlForce + mDampForce + mTrackForce + mTaskForce + mOriForce;

  mFrame++;

}

//==============================================================================
void Controller::evalGravityCompensationForce()
{
  mGravityCompensationForce = mSkel->getGravityForces();
}

//==============================================================================
//void Controller::evalObjControlForce(
//    const std::vector<Eigen::Vector3d>& _contactPoints,
//    const std::vector<Eigen::Vector3d>& _contactForces,
//    const std::vector<std::string>& _contactBodyNames)
//{
//  Eigen::VectorXd tempExtForce = mSkel->getExternalForces();
//  mSkel->clearExternalForces();

//  for (size_t i = 0; i < _contactForces.size(); ++i)
//  {
//    BodyNode* contactBody = mSkel->getBodyNode(_contactBodyNames[i]);
//    contactBody->addExtForce(_contactForces[i], _contactPoints[i], false, false);
//  }

//  mJVec.clear();
//  mInvJVec.clear();

//  // only one contact for each link
//  for (size_t i = 0; i < _contactForces.size(); ++i)
//  {
//    size_t    dof = mSkel->getNumDofs();
//    MatrixXd  J   = MatrixXd::Zero(3, dof);

//    BodyNode*  contactBody = mSkel->getBodyNode(_contactBodyNames[i]);
//    int        numDepDofs  = contactBody->getNumDependentGenCoords();
//    Isometry3d W           = contactBody->getTransform();

//    // do not consider the dof at wrist
//    for(int j = mExtDofNum; j < numDepDofs; j++)
//    {
//      int colIndex = contactBody->getDependentDof(j);
//      Matrix4d Wq = contactBody->getDerivWorldTransform(j);

//      // only one contact for each link
//      Vector3d point = (mSkel->getBodyNode(_contactBodyNames[i]))->mContacts.at(0).first;
//      J.col(colIndex) = dart_math::xformHom(Wq,point);
//    }
//    mJVec.push_back(J);
//  }

//  // evaluate the generalized inverse of Jacobian
//  for (int i = 0; i < _contactForces.size(); ++i) {
//    MatrixXd J = mJVec[i];
//    MatrixXd InvJ = J.transpose()*((J*J.transpose()).inverse());
//    mInvJVec.push_back(InvJ);
//  }

//  mCombinedJ = MatrixXd::Zero(mSkel->getNumDofs(), 3*_contactForces.size());
//  for (int i = 0; i < _contactForces.size(); ++i)
//  {
//    for (int k = 0; k < 3; ++k)
//      mCombinedJ.col(i*3+k) = mInvJVec[i].col(k);
//  }

//  // the transpose of combined general inverse
//  mCombinedJTrans = mCombinedJ.transpose();

//  mSkel->evalExternalForcesPartial(false, mExtNodeNum, mExtDofNum);

//  // if not evaluate the object control using virtual force, then comment
//  mObjControlForce = mSkel->getExternalForces();
//  mSkel->clearExternalForces();
//  mSkel->setExternalForces(tempExtForce);
//}

//==============================================================================
void Controller::evalTrackForce(const Eigen::VectorXd& _dof,
                                const Eigen::VectorXd& _dofVel)
{
  // other force
  Eigen::VectorXd otherForce;
  otherForce = mSkel->getExternalForces();
  otherForce += mGravityCompensationForce
                + mObjControlForce
                + mOriForce
                + mDampForce
                + mMaintainForce
                + mConstraintForce;

  // track a pose using SPD
  MatrixXd invM = (mSkel->getMassMatrix() + mKd * mTimestep).inverse();
  VectorXd p = -mKp * (_dof + _dofVel * mTimestep - mDesiredDofs);
  VectorXd d = -mKd * _dofVel;
  VectorXd qddot = invM * (-mSkel->getCombinedVector() + p + d + otherForce);

  mTrackForce = p + d - mKd * qddot * mTimestep;
}

//===============================================================================
void Controller::evalDampForce(const Eigen::VectorXd& _dofVel)
{
  // other force
  Eigen::VectorXd otherForce;
  otherForce = mSkel->getExternalForces();
  otherForce += mGravityCompensationForce
                + mObjControlForce
                + mOriForce
                + mDampForce
                + mMaintainForce
                + mConstraintForce;

  // damp using SPD
  MatrixXd invM = (mSkel->getMassMatrix() + mKd * mTimestep).inverse();
  VectorXd p = -mKp * (_dofVel * mTimestep);
  VectorXd d = -mKd * _dofVel;
  VectorXd qddot = invM * (-mSkel->getCombinedVector() + p + d + otherForce);
  mDampForce = 0.05 * (p + d - mKd * qddot * mTimestep);
}

//==============================================================================
//void Controller::evalTaskForce()
//{
//  for (int i = 0; i < mTasks.size(); ++i)
//    mTasks[i]->evalTorque();

//  Eigen::VectorXd taskTorque = VectorXd::Zero(mSkel->getNumDofs());
//  int numRemainTask = mTasks.size();

//  // the vector stores the torque for each task
//  std::vector<Eigen::VectorXd> torqueStars;
//  torqueStars.resize(mTasks.size());

//  // the vector stores the inconsistency for each task
//  std::vector<double> inconsistencies;
//  inconsistencies.resize(mTasks.size());

//  bool consistency = true;
//  bool running = true;

//  while (running)
//  {
//    running = false;

//    // only one task
//    if (numRemainTask == 1)
//    {
//      MatrixXd nullSpaceMatrix = MatrixXd::Identity(mSkel->getNumDofs(), mSkel->getNumDofs());
//      ControlTorqueProblem prob(mTasks, 0, mTasks[0]->mTorque, nullSpaceMatrix, mTasks[0]->mNullSpace);
//      snopt::SnoptSolver solver(&prob);
//      solver.solve();
//      VectorXd z = solver.getState();
//      taskTorque = mTasks[0]->mTorque + mTasks[0]->mNullSpace*z;
//    }
//    // multiple tasks
//    else
//    {
//      for (int i = 0; i < numRemainTask; ++i)
//      {
//        torqueStars.at(i) = VectorXd::Zero(mSkel->getNumDofs());
//        inconsistencies.at(i) = 0.0;

//        // calculate the null space matrix for other tasks
//        int numRows = 0;
//        for (int j = 0; j < numRemainTask; ++j)
//        {
//          if (i == j)
//            continue;
//          else
//            numRows += mTasks[j]->mOmega.rows();
//        }

//        MatrixXd rowSpaceMatrix = MatrixXd::Zero(numRows, mSkel->getNumDofs());
//        int rowIndex = 0;

//        for (int j = 0; j < numRemainTask; ++j)
//        {
//          if (i == j)
//          {
//            continue;
//          }
//          else
//          {
//            for (int k = 0; k < mTasks[j]->mOmega.rows(); ++k)
//            {
//              rowSpaceMatrix.row(rowIndex) = mTasks[j]->mOmega.row(k);
//              rowIndex ++;
//            }
//          }
//        }

//        FullPivLU<MatrixXd> lu_decomp(rowSpaceMatrix);
//        MatrixXd nullSpaceMatrix = lu_decomp.kernel();

//        // calculate the control torque
//        // if other tasks have intersection
//        if (nullSpaceMatrix.cols() >= 2)
//        {
//          // calculate control torque using control torque optimization
//          ControlTorqueProblem prob(mTasks, i, mTasks[i]->mTorque, nullSpaceMatrix, mTasks[i]->mNullSpace);
//          snopt::SnoptSolver solver(&prob);
//          solver.solve();
//          VectorXd z = solver.getState();
//          torqueStars.at(i) = mTasks[i]->mTorque + mTasks[i]->mNullSpace*z;

//          // evaluate the inconsistency
//          MatrixXd matrixA = nullSpaceMatrix*(nullSpaceMatrix.transpose()*nullSpaceMatrix).inverse()*nullSpaceMatrix.transpose()*mTasks[i]->mNullSpace-mTasks[i]->mNullSpace;
//          VectorXd vectorb = -nullSpaceMatrix*(nullSpaceMatrix.transpose()*nullSpaceMatrix).inverse()*nullSpaceMatrix.transpose()*mTasks[i]->mTorque+mTasks[i]->mTorque;
//          inconsistencies.at(i) = (matrixA*z - vectorb).norm();
//          // 					std::cout << "task " << i << " inconsistency: " << inconsistencies.at(i) << std::endl;
//        }
//        else
//        {
//          std::cout << "Give up the task having lowest priority according to tasks conflicts with other except task " << i << "." << std::endl;
//          getchar();
//          consistency = false;
//          numRemainTask--;
//          running = true;
//          break;
//          torqueStars.at(i) = mTasks[i]->mTorque;
//        }

//        // evaluate inconsistency
//        double totalInconsistency = 0.0;
//        for (int j = 0; j <= i; ++j) { // sum up the inconsistency for the current evaluated tasks
//          totalInconsistency += inconsistencies.at(j);
//        }
//        if (totalInconsistency > 60.0) {
//          std::cout << "Give up the task having lowest priority according to total consistency beyond threshold." << std::endl;
//          std::cout << "The consistency for each task is:" << std::endl;
//          for (int j = 0; j <= i; ++j) {
//            std::cout << "Active task " << j << ":  " << inconsistencies.at(j) << std::endl;
//          }
//          consistency = false;
//          numRemainTask--;
//          running = true;
//          break;
//        }
//      }
//    }
//  }

//  // if multitasks, then sum up all torque star to get control torque
//  if (mTasks.size() > 1)
//  {
//    for (int i = 0; i < numRemainTask; ++i)
//      taskTorque += torqueStars.at(i);
//  }

//  mTaskForce = taskTorque;
//}

//==============================================================================
void Controller::evalOriForce(const Eigen::VectorXd& _dof,
                              const Eigen::VectorXd& _dofVel)
{
  BodyNode*   wrist     = mSkel->getBodyNode(mPalmName.c_str());
  std::string wristName = wrist->getName();
  tasks::TrackOriTask* trackWristOri = new tasks::TrackOriTask(mSkel,
                                                               wristName,
                                                               "trackWristOri");
  Vector3d angle = Eigen::Vector3d(angleX, angleY, angleZ).normalized();
  trackWristOri->setTarget(angle);

  // state of hand
  Eigen::VectorXd state(_dof.size()+_dofVel.size());
  state.head(_dof.size()) = _dof;
  state.tail(_dofVel.size()) = _dofVel;

  // other force
  Eigen::VectorXd otherForce = Eigen::VectorXd::Zero(mSkel->getNumDofs());
  otherForce = mSkel->getExternalForces();
  otherForce += mGravityCompensationForce
                + mObjControlForce
                + mTrackForce
                + mDampForce
                + mMaintainForce
                + mConstraintForce;
  trackWristOri->updateTask(state, angle, otherForce);

  //
  double taskError = (mPreOriTarget - trackWristOri->getTarget()).norm();

  //
  if (math::isZero(taskError))
  {
    mAccumulateOriError = mAccumulateOriError
                          + trackWristOri->evalTaskError() * mTimestep;
  }
  else
  {
    mAccumulateOriError = Eigen::Vector3d::Zero();
    mPreOriTarget = trackWristOri->getTarget();
    //getchar();
  }

  //
  trackWristOri->setAccumulateError(mAccumulateOriError);

  //
  trackWristOri->evalTorque();

  //
  mOriForce = trackWristOri->mTorque;
}

//void Controller::evalMaintainForce(const Eigen::VectorXd& _dof, const Eigen::VectorXd& _dofVel)
//{
//  int wristIndex = 1;
//  tasks::MaintainTask *maintainWrist = new tasks::MaintainTask(mSkel,wristIndex,"maintainWrist");
//  maintainWrist->setTarget(mSkel->getBodyNode(wristIndex)->getWorldCOM());

//  // state of hand
//  Eigen::VectorXd state(_dof.size()+_dofVel.size());
//  state.head(_dof.size()) = _dof;
//  state.tail(_dofVel.size()) = _dofVel;

//  // other force
//  Eigen::VectorXd otherForce;
//  otherForce = mSkel->getExternalForces();
//  otherForce += mGravityCompensationForce+mObjControlForce+mTrackForce+mDampForce+mOriForce+mConstraintForce;
//  maintainWrist->updateTask(state,Eigen::Vector3d(0.0,0.0,0.0),otherForce);

//  maintainWrist->evalTorque();
//  mMaintainForce = maintainWrist->mTorque;
//}
