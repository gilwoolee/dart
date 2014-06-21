#include "ContactForceProblem.h"

#include <iostream>

#include "dart/optimizer/Function.h"
#include "dart/optimizer/Solver.h"
#include "dart/utils/Paths.h"
#include "dart/math/Helpers.h"

#include "ForceTorqueConstraint.h"
#include "ForceProjConstraint.h"
#include "ForceAngleConstraint.h"
#include "ForceMagniConstraint.h"
#include "ForcePlaneConstraint.h"
#include "ForceSpreadConstraint.h"

using namespace std;
using namespace Eigen;

namespace dart {
namespace optimizer {

//==============================================================================
ContactForceProblem::ContactForceProblem(
    std::vector<Eigen::Vector3d>& _contactPoints,
    std::vector<Eigen::Vector3d>& _normals, Eigen::Vector3d _totalForce,
    Eigen::Vector3d _totalTorque,
    Eigen::Vector3d _objCOM)
  : Problem()
{
  mContactPoints = _contactPoints;
  mNormals = _normals;
  mTotalForce = _totalForce;
  mTotalTorque = _totalTorque;
  mObjCOM = _objCOM;
  initProblem();
}

//==============================================================================
ContactForceProblem::~ContactForceProblem()
{
  for (int i = 0; i < mFTC.size(); ++i)
    delete mFTC[i];
  mFTC.clear();

  for (int i = 0; i < mFPC.size(); ++i)
    delete mFPC[i];
  mFPC.clear();

  for (int i = 0; i < mFAC.size(); ++i)
    delete mFAC[i];
  mFAC.clear();

  for (int i = 0; i < mFMC.size(); ++i)
    delete mFMC[i];
  mFMC.clear();

  for (int i = 0; i < mFSC.size(); ++i)
    delete mFSC[i];
  mFSC.clear();
}

//==============================================================================
void ContactForceProblem::initProblem()
{
  int numContact = mContactPoints.size();

  // add variables
  for (int i = 0; i < numContact*3; i++)
    addVariable(1.0, -50.0, 50.0);

  // Create Con and Obj Boxes
  createBoxes();

  Eigen::MatrixXd A;
  Eigen::VectorXd b;

  // constrain force and torque

  A = Eigen::MatrixXd::Zero(6,3*numContact);
  b = Eigen::VectorXd::Zero(6);
  Eigen::Vector3d objCOM = mObjCOM;

  for (int i = 0; i < numContact; ++i)
  {
    A.block(0,3*i,3,3) = Eigen::MatrixXd::Identity(3,3);
    A.block(3,3*i,3,3) = dart_math::makeSkewSymmetric(mContactPoints[i]-objCOM);
  }

  b.head(3) = mTotalForce;
  b.tail(3) = mTotalTorque;


  // constrain torque
  /*
    A = Eigen::MatrixXd::Zero(3,3*numContact);
    b = Eigen::VectorXd::Zero(3);
    Eigen::Vector3d objCOM = mObjCOM;

    for (int i = 0; i < numContact; ++i) {
      A.block(0,3*i,3,3) = dart_math::makeSkewSymmetric(mContactPoints[i]-objCOM);
    }

    b = mTotalTorque;
    */

  ForceTorqueConstraint* mFT = new ForceTorqueConstraint(this->vars(),A,b,0);
  mFTC.push_back(mFT);
  conBox()->add(mFT);

  for (int i = 0; i < numContact; ++i)
  {
    ForceAngleConstraint* fac = new ForceAngleConstraint(this->vars(),mNormals[i],i,50.0);
    mFAC.push_back(fac);
    objBox()->add(fac);
  }

  for (int i = 0; i < numContact; ++i)
  {
    ForceMagniConstraint* fmc = new ForceMagniConstraint(this->vars(),i);
    mFMC.push_back(fmc);
    objBox()->add(fmc);
  }

  // example of spread force among finger
  /*
    if (numContact == 3) {
      ForceSpreadConstraint* fsc = new ForceSpreadConstraint(this->vars(),1,2);
      mFSC.push_back(fsc);
      objBox()->add(fsc);
    }
    */
}

//==============================================================================
void ContactForceProblem::update(double* coefs)
{
}

}  // namespace optimizer
}  // namespace dart

