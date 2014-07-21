#ifndef CONTACT_FORCE_PROBLEM_H
#define CONTACT_FORCE_PROBLEM_H

#include <vector>
#include <Eigen/Dense>

#include "dart/optimizer/Problem.h"

using namespace Eigen;

namespace dart {
namespace optimizer {

class Constraint;
class ForceTorqueConstraint;
class ForceProjConstraint;
class ForceAngleConstraint;
class ForceMagniConstraint;
class ForcePlaneConstraint;
class ForceSpreadConstraint;

/// class ContactForceProblem
class ContactForceProblem : public optimizer::Problem
{
public:
  /// Constructor
  ContactForceProblem(std::vector<Eigen::Vector3d>& _contactPoints,
                      std::vector<Eigen::Vector3d>& _normals,
                      Eigen::Vector3d _totalForce,
                      Eigen::Vector3d _totalTorque,
                      Eigen::Vector3d _objCOM);

  /// Destructor
  virtual ~ContactForceProblem();

  ///
  void initProblem();

  ///
  virtual void update(double* coefs);

public:
  ///
  std::vector<Eigen::Vector3d> mContactPoints;

  ///
  std::vector<Eigen::Vector3d> mNormals;

  ///
  Eigen::Vector3d mTotalForce;

  ///
  Eigen::Vector3d mTotalTorque;

  ///
  Eigen::Vector3d mObjCOM;

  ///
  std::vector<ForceTorqueConstraint*> mFTC;

  ///
  std::vector<ForceAngleConstraint*> mFAC;

  ///
  std::vector<ForceMagniConstraint*> mFMC;

  ///
  std::vector<ForcePlaneConstraint*> mFPC;

  ///
  std::vector<ForceSpreadConstraint*> mFSC;
};
} // namespace optimizer

#endif
