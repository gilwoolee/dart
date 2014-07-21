#include "ControlTorqueProblem.h"

#include <iostream>
using namespace std;

#include "kinematics/FileInfoSkel.hpp"
#include "kinematics/Skeleton.h"
#include "kinematics/BodyNode.h"
#include "kinematics/Marker.h"
#include "dynamics/BodyNodeDynamics.h"
#include "dynamics/SkeletonDynamics.h"
using namespace kinematics;
using namespace dynamics;

#include "optimizer/Var.h"
#include "optimizer/ObjectiveBox.h"
#include "optimizer/ConstraintBox.h"
using namespace optimizer;
#include "TorqueProjConstraint.h"
#include "TorqueMagniConstraint.h"
#include "utils/Paths.h"

using namespace Eigen;

namespace optimizer {
	ControlTorqueProblem::ControlTorqueProblem(std::vector<tasks::Task*>& _tasks, int _taskIndex, Eigen::VectorXd _particularSolution, Eigen::MatrixXd _otherNullSpace, Eigen::MatrixXd _thisNullSpace)
		: Problem() {
			mTasks = _tasks;
			mTaskIndex = _taskIndex;
			mParticularSolution = _particularSolution;
			mOtherNullSpace = _otherNullSpace;
			mThisNullSpace = _thisNullSpace;
			initProblem();
	}

	ControlTorqueProblem::~ControlTorqueProblem() {
		delete mTPC;
		delete mUTMC;
		delete mLTMC;
	}

	void ControlTorqueProblem::initProblem() {

		// add variables
		for (int i = 0; i < mThisNullSpace.cols(); i++) {
			addVariable(0.0, -1000.0, 1000.0);
		}
// 		LOG(INFO) << "add # " << getNumVariables() << " Variables";

		// Create Con and Obj Boxes
		createBoxes();

		MatrixXd matrixA = mOtherNullSpace*(mOtherNullSpace.transpose()*mOtherNullSpace).inverse()*mOtherNullSpace.transpose()*mThisNullSpace-mThisNullSpace;
		VectorXd vectorb = -mOtherNullSpace*(mOtherNullSpace.transpose()*mOtherNullSpace).inverse()*mOtherNullSpace.transpose()*mParticularSolution+mParticularSolution;

		// add constraints
		mTPC = new TorqueProjConstraint(this->vars(),vectorb,matrixA);
		objBox()->add(mTPC);

		VectorXd upperBound(mParticularSolution.size());
		VectorXd lowerBound(mParticularSolution.size());

		for (int i = 0; i < mParticularSolution.size(); ++i) {
			upperBound(i) = 10.0;
			lowerBound(i) = -10.0;
		}
// 		upperBound(14) = 20.0;
// 		lowerBound(14) = -20.0;
// 		upperBound(15) = 1.0;
// 		lowerBound(15) = -1.0;
// 		upperBound(16) = 1.0;
// 		lowerBound(16) = -1.0;
// 		upperBound(17) = 1.0;
// 		lowerBound(17) = -1.0;

		mUTMC = new TorqueMagniConstraint(this->vars(), mParticularSolution, mThisNullSpace, upperBound, -1); // the bound for each task should depend on the task
		mLTMC = new TorqueMagniConstraint(this->vars(), mParticularSolution, mThisNullSpace, lowerBound, 1);  // the bound for each task should depend on the task

		conBox()->add(mUTMC);
		conBox()->add(mLTMC);

// 		LOG(INFO) << "# Constraints = " << conBox()->getNumConstraints();
// 		LOG(INFO) << "# Objectives = " << objBox()->getNumConstraints();
// 
// 		LOG(INFO) << "initProblem OK";

	}

	void ControlTorqueProblem::update(double* coefs) {

	}

} // namespace optimizer
