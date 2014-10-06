#include "TorqueMagniConstraint.h"
using namespace Eigen;

#include "kinematics/BodyNode.h"
#include "kinematics/Skeleton.h"
#include "dynamics/BodyNodeDynamics.h"
#include "dynamics/SkeletonDynamics.h"
#include "optimizer/Var.h"
#include "math/UtilsMath.h"

namespace optimizer {

	TorqueMagniConstraint::TorqueMagniConstraint(std::vector<Var *>& _var, Eigen::VectorXd _particularSolution, Eigen::MatrixXd _nullSpace, Eigen::VectorXd _bound, int _equality)
		: Constraint(_var), mParticularSolution(_particularSolution), mNullSpace(_nullSpace), mBound(_bound)
	{
		mNumRows = mParticularSolution.size();

		mWeight = VectorXd::Ones(mNumRows);
		mConstTerm = VectorXd::Zero(mNumRows);
		mCompletion = VectorXd::Zero(mNumRows);

		mEquality = _equality; // <=0 
	}

	VectorXd TorqueMagniConstraint::evalCon() {
		VectorXd ret = VectorXd::Zero(mParticularSolution.size());
		double w = 1.0;

		VectorXd z(mVariables.size());
		for (int i = 0; i < mVariables.size(); ++i) {
			z(i) = mVariables.at(i)->mVal;
		}

		ret = mParticularSolution + mNullSpace * z - mBound;
		return ret;
	}

	void TorqueMagniConstraint::fillJac(VVD jEntry, VVB jMap, int index) {
		double w = 1.0;

		for (int i = 0; i < mNumRows; ++i) {
			for (int j = 0; j < mVariables.size(); ++j)	{
				VectorXd J = mNullSpace.col(j);
				J /= w;
				jEntry->at(index + i)->at(j) = J(i);
				jMap->at(index + i)->at(j) = true;
			}
		}
	}

	void TorqueMagniConstraint::fillObjGrad(std::vector<double>& dG) {

	}

	void TorqueMagniConstraint::setParticularSolution(const Eigen::VectorXd _particularSolution) {
		this->mParticularSolution = _particularSolution;
	}

	Eigen::VectorXd TorqueMagniConstraint::getParticularSolution() const {
		return mParticularSolution;
	}

	void TorqueMagniConstraint::setNullSpace(const Eigen::MatrixXd _nullSpace) {
		this->mNullSpace = _nullSpace;
	}

	Eigen::MatrixXd TorqueMagniConstraint::getNullSpace() const {
		return mNullSpace;
	}

	void TorqueMagniConstraint::setBound(const Eigen::VectorXd _bound) {
		this->mBound = _bound;
	}

	Eigen::VectorXd TorqueMagniConstraint::getBound() const {
		return mBound;
	}

} // namespace optimizer
