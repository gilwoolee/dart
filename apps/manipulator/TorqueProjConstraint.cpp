#include "TorqueProjConstraint.h"
using namespace Eigen;

#include "optimizer/Var.h"
#include "math/UtilsMath.h"

namespace optimizer {

	TorqueProjConstraint::TorqueProjConstraint(std::vector<Var *>& _var, Eigen::VectorXd _target, Eigen::MatrixXd _jacobian)
		: Constraint(_var), mTarget(_target), mJacobian(_jacobian)
	{
		mNumRows = mTarget.size();

		mWeight = VectorXd::Ones(mNumRows);
		mConstTerm = VectorXd::Zero(mNumRows);
		mCompletion = VectorXd::Zero(mNumRows);
	}

	VectorXd TorqueProjConstraint::evalCon() {
		VectorXd ret = VectorXd::Zero(mTarget.size());
		double w = 1.0;

		VectorXd z(mVariables.size());
		for (int i = 0; i < mVariables.size(); ++i) {
			z(i) = mVariables.at(i)->mVal;
		}

		ret = mJacobian * z - mTarget;

		return ret;
	}

	void TorqueProjConstraint::fillJac(VVD jEntry, VVB jMap, int index) {
	}

	void TorqueProjConstraint::fillObjGrad(std::vector<double>& dG) {
		VectorXd dP = evalCon();

		double w = 1.0;

		for (int i = 0; i < mVariables.size(); ++i)	{
			VectorXd J = mJacobian.col(i);
			J /= w;
			dG.at(i) = dP.dot(J);
		}
	}

	void TorqueProjConstraint::setTarget(const Eigen::VectorXd _target) {
		this->mTarget = _target;
	}

	Eigen::VectorXd TorqueProjConstraint::getTarget() const {
		return mTarget;
	}

	void TorqueProjConstraint::setJacobian(const Eigen::MatrixXd _jacobian) {
		this->mJacobian = _jacobian;
	}

	Eigen::MatrixXd TorqueProjConstraint::getJacobian() const {
		return mJacobian;
	}
} // namespace optimizer
