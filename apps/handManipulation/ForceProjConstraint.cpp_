#include "ForceProjConstraint.h"
using namespace Eigen;

#include "optimizer/Var.h"
#include "math/UtilsMath.h"

namespace optimizer {

	ForceProjConstraint::ForceProjConstraint(std::vector<Var *>& _var, Eigen::Vector3d _normal, int _forceIndex)
		: Constraint(_var), mNormal(_normal), mForceIndex(_forceIndex)
	{
		mNumRows = 3;

		mWeight = VectorXd::Ones(mNumRows);
		mConstTerm = VectorXd::Zero(mNumRows);
		mCompletion = VectorXd::Zero(mNumRows);
	}

	VectorXd ForceProjConstraint::evalCon() {
		VectorXd ret = VectorXd::Zero(3);
		double w = 1.0;

		Vector3d force(mVariables.at(mForceIndex*3+0)->mVal,mVariables.at(mForceIndex*3+1)->mVal,mVariables.at(mForceIndex*3+2)->mVal);
		ret = force.dot(mNormal)*mNormal-force;

		return ret;
	}

	void ForceProjConstraint::fillJac(VVD jEntry, VVB jMap, int index) {
	}

	void ForceProjConstraint::fillObjGrad(std::vector<double>& dG) {
		VectorXd dP = evalCon();

		double w = 1.0;

		for (int i = 0; i < mVariables.size(); ++i)	{
			VectorXd J = VectorXd::Zero(3);
			if (mForceIndex*3+0 == i) {
				J(0) = mNormal(0)*mNormal(0)-1;
				J(1) = mNormal(1)*mNormal(0);
				J(2) = mNormal(2)*mNormal(0);
			}
			else if (mForceIndex*3+1 == i) {
				J(0) = mNormal(0)*mNormal(1);
				J(1) = mNormal(1)*mNormal(1)-1;
				J(2) = mNormal(2)*mNormal(1);
			}
			else if (mForceIndex*3+2 == i) {
				J(0) = mNormal(0)*mNormal(2);
				J(1) = mNormal(1)*mNormal(2);
				J(2) = mNormal(2)*mNormal(2)-1;
			}
			J /= w;
			dG.at(i) += dP.dot(J);
		}
	}

} // namespace optimizer
