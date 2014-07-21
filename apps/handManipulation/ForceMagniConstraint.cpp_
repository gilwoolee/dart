#include "ForceMagniConstraint.h"
using namespace Eigen;

#include "kinematics/BodyNode.h"
#include "kinematics/Skeleton.h"
#include "dynamics/BodyNodeDynamics.h"
#include "dynamics/SkeletonDynamics.h"
#include "optimizer/Var.h"
#include "math/UtilsMath.h"

namespace optimizer {

	ForceMagniConstraint::ForceMagniConstraint(std::vector<Var *>& _var, int _forceIndex)
		: Constraint(_var), mForceIndex(_forceIndex)
	{
		mNumRows = 3;

		mWeight = VectorXd::Ones(mNumRows);
		mConstTerm = VectorXd::Zero(mNumRows);
		mCompletion = VectorXd::Zero(mNumRows);

	}

	VectorXd ForceMagniConstraint::evalCon() {
		VectorXd ret = VectorXd::Zero(3);
		double w = 1.0;

		Vector3d force(mVariables.at(mForceIndex*3+0)->mVal,mVariables.at(mForceIndex*3+1)->mVal,mVariables.at(mForceIndex*3+2)->mVal);

		ret = force; 

		return ret;
	}

	void ForceMagniConstraint::fillJac(VVD jEntry, VVB jMap, int index) {
	
	}

	void ForceMagniConstraint::fillObjGrad(std::vector<double>& dG) {
		VectorXd dP = evalCon();

		double w = 1.0;

		for (int i = 0; i < mVariables.size(); ++i)	{
			VectorXd J = VectorXd::Zero(3);
			if (mForceIndex*3+0 == i) {
				J(0) = 1.0;
				J(1) = 0.0;
				J(2) = 0.0;
				J *= w;
				dG.at(i) += dP.dot(J);
			}
			else if (mForceIndex*3+1 == i) {
				J(0) = 0.0;
				J(1) = 1.0;
				J(2) = 0.0;
				J *= w;
				dG.at(i) += dP.dot(J);
			}
			else if (mForceIndex*3+2 == i) {
				J(0) = 0.0;
				J(1) = 0.0;
				J(2) = 1.0;
				J *= w;
				dG.at(i) += dP.dot(J);
			}
		}
	}

} // namespace optimizer
