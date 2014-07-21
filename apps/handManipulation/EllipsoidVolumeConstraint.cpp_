#include "EllipsoidVolumeConstraint.h"
using namespace Eigen;

#include "kinematics/BodyNode.h"
#include "kinematics/Skeleton.h"
#include "dynamics/BodyNodeDynamics.h"
#include "dynamics/SkeletonDynamics.h"
#include "optimizer/Var.h"
#include "math/UtilsMath.h"

namespace optimizer {

	EllipsoidVolumeConstraint::EllipsoidVolumeConstraint(std::vector<Var *>& _var)
		: Constraint(_var)
	{
		mNumRows = 1;

		mWeight = VectorXd::Ones(mNumRows);
		mConstTerm = VectorXd::Zero(mNumRows);
		mCompletion = VectorXd::Zero(mNumRows);

	}

	VectorXd EllipsoidVolumeConstraint::evalCon() {
		VectorXd ret = VectorXd::Zero(1);
		double w = 1.0;

		double a = mVariables.at(0)->mVal;
		double b = mVariables.at(1)->mVal;
		double c = mVariables.at(2)->mVal;

		ret(0) = 4.0*M_PI*a*b*c/3.0;

		return ret;
	}

	void EllipsoidVolumeConstraint::fillJac(VVD jEntry, VVB jMap, int index) {
	
	}

	void EllipsoidVolumeConstraint::fillObjGrad(std::vector<double>& dG) {
		VectorXd dP = evalCon();

		double w = 1.0;

		double a = mVariables.at(0)->mVal;
		double b = mVariables.at(1)->mVal;
		double c = mVariables.at(2)->mVal;

		for (int i = 0; i < 3; ++i)	{
			VectorXd J = VectorXd::Zero(3);
			if (i == 0) {
				J(0) = 4.0*M_PI*b*c/3.0;
			}
			else if (i == 1) {
				J(0) = 4.0*M_PI*a*c/3.0;
			}
			else if (i == 2) {
				J(0) = 4.0*M_PI*a*b/3.0;
			}
			J /= w;
			dG.at(i) += dP.dot(J);
		}
	}

} // namespace optimizer
