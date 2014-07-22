#include "ForceSpreadConstraint.h"
using namespace Eigen;

#include <math.h>

#include "optimizer/Var.h"
#include "math/UtilsMath.h"

namespace optimizer {

	ForceSpreadConstraint::ForceSpreadConstraint(std::vector<Var *>& _var, int _firstForceIndex, int _secondForceIndex)
		: Constraint(_var), mFirstForceIndex(_firstForceIndex), mSecondForceIndex(_secondForceIndex)
	{
		mNumRows = 1;

		mWeight = VectorXd::Ones(mNumRows);
		mConstTerm = VectorXd::Zero(mNumRows);
		mCompletion = VectorXd::Zero(mNumRows);
	}

	VectorXd ForceSpreadConstraint::evalCon() {
		VectorXd ret = VectorXd::Zero(1);
		double w = 0.1;

		double firstForceMag = mVariables.at(mFirstForceIndex*3+0)->mVal*mVariables.at(mFirstForceIndex*3+0)->mVal+mVariables.at(mFirstForceIndex*3+1)->mVal*mVariables.at(mFirstForceIndex*3+1)->mVal+mVariables.at(mFirstForceIndex*3+2)->mVal*mVariables.at(mFirstForceIndex*3+2)->mVal;
		double secondForceMag = mVariables.at(mSecondForceIndex*3+0)->mVal*mVariables.at(mSecondForceIndex*3+0)->mVal+mVariables.at(mSecondForceIndex*3+1)->mVal*mVariables.at(mSecondForceIndex*3+1)->mVal+mVariables.at(mSecondForceIndex*3+2)->mVal*mVariables.at(mSecondForceIndex*3+2)->mVal;
		double r = firstForceMag-secondForceMag;
		
		r = w*r;
		ret(0) = r;

		return ret;
	}

	void ForceSpreadConstraint::fillJac(VVD jEntry, VVB jMap, int index) {
	}

	void ForceSpreadConstraint::fillObjGrad(std::vector<double>& dG) {
		VectorXd dP = evalCon();
		double w = 0.1;

		for (int i = 0; i < mVariables.size(); ++i)	{
			VectorXd J = VectorXd::Zero(1);
			if (mFirstForceIndex*3+0 == i) {
				J(0) = mVariables.at(mFirstForceIndex*3+0)->mVal;
				J *= w;
				dG.at(i) += dP.dot(J);
			}
			else if (mFirstForceIndex*3+1 == i) {
				J(0) = mVariables.at(mFirstForceIndex*3+1)->mVal;
				J *= w;
				dG.at(i) += dP.dot(J);
			}
			else if (mFirstForceIndex*3+2 == i) {
				J(0) = mVariables.at(mFirstForceIndex*3+2)->mVal;
				J *= w;
				dG.at(i) += dP.dot(J);
			}
			else if (mSecondForceIndex*3+0 == i) {
				J(0) = mVariables.at(mSecondForceIndex*3+0)->mVal;
				J *= w;
				dG.at(i) += dP.dot(J);
			}
			else if (mSecondForceIndex*3+1 == i) {
				J(0) = mVariables.at(mSecondForceIndex*3+1)->mVal;
				J *= w;
				dG.at(i) += dP.dot(J);
			}
			else if (mSecondForceIndex*3+2 == i) {
				J(0) = mVariables.at(mSecondForceIndex*3+2)->mVal;
				J *= w;
				dG.at(i) += dP.dot(J);
			}
		}
	}

} // namespace optimizer
