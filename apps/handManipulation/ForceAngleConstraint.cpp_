#include "ForceAngleConstraint.h"
using namespace Eigen;

#include <math.h>

#include "optimizer/Var.h"
#include "math/UtilsMath.h"

namespace optimizer {

	ForceAngleConstraint::ForceAngleConstraint(std::vector<Var *>& _var, Eigen::Vector3d _normal, int _forceIndex, double _weight)
		: Constraint(_var), mNormal(_normal), mForceIndex(_forceIndex), mAngleWeight(_weight)
	{
		mNumRows = 1;

		mWeight = VectorXd::Ones(mNumRows);
		mConstTerm = VectorXd::Zero(mNumRows);
		mCompletion = VectorXd::Zero(mNumRows);
	}

	VectorXd ForceAngleConstraint::evalCon() {
		VectorXd ret = VectorXd::Zero(1);
		double w = mAngleWeight;

		Vector3d force(mVariables.at(mForceIndex*3+0)->mVal,mVariables.at(mForceIndex*3+1)->mVal,mVariables.at(mForceIndex*3+2)->mVal);
		double forceMag = pow(mVariables.at(mForceIndex*3+0)->mVal*mVariables.at(mForceIndex*3+0)->mVal+mVariables.at(mForceIndex*3+1)->mVal*mVariables.at(mForceIndex*3+1)->mVal+mVariables.at(mForceIndex*3+2)->mVal*mVariables.at(mForceIndex*3+2)->mVal,-0.5);
		double r = force.dot(mNormal)*forceMag;
		r = -r;

		r = w*r;
		ret(0) = r;

		return ret;
	}

	double ForceAngleConstraint::evalObj() {
		VectorXd constr = evalCon();
		return constr(0);
	}

	void ForceAngleConstraint::fillJac(VVD jEntry, VVB jMap, int index) {
	}

	void ForceAngleConstraint::fillObjGrad(std::vector<double>& dG) {
		double w = mAngleWeight;

		double forceMag = pow(mVariables.at(mForceIndex*3+0)->mVal*mVariables.at(mForceIndex*3+0)->mVal+mVariables.at(mForceIndex*3+1)->mVal*mVariables.at(mForceIndex*3+1)->mVal+mVariables.at(mForceIndex*3+2)->mVal*mVariables.at(mForceIndex*3+2)->mVal,-0.5);
		Vector3d force(mVariables.at(mForceIndex*3+0)->mVal,mVariables.at(mForceIndex*3+1)->mVal,mVariables.at(mForceIndex*3+2)->mVal);
		double forceMagDeriv = pow(mVariables.at(mForceIndex*3+0)->mVal*mVariables.at(mForceIndex*3+0)->mVal+mVariables.at(mForceIndex*3+1)->mVal*mVariables.at(mForceIndex*3+1)->mVal+mVariables.at(mForceIndex*3+2)->mVal*mVariables.at(mForceIndex*3+2)->mVal,-1.5);

		for (int i = 0; i < mVariables.size(); ++i)	{
			VectorXd J = VectorXd::Zero(1);
			if (mForceIndex*3+0 == i) {
				J(0) = mNormal(0)*forceMag-mVariables.at(mForceIndex*3+0)->mVal*force.dot(mNormal)*forceMagDeriv;
				J(0) = -J(0);
				J *= w;
				dG.at(i) += J(0);
			}
			else if (mForceIndex*3+1 == i) {
				J(0) = mNormal(1)*forceMag-mVariables.at(mForceIndex*3+1)->mVal*force.dot(mNormal)*forceMagDeriv;
				J(0) = -J(0);
				J *= w;
				dG.at(i) += J(0);
			}
			else if (mForceIndex*3+2 == i) {
				J(0) = mNormal(2)*forceMag-mVariables.at(mForceIndex*3+2)->mVal*force.dot(mNormal)*forceMagDeriv;
				J(0) = -J(0);
				J *= w;
				dG.at(i) += J(0);
			}
		}
	}

} // namespace optimizer
