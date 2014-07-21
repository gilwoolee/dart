#include "ForcePlaneConstraint.h"
using namespace Eigen;

#include <math.h>

#include "optimizer/Var.h"
#include "math/UtilsMath.h"

namespace optimizer {

	ForcePlaneConstraint::ForcePlaneConstraint(std::vector<Var *>& _var, Eigen::Vector3d _normal, int _forceIndex)
		: Constraint(_var), mNormal(_normal), mForceIndex(_forceIndex)
	{
		mNumRows = 1;

		mWeight = VectorXd::Ones(mNumRows);
		mConstTerm = VectorXd::Zero(mNumRows);
		mCompletion = VectorXd::Zero(mNumRows);

		mEquality = 0;
	}

	VectorXd ForcePlaneConstraint::evalCon() {
		VectorXd ret = VectorXd::Zero(1);
		double w = 1.0;

		Vector3d force(mVariables.at(mForceIndex*3+0)->mVal,mVariables.at(mForceIndex*3+1)->mVal,mVariables.at(mForceIndex*3+2)->mVal);
		double forceMag = pow(mVariables.at(mForceIndex*3+0)->mVal*mVariables.at(mForceIndex*3+0)->mVal+mVariables.at(mForceIndex*3+1)->mVal*mVariables.at(mForceIndex*3+1)->mVal+mVariables.at(mForceIndex*3+2)->mVal*mVariables.at(mForceIndex*3+2)->mVal,-0.5);
		double r = force.dot(mNormal)*forceMag;
		
		r = w*r;
		ret(0) = r;

		return ret;
	}

	void ForcePlaneConstraint::fillJac(VVD jEntry, VVB jMap, int index) {
		double w = 1.0;

		double forceMag = pow(mVariables.at(mForceIndex*3+0)->mVal*mVariables.at(mForceIndex*3+0)->mVal+mVariables.at(mForceIndex*3+1)->mVal*mVariables.at(mForceIndex*3+1)->mVal+mVariables.at(mForceIndex*3+2)->mVal*mVariables.at(mForceIndex*3+2)->mVal,-0.5);
		Vector3d force(mVariables.at(mForceIndex*3+0)->mVal,mVariables.at(mForceIndex*3+1)->mVal,mVariables.at(mForceIndex*3+2)->mVal);
		double forceMagDeriv = pow(mVariables.at(mForceIndex*3+0)->mVal*mVariables.at(mForceIndex*3+0)->mVal+mVariables.at(mForceIndex*3+1)->mVal*mVariables.at(mForceIndex*3+1)->mVal+mVariables.at(mForceIndex*3+2)->mVal*mVariables.at(mForceIndex*3+2)->mVal,-1.5);

		for (int i = 0; i < mVariables.size(); ++i)	{
			VectorXd J = VectorXd::Zero(1);
			if (mForceIndex*3+0 == i) {
				J(0) = mNormal(0)*forceMag-mVariables.at(mForceIndex*3+0)->mVal*force.dot(mNormal)*forceMagDeriv;
				J *= w;
				jEntry->at(index + 0)->at(i) = J(0);
				jMap->at(index + 0)->at(i) = true;
			}
			else if (mForceIndex*3+1 == i) {
				J(0) = mNormal(1)*forceMag-mVariables.at(mForceIndex*3+1)->mVal*force.dot(mNormal)*forceMagDeriv;
				J *= w;
				jEntry->at(index + 0)->at(i) = J(0);
				jMap->at(index + 0)->at(i) = true;
			}
			else if (mForceIndex*3+2 == i) {
				J(0) = mNormal(2)*forceMag-mVariables.at(mForceIndex*3+2)->mVal*force.dot(mNormal)*forceMagDeriv;
				J *= w;
				jEntry->at(index + 0)->at(i) = J(0);
				jMap->at(index + 0)->at(i) = true;
			}
		}
	}

	void ForcePlaneConstraint::fillObjGrad(std::vector<double>& dG) {
		VectorXd dP = evalCon();

		double w = 50.0;

		double forceMag = pow(mVariables.at(mForceIndex*3+0)->mVal*mVariables.at(mForceIndex*3+0)->mVal+mVariables.at(mForceIndex*3+1)->mVal*mVariables.at(mForceIndex*3+1)->mVal+mVariables.at(mForceIndex*3+2)->mVal*mVariables.at(mForceIndex*3+2)->mVal,-0.5);
		Vector3d force(mVariables.at(mForceIndex*3+0)->mVal,mVariables.at(mForceIndex*3+1)->mVal,mVariables.at(mForceIndex*3+2)->mVal);
		double forceMagDeriv = pow(mVariables.at(mForceIndex*3+0)->mVal*mVariables.at(mForceIndex*3+0)->mVal+mVariables.at(mForceIndex*3+1)->mVal*mVariables.at(mForceIndex*3+1)->mVal+mVariables.at(mForceIndex*3+2)->mVal*mVariables.at(mForceIndex*3+2)->mVal,-1.5);

		for (int i = 0; i < mVariables.size(); ++i)	{
			VectorXd J = VectorXd::Zero(1);
			if (mForceIndex*3+0 == i) {
				J(0) = mNormal(0)*forceMag-mVariables.at(mForceIndex*3+0)->mVal*force.dot(mNormal)*forceMagDeriv;
				J *= w;
				dG.at(i) += dP.dot(J);
			}
			else if (mForceIndex*3+1 == i) {
				J(0) = mNormal(1)*forceMag-mVariables.at(mForceIndex*3+1)->mVal*force.dot(mNormal)*forceMagDeriv;
				J *= w;
				dG.at(i) += dP.dot(J);
			}
			else if (mForceIndex*3+2 == i) {
				J(0) = mNormal(2)*forceMag-mVariables.at(mForceIndex*3+2)->mVal*force.dot(mNormal)*forceMagDeriv;
				J *= w;
				dG.at(i) += dP.dot(J);
			}
		}
	}

} // namespace optimizer
