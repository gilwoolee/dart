#include "InEllipsoidConstraint.h"
using namespace Eigen;

#include "optimizer/Var.h"
#include "math/UtilsMath.h"

namespace optimizer {

	InEllipsoidConstraint::InEllipsoidConstraint(std::vector<Var *>& _var, Eigen::Vector3d _featurePoint, int _equality)
		: Constraint(_var), mFeaturePoint(_featurePoint)
	{
		mNumRows = 1;

		mWeight = VectorXd::Ones(mNumRows);
		mConstTerm = VectorXd::Zero(mNumRows);
		mCompletion = VectorXd::Zero(mNumRows);

		mEquality = _equality; // <=0 
	}

	VectorXd InEllipsoidConstraint::evalCon() {
		VectorXd ret = VectorXd::Zero(1);
		double w = 1.0;

		if (mVariables.size() == 6) {
			for (int i = 0; i < 3; ++i) {
				ret(0) += (mFeaturePoint(i)-mVariables.at(3+i)->mVal)*(mFeaturePoint(i)-mVariables.at(3+i)->mVal)/(mVariables.at(i)->mVal*mVariables.at(i)->mVal);
			}
		}

		ret(0) -= 1.0;
		return ret;
	}

	void InEllipsoidConstraint::fillJac(VVD jEntry, VVB jMap, int index) {
		double w = 1.0;

		if (mVariables.size() == 6) {
			for (int i = 0; i < 6; ++i) {
				VectorXd J = VectorXd::Zero(1);
				if (i < 3) {
					J(0) = -2.0*((mFeaturePoint(i)-mVariables.at(3+i)->mVal)*(mFeaturePoint(i)-mVariables.at(3+i)->mVal))/(mVariables.at(i)->mVal*mVariables.at(i)->mVal*mVariables.at(i)->mVal);
				}
				else {
					J(0) = -2.0*(mFeaturePoint(i-3)-mVariables.at(i)->mVal)/(mVariables.at(i-3)->mVal*mVariables.at(i-3)->mVal);
				}
				J /= w;
				jEntry->at(index)->at(i) = J(0);
				jMap->at(index)->at(i) = true;
			}
		}
	}

	void InEllipsoidConstraint::fillObjGrad(std::vector<double>& dG) {

	}

} // namespace optimizer
