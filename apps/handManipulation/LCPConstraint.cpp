#include "LCPConstraint.h"
using namespace Eigen;

#include "optimizer/Var.h"
#include "math/UtilsMath.h"

namespace optimizer {

	LCPConstraint::LCPConstraint(std::vector<Var *>& _var, Eigen::MatrixXd _A, Eigen::VectorXd _b, int _equality)
		: Constraint(_var), mA(_A), mb(_b)
	{
		mNumRows = _b.size();

		mWeight = VectorXd::Ones(mNumRows);
		mConstTerm = VectorXd::Zero(mNumRows);
		mCompletion = VectorXd::Zero(mNumRows);

		mEquality = _equality;  
	}

	VectorXd LCPConstraint::evalCon() {
		VectorXd ret = VectorXd::Zero(mb.size());
		double w = 1.0;

		VectorXd x(mVariables.size());
		for (int i = 0; i < mVariables.size(); ++i) {
			x(i) = mVariables.at(i)->mVal;
		}

		ret = mA * x - mb;
		return ret;
	}

	void LCPConstraint::fillJac(VVD jEntry, VVB jMap, int index) {
		double w = 1.0;

		for (int i = 0; i < mNumRows; ++i) {
			for (int j = 0; j < mVariables.size(); ++j)	{
				VectorXd J = mA.col(j);
				J *= w;
				jEntry->at(index + i)->at(j) = J(i);
				jMap->at(index + i)->at(j) = true;
			}
		}
	}

	void LCPConstraint::fillObjGrad(std::vector<double>& dG) {
		double w = 1.0;
		VectorXd dP = evalCon();
		for (int j = 0; j < mVariables.size(); ++j)	{
			VectorXd J = mA.col(j);
			J *= w;
			dG[j] += dP.dot(J);
		}
	}

} // namespace optimizer
