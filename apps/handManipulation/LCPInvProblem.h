#ifndef LCP_INV_PROBLEM_H
#define LCP_INV_PROBLEM_H

#include <vector>
#include <Eigen/Dense>
#include "optimizer/Problem.h"

using namespace Eigen;

namespace optimizer {
	class Constraint;
	class LCPConstraint;

	class LCPInvProblem : public optimizer::Problem {
	public:
		LCPInvProblem(Eigen::MatrixXd& _A, Eigen::VectorXd& _b, std::vector<bool>& _incontact);
		virtual ~LCPInvProblem();

		void initProblem();
		virtual void update(double* coefs);

	public:
		Eigen::MatrixXd A;
		Eigen::VectorXd b;
		std::vector<bool> inContactIndices;

		LCPConstraint * mLCPC;
	};
} // namespace optimizer

#endif
