#ifndef LCP_CONSTRAINT_H
#define LCP_CONSTRAINT_H

#include "optimizer/Constraint.h"

namespace optimizer {
	class Var;

	class LCPConstraint : public Constraint {
	public:
		LCPConstraint(std::vector<Var *>& _var, Eigen::MatrixXd _A, Eigen::VectorXd _b, int _equality);
		virtual Eigen::VectorXd evalCon();
		virtual void fillJac(VVD, int){}
		virtual void fillJac(VVD, VVB, int);
		virtual void fillObjGrad(std::vector<double>&);

	public:
		Eigen::VectorXd mb;
		Eigen::MatrixXd mA;
	};
} // namespace optimizer

#endif // #ifndef LCP_CONSTRAINT_H

