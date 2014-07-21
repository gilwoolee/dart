#ifndef FORCE_TORQUE_CONSTRAINT_H
#define FORCE_TORQUE_CONSTRAINT_H

#include "optimizer/Constraint.h"

namespace optimizer {
	class Var;

	class ForceTorqueConstraint : public Constraint {
	public:
		ForceTorqueConstraint(std::vector<Var *>& _var, Eigen::MatrixXd _A, Eigen::VectorXd _b, int _equality);
		virtual Eigen::VectorXd evalCon();
		virtual void fillJac(VVD, int){}
		virtual void fillJac(VVD, VVB, int);
		virtual void fillObjGrad(std::vector<double>&);

	public:
		Eigen::VectorXd mb;
		Eigen::MatrixXd mA;
	};
} // namespace optimizer

#endif // #ifndef FORCE_TORQUE_CONSTRAINT_H

