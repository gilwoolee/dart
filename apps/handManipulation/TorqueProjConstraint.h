#ifndef TORQUE_PROJ_CONSTRAINT_H
#define TORQUE_PROJ_CONSTRAINT_H

#include "optimizer/Constraint.h"

namespace optimizer {
	class Var;

	class TorqueProjConstraint : public Constraint {
	public:
		TorqueProjConstraint(std::vector<Var *>& _var, Eigen::VectorXd _target, Eigen::MatrixXd _jacobian);

		virtual Eigen::VectorXd evalCon();
		virtual void fillJac(VVD, int){}
		virtual void fillJac(VVD, VVB, int);
		virtual void fillObjGrad(std::vector<double>&);

		void setTarget(const Eigen::VectorXd _target);
		Eigen::VectorXd getTarget() const;
		void setJacobian(const Eigen::MatrixXd _jacobian);
		Eigen::MatrixXd getJacobian() const;

	protected:
		Eigen::VectorXd mTarget; // the target is the b vector
		Eigen::MatrixXd mJacobian; // the jacobian is the A matrix
	};
} // namespace optimizer

#endif // #ifndef TORQUE_PROJ_CONSTRAINT_H

