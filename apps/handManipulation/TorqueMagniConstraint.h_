#ifndef TORQUE_MAGNI_CONSTRAINT_H
#define TORQUE_MAGNI_CONSTRAINT_H

#include "optimizer/Constraint.h"

namespace optimizer {
	class Var;

	class TorqueMagniConstraint : public Constraint {
	public:
		TorqueMagniConstraint(std::vector<Var *>& _var, Eigen::VectorXd _particularSolution, Eigen::MatrixXd _nullSpace, Eigen::VectorXd _bound, int _equality);
		virtual Eigen::VectorXd evalCon();
		virtual void fillJac(VVD, int){}
		virtual void fillJac(VVD, VVB, int);
		virtual void fillObjGrad(std::vector<double>&);

		void setParticularSolution(const Eigen::VectorXd _particularSolution);
		Eigen::VectorXd getParticularSolution() const;
		void setNullSpace(const Eigen::MatrixXd _nullSpace);
		Eigen::MatrixXd getNullSpace() const;
		void setBound(const Eigen::VectorXd _bound);
		Eigen::VectorXd getBound() const;

	protected:
		Eigen::VectorXd mParticularSolution;
		Eigen::MatrixXd mNullSpace;
		Eigen::VectorXd mBound;
	};
} // namespace optimizer

#endif // #ifndef TORQUE_MAGNI_CONSTRAINT_H

