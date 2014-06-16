#ifndef FORCE_SPREAD_CONSTRAINT_H
#define FORCE_SPREAD_CONSTRAINT_H

#include "optimizer/Constraint.h"

namespace optimizer {
	class Var;

	class ForceSpreadConstraint : public Constraint {
	public:
		ForceSpreadConstraint(std::vector<Var *>& _var, int _firstForceIndex, int _secondForceIndex);

		virtual Eigen::VectorXd evalCon();
		virtual void fillJac(VVD, int){}
		virtual void fillJac(VVD, VVB, int);
		virtual void fillObjGrad(std::vector<double>&);

	public:
		int mFirstForceIndex;
		int mSecondForceIndex;
	};
} // namespace optimizer

#endif // #ifndef FORCE_SPREAD_CONSTRAINT_H

