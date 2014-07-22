#ifndef FORCE_MAGNI_CONSTRAINT_H
#define FORCE_MAGNI_CONSTRAINT_H

#include "optimizer/Constraint.h"

namespace optimizer {
	class Var;

	class ForceMagniConstraint : public Constraint {
	public:
		ForceMagniConstraint(std::vector<Var *>& _var, int _forceIndex);
		virtual Eigen::VectorXd evalCon();
		virtual void fillJac(VVD, int){}
		virtual void fillJac(VVD, VVB, int);
		virtual void fillObjGrad(std::vector<double>&);

	public:
		int mForceIndex;
	};
} // namespace optimizer

#endif // #ifndef FORCE_MAGNI_CONSTRAINT_H

