#ifndef FORCE_PROJ_CONSTRAINT_H
#define FORCE_PROJ_CONSTRAINT_H

#include "optimizer/Constraint.h"

namespace optimizer {
	class Var;

	class ForceProjConstraint : public Constraint {
	public:
		ForceProjConstraint(std::vector<Var *>& _var, Eigen::Vector3d _normal, int _forceIndex);

		virtual Eigen::VectorXd evalCon();
		virtual void fillJac(VVD, int){}
		virtual void fillJac(VVD, VVB, int);
		virtual void fillObjGrad(std::vector<double>&);

	public:
		Eigen::Vector3d mNormal;
		int mForceIndex;
	};
} // namespace optimizer

#endif // #ifndef FORCE_PROJ_CONSTRAINT_H

