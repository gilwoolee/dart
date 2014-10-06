#ifndef ELLIPSOID_VOLUME_CONSTRAINT_H
#define ELLIPSOID_VOLUME_CONSTRAINT_H

#include "optimizer/Constraint.h"

namespace optimizer {
	class Var;

	class EllipsoidVolumeConstraint : public Constraint {
	public:	
		EllipsoidVolumeConstraint(std::vector<Var *>& _var);
		virtual Eigen::VectorXd evalCon();
		virtual void fillJac(VVD, int){}
		virtual void fillJac(VVD, VVB, int);
		virtual void fillObjGrad(std::vector<double>&);

	public:

	};
} // namespace optimizer

#endif // #ifndef ELLIPSOID_VOLUME_CONSTRAINT_H

