#ifndef IN_ELLIPSOID_CONSTRAINT_H
#define IN_ELLIPSOID_CONSTRAINT_H

#include "optimizer/Constraint.h"

namespace optimizer {
	class Var;

	class InEllipsoidConstraint : public Constraint {
	public:
		InEllipsoidConstraint(std::vector<Var *>& _var, Eigen::Vector3d _featurePoint, int _equality);
		virtual Eigen::VectorXd evalCon();
		virtual void fillJac(VVD, int){}
		virtual void fillJac(VVD, VVB, int);
		virtual void fillObjGrad(std::vector<double>&);

	public:
		Eigen::Vector3d mFeaturePoint;
	};
} // namespace optimizer

#endif // #ifndef IN_ELLIPSOID_CONSTRAINT_H

