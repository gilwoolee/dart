#ifndef FORCE_ANGLE_CONSTRAINT_H
#define FORCE_ANGLE_CONSTRAINT_H

#include "optimizer/Constraint.h"

namespace optimizer {
	class Var;

	class ForceAngleConstraint : public Constraint {
	public:
		ForceAngleConstraint(std::vector<Var *>& _var, Eigen::Vector3d _normal, int _forceIndex, double _weight);

		virtual Eigen::VectorXd evalCon();
		virtual double evalObj();
		virtual void fillJac(VVD, int){}
		virtual void fillJac(VVD, VVB, int);
		virtual void fillObjGrad(std::vector<double>&);

	public:
		Eigen::Vector3d mNormal;
		int mForceIndex;
		double mAngleWeight;
	};
} // namespace optimizer

#endif // #ifndef FORCE_ANGLE_CONSTRAINT_H

