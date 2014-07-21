#ifndef BOUND_ELLIPSOID_PROBLEM_H
#define BOUND_ELLIPSOID_PROBLEM_H

#include <vector>
#include <Eigen/Dense>
#include "optimizer/Problem.h"

using namespace Eigen;

namespace optimizer {
	class Constraint;
	class EllipsoidVolumeConstraint;
	class InEllipsoidConstraint;

	class BoundEllipsoidProblem : public optimizer::Problem {
	public:
		BoundEllipsoidProblem(std::vector<Eigen::Vector3d>& _featurePoints);
		virtual ~BoundEllipsoidProblem();

		void initProblem();
		virtual void update(double* coefs);

	public:
		std::vector<Eigen::Vector3d> mFeaturePoints;

		EllipsoidVolumeConstraint * mEVC;
		std::vector<InEllipsoidConstraint *> mIEC;
	};
} // namespace optimizer

#endif
