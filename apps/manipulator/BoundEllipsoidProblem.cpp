#include "BoundEllipsoidProblem.h"

#include <iostream>
using namespace std;

#include "optimizer/Var.h"
#include "optimizer/ObjectiveBox.h"
#include "optimizer/ConstraintBox.h"
using namespace optimizer;
#include "InEllipsoidConstraint.h"
#include "EllipsoidVolumeConstraint.h"
#include "utils/Paths.h"
#include "math/UtilsMath.h"

using namespace Eigen;

namespace optimizer {
	BoundEllipsoidProblem::BoundEllipsoidProblem(std::vector<Eigen::Vector3d>& _featurePoints)
		: Problem() {
			mFeaturePoints = _featurePoints;
			initProblem();
	}

	BoundEllipsoidProblem::~BoundEllipsoidProblem() {
		delete mEVC;
		for (int i = 0; i < mIEC.size(); ++i) {
			delete mIEC[i];
		}
	}

	void BoundEllipsoidProblem::initProblem() {

		int numFeature = mFeaturePoints.size();

		addVariable(0.025, 0.0001, 50.0);
		addVariable(0.025, 0.0001, 50.0);
		addVariable(0.025, 0.0001, 50.0);
		addVariable(0.0, -50.0, 50.0);
		addVariable(0.0, -50.0, 50.0);
		addVariable(0.0, -50.0, 50.0);

		// Create Con and Obj Boxes
		createBoxes();

		// add constraints
		mEVC = new EllipsoidVolumeConstraint(this->vars());
		objBox()->add(mEVC);

		for (int i = 0; i < numFeature; ++i) {
			InEllipsoidConstraint* c = new InEllipsoidConstraint(this->vars(),mFeaturePoints.at(i),-1);
			mIEC.push_back(c);
			conBox()->add(c);
		}

	}

	void BoundEllipsoidProblem::update(double* coefs) {

	}

} // namespace optimizer
