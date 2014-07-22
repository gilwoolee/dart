#include "LCPInvProblem.h"

#include <iostream>
using namespace std;

#include "optimizer/Var.h"
#include "optimizer/ObjectiveBox.h"
#include "optimizer/ConstraintBox.h"
using namespace optimizer;
#include "LCPConstraint.h"
#include "utils/Paths.h"
#include "math/UtilsMath.h"

using namespace Eigen;

namespace optimizer {
	LCPInvProblem::LCPInvProblem(Eigen::MatrixXd& _A, Eigen::VectorXd& _b, std::vector<bool>& _incontact)
		: Problem() {
			A = _A;
			b = _b;
			inContactIndices = _incontact;
			initProblem();
	}

	LCPInvProblem::~LCPInvProblem() {
		delete mLCPC;
	}

	void LCPInvProblem::initProblem() {

		int numDof = A.cols()/2;
		
		for (int i = 0; i < 2*numDof; ++i) {
			addVariable(0.0,-50.0,50.0);
		}
		
		// if there is torque bounds
		/*
		// add variables
		for (int i = 0; i < numDof; ++i) {
			addVariable(0.0,-50.0,50.0);
		}
		// wrist
		addVariable(0.0, -0.00001, 0.00001);
		addVariable(0.0, -0.00001, 0.00001);
		addVariable(0.0, -0.00001, 0.00001);
		addVariable(0.0, -0.00001, 0.00001);
		// thumb
		if (inContactIndices[0]) {
			addVariable(0.0, -0.05, 0.05);
			addVariable(0.0, -0.05, 0.05);
			addVariable(0.0, -0.05, 0.05);
			addVariable(0.0, -0.05, 0.05);
			addVariable(0.0, -0.05, 0.05);
		}
		else {
			addVariable(0.0, -0.00001, 0.00001);
			addVariable(0.0, -0.00001, 0.00001);
			addVariable(0.0, -0.00001, 0.00001);
			addVariable(0.0, -0.00001, 0.00001);
			addVariable(0.0, -0.00001, 0.00001);
		}
		// index
		if (inContactIndices[1]) {
			addVariable(0.0, -0.05, 0.05);
			addVariable(0.0, -0.05, 0.05);
			addVariable(0.0, -0.05, 0.05);
			addVariable(0.0, -0.05, 0.05);
		}
		else {
			addVariable(0.0, -0.00001, 0.00001);
			addVariable(0.0, -0.00001, 0.00001);
			addVariable(0.0, -0.00001, 0.00001);
			addVariable(0.0, -0.00001, 0.00001);
		}
		// middle
		if (inContactIndices[2]) {
			addVariable(0.0, -0.05, 0.05);
			addVariable(0.0, -0.05, 0.05);
			addVariable(0.0, -0.05, 0.05);
			addVariable(0.0, -0.05, 0.05);
		}
		else {
			addVariable(0.0, -0.00001, 0.00001);
			addVariable(0.0, -0.00001, 0.00001);
			addVariable(0.0, -0.00001, 0.00001);
			addVariable(0.0, -0.00001, 0.00001);
		}
		// ring
		if (inContactIndices[3]) {
			addVariable(0.0, -0.05, 0.05);
			addVariable(0.0, -0.05, 0.05);
			addVariable(0.0, -0.05, 0.05);
			addVariable(0.0, -0.05, 0.05);
		}
		else {
			addVariable(0.0, -0.00001, 0.00001);
			addVariable(0.0, -0.00001, 0.00001);
			addVariable(0.0, -0.00001, 0.00001);
			addVariable(0.0, -0.00001, 0.00001);
		}
		// pinky
		if (inContactIndices[4]) {
			addVariable(0.0, -0.05, 0.05);
			addVariable(0.0, -0.05, 0.05);
			addVariable(0.0, -0.05, 0.05);
			addVariable(0.0, -0.05, 0.05);
		}
		else {
			addVariable(0.0, -0.00001, 0.00001);
			addVariable(0.0, -0.00001, 0.00001);
			addVariable(0.0, -0.00001, 0.00001);
			addVariable(0.0, -0.00001, 0.00001);
		}
		// add variables for internal torque of object
		addVariable(0.0, -0.00001, 0.00001);
		addVariable(0.0, -0.00001, 0.00001);
		addVariable(0.0, -0.00001, 0.00001);
		addVariable(0.0, -0.00001, 0.00001);
		addVariable(0.0, -0.00001, 0.00001);
		addVariable(0.0, -0.00001, 0.00001);
		*/
		// Create Con and Obj Boxes
		createBoxes();
		mLCPC = new LCPConstraint(this->vars(),A,b,0);
		conBox()->add(mLCPC);
	}

	void LCPInvProblem::update(double* coefs) {

	}

} // namespace optimizer
