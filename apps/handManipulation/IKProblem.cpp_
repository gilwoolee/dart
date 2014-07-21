#include "IKProblem.h"

#include <iostream>
using namespace std;

#include "kinematics/FileInfoSkel.hpp"
#include "kinematics/Skeleton.h"
#include "kinematics/BodyNode.h"
#include "kinematics/Marker.h"
using namespace kinematics;

#include "dynamics/SkeletonDynamics.h"
#include "dynamics/BodyNodeDynamics.h"
using namespace dynamics;


#include "optimizer/Var.h"
#include "optimizer/ObjectiveBox.h"
#include "optimizer/ConstraintBox.h"
using namespace optimizer;
#include "PositionConstraint.h"
#include "OrientationConstraint.h"
#include "utils/Paths.h"

namespace optimizer {
    IKProblem::IKProblem(dynamics::SkeletonDynamics *_skel, std::vector<std::string>& _fingerNames)
        : Problem(), mSkel(_skel), mFingerNames(_fingerNames) {
        initProblem(_skel);
    }

    IKProblem::~IKProblem() {
    }

    void IKProblem::initProblem(dynamics::SkeletonDynamics *_skel) {
       
        // add variables
        for (int i = 0; i < getSkel()->getNumDofs(); i++) {
	  if (i == 0 || i == 1 || i == 2) {
				addVariable(mSkel->getDof(i)->getValue(), mSkel->getDof(i)->getValue()-0.0001, mSkel->getDof(i)->getValue()+0.0001);
			}
			else {
				addVariable(mSkel->getDof(i)->getValue(), mSkel->getDof(i)->getMin(), mSkel->getDof(i)->getMax());
			}
        }

        // Create Con and Obj Boxes
        createBoxes();

        // add positional constraints, should based on the controller to add different constraints
        mConstraints.clear();

		for (int i = 0; i < mFingerNames.size(); ++i) {
			BodyNode* node = mSkel->getNode(mFingerNames[i].c_str());
			//Eigen::Vector3d offset = node->getLocalCOM();
			Eigen::Vector3d offset = Vector3d(0.0,-0.005,0.025);
			PositionConstraint* pos = new PositionConstraint(mVariables, mSkel, node, offset, Eigen::Vector3d::Zero());
			OrientationConstraint* ori = new OrientationConstraint(mVariables, mSkel, node, Eigen::Vector3d::Zero());
			if (i == 1 || i == 2|| i == 3 || i == 4) {
				mObjBox->add(pos);
			}
			if (i == 1|| i == 2 || i == 3 || i == 4) {
				mObjBox->add(ori);
			}

			mConstraints.push_back(pos);
			mOriConstraints.push_back(ori);
		}
		
    }

    void IKProblem::update(double* coefs) {
        Eigen::VectorXd pose(mVariables.size());
        for (unsigned int i = 0; i < mVariables.size(); ++i) {
            pose(i) = mVariables[i]->mVal;
        }
        getSkel()->setPose(pose,true,true);
    }

    dynamics::SkeletonDynamics* IKProblem::getSkel() const {
        return mSkel;
    }

    PositionConstraint* IKProblem::getConstraint(int index) const {
        return mConstraints[index];
    }

	OrientationConstraint* IKProblem::getOriConstraint(int index) const {
		return mOriConstraints[index];
	}


} // namespace optimizer
