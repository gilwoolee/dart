#include "OrientationConstraint.h"
#include "kinematics/BodyNode.h"
#include "kinematics/Skeleton.h"
#include "optimizer/Var.h"
#include "math/UtilsMath.h"

using namespace kinematics;
using namespace dart_math;

namespace optimizer {
    
    OrientationConstraint::OrientationConstraint(vector<Var *>& var, Skeleton* skel, BodyNode* node, const Vector3d& val) : Constraint(var), mSkel(skel), mNode(node), mTarget(val) {
        mNumRows = 3;

        mWeight = VectorXd::Ones(mNumRows);
        mConstTerm = VectorXd::Zero(mNumRows);
        mCompletion = VectorXd::Zero(mNumRows);
    }

    VectorXd OrientationConstraint::evalCon() {
		double w = 0.01;

		Eigen::Matrix4d transformMatrix = mNode->getWorldTransform();
		Eigen::Matrix3d rotationMatrix = transformMatrix.topLeftCorner(3,3);
		Eigen::Vector3d orientationVector = rotationMatrix*Vector3d(0.0,-1.0,0.0);

        Vector3d c = orientationVector - mTarget;
        VectorXd ret(c);

		ret = w*ret;

        return ret;
    }

    void OrientationConstraint::fillJac(VVD jEntry, VVB jMap, int index) {
    }

    void OrientationConstraint::fillObjGrad(std::vector<double>& dG) {
        VectorXd dP = evalCon();
        for(int dofIndex = 0; dofIndex < mNode->getNumDependentDofs(); dofIndex++) {
            int i = mNode->getDependentDof(dofIndex);            
            const Var* v = mVariables[i];
            double w = v->mWeight;
			w = 0.01;
            VectorXd J = mNode->getDerivWorldTransform(dofIndex).topLeftCorner(3,3)*Vector3d(0.0,-1.0,0.0);
            J = w*J;
            dG[i] += dP.dot(J);
        }
    }

    void OrientationConstraint::setTarget(const Vector3d& target) {
        mTarget = target;
    }

    Vector3d OrientationConstraint::getTarget() const {
        return mTarget;
    }
} // namespace optimizer
