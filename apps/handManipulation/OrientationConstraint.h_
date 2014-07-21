#ifndef ORIENTATION_CONSTRAINT_H
#define ORIENTATION_CONSTRAINT_H

#include "optimizer/Constraint.h"

namespace kinematics {
    class Skeleton;
    class BodyNode;
} // namespace kinematics

namespace optimizer {
    class Var;

    class OrientationConstraint : public Constraint {
    public:
        OrientationConstraint(std::vector<Var *>& var, kinematics::Skeleton* skel, kinematics::BodyNode* node, const Eigen::Vector3d& val);

        virtual Eigen::VectorXd evalCon();
        virtual void fillJac(VVD, int){}
        virtual void fillJac(VVD, VVB, int);
        virtual void fillObjGrad(std::vector<double>&);

        void setTarget(const Eigen::Vector3d& target);
        Eigen::Vector3d getTarget() const;

    protected:
        Eigen::Vector3d mTarget;

        kinematics::Skeleton* mSkel;
        kinematics::BodyNode* mNode;
    };
} // namespace optimizer
    
#endif // #ifndef ORIENTATION_CONSTRAINT_H
