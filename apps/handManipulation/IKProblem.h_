#ifndef PROBLEM_H
#define PROBLEM_H

#include <vector>
#include "optimizer/Problem.h"

namespace kinematics {
    class Skeleton;
    template<class SkeletonType>
    class FileInfoSkel;
} // namespace kinematics

namespace dynamics{
	class SkeletonDynamics;
}

namespace optimizer {
    class PositionConstraint;
	class OrientationConstraint;
    
    class IKProblem : public optimizer::Problem {
    public:
        IKProblem(dynamics::SkeletonDynamics *_skel, std::vector<std::string>& _fingerNames);
        virtual ~IKProblem();
    
        void initProblem(dynamics::SkeletonDynamics *_skel);
        virtual void update(double* coefs);
        dynamics::SkeletonDynamics* getSkel() const;

        PositionConstraint* getConstraint(int index) const;
		OrientationConstraint* getOriConstraint(int index) const;
    protected:
        dynamics::SkeletonDynamics *mSkel;
        std::vector<PositionConstraint*> mConstraints;
		std::vector<OrientationConstraint*> mOriConstraints;
		std::vector<std::string> mFingerNames;
    };
} // namespace optimizer

#endif
