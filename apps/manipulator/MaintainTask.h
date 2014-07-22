#ifndef	MAINTAIN_TASK_H
#define MAINTAIN_TASK_H

#include "Task.h"

namespace tasks {
	class MaintainTask : public Task {
	public:
		MaintainTask(dynamics::SkeletonDynamics *_model, int _eeIndex, char *_name);

		virtual void evalTorque();
		virtual void evalTaskFinish();
		void updateTask(Eigen::VectorXd _state, Eigen::Vector3d _commandForce, Eigen::VectorXd _otherForce);
		virtual Eigen::MatrixXd getNullSpace() const;
		virtual Eigen::MatrixXd getTaskSpace() const;
		int getEEIndex() const { return mEEIndex; }
		void setTarget(Eigen::Vector3d _target) { mTarget = _target; }

	protected:
		Eigen::Vector3d mCommandForce; // the commanding force to hold the object
		int mEEIndex;
		Eigen::VectorXd mOtherForce;
		Eigen::Vector3d mGravity;
		Eigen::Vector3d mTarget;      // the target of the end effector
		double mPGain; 
		double mVGain;
	};
} // namespace tasks

#endif // #ifndef MAINTAIN_TASK_H

