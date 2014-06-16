#ifndef	TRACK_ORI_TASK_H
#define TRACK_ORI_TASK_H

#include "Task.h"

namespace tasks {
	class TrackOriTask : public Task {
	public:
		TrackOriTask(dynamics::SkeletonDynamics *_model, int _eeIndex, char *_name);

		virtual void evalTorque();
		virtual void evalTaskFinish();
		void updateTask(Eigen::VectorXd _state, Eigen::Vector3d _target, Eigen::VectorXd _gravityCompensation);
		virtual Eigen::MatrixXd getNullSpace() const;
		virtual Eigen::MatrixXd getTaskSpace() const;
		Eigen::Vector3d getTarget() { return mTarget; }
		int getEEIndex() const { return mEEIndex; }
		void setTarget(Eigen::Vector3d _target) { mTarget = _target; }
		Eigen::Vector3d evalTaskError();
		void setAccumulateError(Eigen::Vector3d _accumulateError) { mAccumulateError = _accumulateError; }

	protected:
		Eigen::Vector3d mTarget; // the target of the end effector
		int mEEIndex;
		double mPGain; 
		double mVGain;
		double mIGain;
		Eigen::VectorXd mOtherForce;
		Eigen::Vector3d mAccumulateError;
	};
} // namespace tasks

#endif // #ifndef TRACK_ORI_TASK_H

