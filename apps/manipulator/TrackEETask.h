#ifndef	TRACK_EE_TASK_H
#define TRACK_EE_TASK_H

#include "Task.h"
#include "TrackEETraj.h"

namespace tasks {
	class TrackEETask : public Task {
	public:
		TrackEETask(dynamics::SkeletonDynamics *_model, dynamics::SkeletonDynamics *_obj, int _eeIndex, char *_name, tasks::EETraj* _traj = NULL);

		virtual void evalTorque();
		virtual void evalTaskFinish();
		bool evalCloseToTarget();  
		void updateTask(Eigen::VectorXd _state, Eigen::Vector3d _target, Eigen::VectorXd _gravityCompensation);
		virtual Eigen::MatrixXd getNullSpace() const;
		virtual Eigen::MatrixXd getTaskSpace() const;
		Eigen::Vector3d getTarget() { return mTarget; }
		int getEEIndex() const { return mEEIndex; }
		void setFinalTarget(Eigen::Vector3d _finalTarget) { mFinalTarget = _finalTarget; }
		Eigen::Vector3d getFinalTarget() const { return mFinalTarget; }
		void setTargetVel(Eigen::Vector3d _targetVel) { mTargetVel = _targetVel; }
		Eigen::Vector3d getTargetVel() const { return mTargetVel; }

		EETraj* getEETraj() const { return mTraj; }
		void setEETraj( tasks::EETraj* _traj) { mTraj = _traj; }
		dynamics::SkeletonDynamics* getObj() const { return mObj; }
		void setObj( dynamics::SkeletonDynamics* _obj ) { mObj = _obj; }
		
	protected:
		Eigen::Vector3d mTarget;      // the target of the end effector
		Eigen::Vector3d mFinalTarget; // the final target of the end effector
		Eigen::Vector3d mTargetVel;   // the target velocity of the end effector when there is trajectory
		int mEEIndex;
		double mPGain; 
		double mVGain;
		double mPGainTraj;
		double mVGainTraj;
		Eigen::VectorXd mOtherForce;
		tasks::EETraj* mTraj;
		dynamics::SkeletonDynamics *mObj;
	};
} // namespace tasks

#endif // #ifndef TRACK_EE_TASK_H

