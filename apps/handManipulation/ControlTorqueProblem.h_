#ifndef CONTROL_TORQUE_PROBLEM_H
#define CONTROL_TORQUE_PROBLEM_H

#include <vector>
#include "optimizer/Problem.h"
#include "Task.h"
#include "MaintainTask.h"
#include "TrackOriTask.h"
#include "TrackPoseTask.h"
#include "TrackEETask.h"

namespace optimizer {
	class Constraint;
	class TorqueProjConstraint;
	class TorqueMagniConstraint;

	class ControlTorqueProblem : public optimizer::Problem {
	public:
		ControlTorqueProblem(std::vector<tasks::Task*>& _tasks, int _taskIndex, Eigen::VectorXd _particularSolution, Eigen::MatrixXd _otherNullSpace, Eigen::MatrixXd _thisNullSpace);
		virtual ~ControlTorqueProblem();

		void initProblem();
		virtual void update(double* coefs);

	public:
		std::vector<tasks::Task*> mTasks; // mTasks and mModel are associated, both from multi task system
		int mTaskIndex; // the index of the task that we are care about, and need to calculate the torque star of this task
		Eigen::VectorXd mParticularSolution; // the particular solution of torque for this task
		Eigen::MatrixXd mOtherNullSpace; // the null space of all the other tasks
		Eigen::MatrixXd mThisNullSpace; // the null space of this task

		TorqueProjConstraint *mTPC;
		TorqueMagniConstraint *mUTMC;
		TorqueMagniConstraint *mLTMC;
	};
} // namespace optimizer

#endif
