#include "MaintainTask.h"
#include "dynamics/SkeletonDynamics.h"
#include "dynamics/BodyNodeDynamics.h"
#include "kinematics/dof.h"
#include "kinematics/Joint.h"
#include "kinematics/BodyNode.h"

using namespace Eigen;

namespace tasks {
	MaintainTask::MaintainTask(dynamics::SkeletonDynamics *_model, int _eeIndex, char *_name)
		: Task(_model)
	{
		mDependTask = NULL;
		strcpy(mName, _name);
		mTaskType = tasks::MAINTAIN;
		mPriority = 5;
		mFinish = false;

		int numDofs = mModel->getNumDofs();
		mDofs.resize(numDofs);
		mDofVels.resize(numDofs);
		mTorque.resize(numDofs);
		mOmega.resize(3,numDofs);
		mJ.resize(3,numDofs);
		mJDot.resize(3,numDofs);
		mEEIndex = _eeIndex;
		mOtherForce = VectorXd::Zero(mModel->getNumDofs());
		mCommandForce = VectorXd::Zero(3);
		mGravity = Eigen::Vector3d(0.0,-9.8,0.0);

		mTarget = Eigen::Vector3d(0.0,0.0,0.0);
		mPGain = -50.0;
		mVGain = -10.0;
	}

	void MaintainTask::evalTorque() {
		int nodeJacobianIndex = 0; // index of column in Jacobian matrix of a node
		int modelJacobianIndex = 0; // index of column in Jacobian matrix of a model
		dynamics::BodyNodeDynamics *nodel = static_cast<dynamics::BodyNodeDynamics*>(mModel->getNode(mEEIndex));

		for (modelJacobianIndex = 0; modelJacobianIndex < mModel->getNumDofs(); ++modelJacobianIndex)
		{
			if (mModel->getNode(mEEIndex)->dependsOn(modelJacobianIndex))
			{
				mJ.col(modelJacobianIndex) = mModel->getNode(mEEIndex)->getJacobianLinear().col(nodeJacobianIndex);
				nodeJacobianIndex++;
			}
			else
			{
				mJ.col(modelJacobianIndex) = VectorXd::Zero(3);
			}
		}
		mOmega = mJ * (mModel->getMassMatrix().inverse());
		FullPivLU<MatrixXd> lu_decomp(mOmega);
		mNullSpace = lu_decomp.kernel();

		nodeJacobianIndex = 0; 
		modelJacobianIndex = 0;

		for (modelJacobianIndex = 0; modelJacobianIndex < mModel->getNumDofs(); ++modelJacobianIndex)
		{
			if (mModel->getNode(mEEIndex)->dependsOn(modelJacobianIndex))
			{
				mJDot.col(modelJacobianIndex) = nodel->mJvDot.col(nodeJacobianIndex);
				nodeJacobianIndex++;
			}
			else
			{
				mJDot.col(modelJacobianIndex) = VectorXd::Zero(3);
			}
		}
		if (mEEIndex == 1) { // maintain the COM of wrist
			mCommandForce = mPGain*(mModel->getNode(mEEIndex)->getWorldCOM()-mTarget) + mVGain*mJ*mDofVels;
		}
		mTorque = mOmega.transpose()*(mOmega*mOmega.transpose()).inverse()*(mCommandForce+mOmega*(mModel->getCombinedVector() - mOtherForce)-mJDot*mDofVels);
	}

	void MaintainTask::evalTaskFinish() {

	}

	void MaintainTask::updateTask(Eigen::VectorXd _state, Eigen::Vector3d _commandForce, Eigen::VectorXd _otherForce) {
		mDofVels = _state.tail(mDofVels.size());
		mDofs = _state.head(mDofs.size());
		mCommandForce = _commandForce;
		mOtherForce = _otherForce;
	}

	Eigen::MatrixXd MaintainTask::getNullSpace() const {
		int numDofs = mModel->getNumDofs();
		MatrixXd omega = MatrixXd::Zero(3,numDofs);
		MatrixXd jv = MatrixXd::Zero(3,numDofs);
		int nodeJacobianIndex = 0; // index of column in Jacobian matrix of a node
		int modelJacobianIndex = 0; // index of column in Jacobian matrix of a model
		for (modelJacobianIndex = 0; modelJacobianIndex < mModel->getNumDofs(); ++modelJacobianIndex)
		{
			if (mModel->getNode(mEEIndex)->dependsOn(modelJacobianIndex))
			{
				jv.col(modelJacobianIndex) = mModel->getNode(mEEIndex)->getJacobianLinear().col(nodeJacobianIndex);
				nodeJacobianIndex++;
			}
			else
			{
				jv.col(modelJacobianIndex) = VectorXd::Zero(3);
			}
		}

		omega = jv * (mModel->getMassMatrix().inverse());
		FullPivLU<MatrixXd> lu_decomp(omega);
		MatrixXd nullSpace = lu_decomp.kernel();
		return nullSpace;
	}

	Eigen::MatrixXd MaintainTask::getTaskSpace() const {
		int numDofs = mModel->getNumDofs();
		MatrixXd omega = MatrixXd::Zero(3,numDofs);
		MatrixXd jv = MatrixXd::Zero(3,numDofs);
		int nodeJacobianIndex = 0; // index of column in Jacobian matrix of a node
		int modelJacobianIndex = 0; // index of column in Jacobian matrix of a model
		for (modelJacobianIndex = 0; modelJacobianIndex < mModel->getNumDofs(); ++modelJacobianIndex)
		{
			if (mModel->getNode(mEEIndex)->dependsOn(modelJacobianIndex))
			{
				jv.col(modelJacobianIndex) = mModel->getNode(mEEIndex)->getJacobianLinear().col(nodeJacobianIndex);
				nodeJacobianIndex++;
			}
			else
			{
				jv.col(modelJacobianIndex) = VectorXd::Zero(3);
			}
		}

		omega = jv * (mModel->getMassMatrix().inverse());
		return omega;
	}
} // namespace tasks
