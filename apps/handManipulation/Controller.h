#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <Eigen/Dense>
#include <iostream>
#include <vector>
#include <string>
#include <cstring>
#include "Task.h"
#include "MaintainTask.h"
#include "TrackOriTask.h"
#include "TrackPoseTask.h"
#include "TrackEETask.h"

extern double angleX;
extern double angleY;
extern double angleZ;
extern double handPosX;
extern double handPosY;
extern double handPosZ;
extern double objPosX;
extern double objPosY;
extern double objPosZ;

namespace dynamics{
    class SkeletonDynamics;
}

namespace tasks {
	class Task;
	class MaintainTask;
	class TrackOriTask;
	class TrackPoseTask;
	class TrackEETask;
}

namespace optimizer {
	class ObjectiveBox;
	class Var;
}

class Controller {
 public:
    Controller(dynamics::SkeletonDynamics *_skel, double _t, std::vector<tasks::Task*>& _tasks, int _fingerNum, std::vector<std::string>& _fingerRootNames, std::vector<int>& _fingerTipIndices, std::string _palmName)
		:mSkel(_skel),mTimestep(_t),mTasks(_tasks),mFingerNum(_fingerNum),mFingerRootNames(_fingerRootNames),mFingerTipIndices(_fingerTipIndices),mPalmName(_palmName)
	{
		initController();
	}
    virtual ~Controller() {};

	void initController();
	void resetController(std::vector<tasks::Task*>& _tasks);
    Eigen::VectorXd getTorques() { return mTorques; };
    double getTorque(int _index) { return mTorques[_index]; };
    void setDesiredDof(int _index, double _val) { mDesiredDofs[_index] = _val; };
    void computeTorques(const Eigen::VectorXd& _dof, const Eigen::VectorXd& _dofVel, const Eigen::VectorXd& _dofAcc, const Eigen::VectorXd& _objDof, const Eigen::VectorXd& _objVel, std::vector<Eigen::Vector3d>& _contactPoints, std::vector<Eigen::Vector3d>& _contactForces, std::vector<int>& _contactIndices, const Eigen::Vector3d& _targetOri, const Eigen::Vector3d& _objOri);
    dynamics::SkeletonDynamics* getSkel() { return mSkel; };
    Eigen::VectorXd getDesiredDofs() { return mDesiredDofs; };
    Eigen::MatrixXd getKp() {return mKp; };
	void evalTargetOri(const Eigen::VectorXd& _dof, const Eigen::VectorXd& _dofVel, const Eigen::VectorXd& _objDof, const Eigen::VectorXd& _objVel, const Eigen::Vector3d& _offset, const Eigen::Vector3d& _targetOri, const Eigen::Vector3d& _objOri);
	void evalTargetPose(const Eigen::VectorXd& _dof, const Eigen::VectorXd& _dofVel, const Eigen::VectorXd& _objDof, const Eigen::VectorXd& _objVel, const Eigen::Vector3d& _offset, const Eigen::Vector3d& _targetOri, const Eigen::Vector3d& _objOri);
	Eigen::MatrixXd evalTaskNullSpace(); 
	void evalGravityCompensationForce();
	void evalObjControlForce(const std::vector<Eigen::Vector3d>& _contactPoints, const std::vector<Eigen::Vector3d>& _contactForces, const std::vector<int>& _contactIndices);
	void evalTrackForce(const Eigen::VectorXd& _dof, const Eigen::VectorXd& _dofVel);
	void evalDampForce(const Eigen::VectorXd& _dof);
	void evalOriForce(const Eigen::VectorXd& _dof, const Eigen::VectorXd& _dofVel);
	void evalMaintainForce(const Eigen::VectorXd& _dof, const Eigen::VectorXd& _dofVel);
	void evalTaskForce();
 public: 
    dynamics::SkeletonDynamics *mSkel;
    Eigen::VectorXd mTorques;
	Eigen::VectorXd mGravityCompensationForce;
	Eigen::VectorXd mObjControlForce;
	Eigen::VectorXd mTrackForce;
	Eigen::VectorXd mDampForce;
	Eigen::VectorXd mOriForce;
	Eigen::VectorXd mMaintainForce;
	Eigen::VectorXd mTaskForce;
	Eigen::VectorXd mConstraintForce;
    Eigen::VectorXd mDesiredDofs;
	Eigen::VectorXd mFingerEndPose;
	Eigen::VectorXd mFingerRestPose;
	Eigen::VectorXd mFingerInterceptPose;
    Eigen::MatrixXd mKp;
    Eigen::MatrixXd mKd;
    double mTimestep;
    int mFrame;
	int mFingerNum;

	std::vector<tasks::Task*> mTasks;
	std::vector<Eigen::MatrixXd> mJVec;
	std::vector<Eigen::MatrixXd> mInvJVec;
	Eigen::MatrixXd mCombinedJ;
	Eigen::MatrixXd mCombinedJTrans;

	std::vector<int> mActiveNumFrame;
	std::vector<int> mActiveSimFrame;
	std::vector<int> mTrackNumFrame;
	std::vector<int> mTrackSimFrame;
	std::vector<int> mInContactFrame;
	std::vector<bool> mActiveFingers;
	std::vector<bool> mContactFingers;
	std::vector<bool> mInContactFingers;
	std::vector<bool> mRestFingers;
	std::vector<bool> mTrackFingers;
	std::vector<VectorXi> mFingerDofs;
	Eigen::VectorXi mOriDofs;
	std::vector<int> mFingerTipIndices;
	std::vector<std::string> mFingerRootNames;
	std::string mPalmName;
	int mExtNodeNum;  // excluded node number when doing virtual force control
	int mExtDofNum;  // excluded dof number when doing virtual force control

	// plan related
	bool mOriFlag;
	int mOriSimFrame;
	int mOriNumFrame;
	bool mMaintainFlag;
	bool mControlFlag;
	bool mTaskFlag;
	bool mOnPalmFlag;

	Eigen::Vector3d mPreOriTarget;
	Eigen::Vector3d mAccumulateOriError;

};
    
    

#endif // #CONTROLLER_H
