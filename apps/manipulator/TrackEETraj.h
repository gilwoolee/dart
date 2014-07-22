#ifndef TRACK_EE_TRAJ_H
#define TRACK_EE_TRAJ_H

#include <vector>
#include <Eigen/Dense>
#include "Task.h"

using namespace Eigen;

namespace tasks {

	class EETraj {
	public:
		EETraj(Eigen::Vector3d _startPoint, Eigen::Vector3d _endPoint, Eigen::Vector3d _startDir = Eigen::Vector3d(0.0,0.0,0.0), Eigen::Vector3d _endDir = Eigen::Vector3d(0.0,0.0,0.0));
		virtual ~EETraj();

		Eigen::Vector3d getStartPoint() const { return mStartPoint; }
		void setStartPoint(Eigen::Vector3d _p) { mStartPoint = _p; }
		Eigen::Vector3d getStartDirection() const { return mStartDir; }
		void setStartDirection(Eigen::Vector3d _dir) { mStartDir = _dir; }
		Eigen::Vector3d getEndPoint() const { return mEndPoint; }
		void setEndPoint(Eigen::Vector3d _p) { mEndPoint = _p; }
		Eigen::Vector3d getEndDirection() const { return mEndDir; }
		void setEndDirection(Eigen::Vector3d _dir) { mEndDir = _dir; }

		void initHermiteBasisMatrix();
		double evalTime(double _value, int _index);  // index is the index of dimension
		double evalValue(double _time, int _index);  // index is the index of dimension
		double evalTangent(double _time, int _index);
		Eigen::Vector3d evalNextTarget(Eigen::Vector3d _curPoint);
		Eigen::Vector3d evalNextTargetVel(Eigen::Vector3d _curPoint);
		void evalSamplePoint();

		int getNumSamplePoint() const { return mSamplePoints.size(); }
		Eigen::Vector3d getSamplePoint(int _index) { return mSamplePoints.at(_index); }

	private:
		Eigen::Vector3d mStartPoint;
		Eigen::Vector3d mStartDir;
		Eigen::Vector3d mEndPoint;
		Eigen::Vector3d mEndDir;
		Eigen::MatrixXd mBasisMatrix;
		std::vector<Eigen::Vector3d> mSamplePoints;
		double mStep;
	};

} // namespace tasks

#endif // TRACK_EE_TRAJ_H