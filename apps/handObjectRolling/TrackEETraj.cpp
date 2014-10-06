#include "TrackEETraj.h"

namespace tasks{

	EETraj::EETraj(Eigen::Vector3d _startPoint, Eigen::Vector3d _endPoint, Eigen::Vector3d _startDir, Eigen::Vector3d _endDir) {
		// fast
		mStep = 0.02;
		// slow
		//mStep = 0.01;
		mEndPoint = _endPoint;
		mEndDir = _endDir;
		mStartPoint = _startPoint;
		mStartDir = _startDir;
		initHermiteBasisMatrix();
	}

	EETraj::~EETraj() {
	}

	void EETraj::initHermiteBasisMatrix() {
		mBasisMatrix = MatrixXd::Zero(4,4);
		mBasisMatrix(0,0) = 2.0;
		mBasisMatrix(0,1) = -2.0;
		mBasisMatrix(0,2) = 1.0;
		mBasisMatrix(0,3) = 1.0;
		mBasisMatrix(1,0) = -3.0;
		mBasisMatrix(1,1) = 3.0;
		mBasisMatrix(1,2) = -2.0;
		mBasisMatrix(1,3) = -1.0;
		mBasisMatrix(2,2) = 1.0;
		mBasisMatrix(3,0) = 1.0;
	}

	double EETraj::evalTime(double _value, int _index) {
		if (_value < mStartPoint(_index) && _value < mEndPoint(_index)) {
			if (mStartPoint(_index) < mEndPoint(_index)) {
				_value = mStartPoint(_index);
			}
			else {
				_value = mEndPoint(_index);
			}			
		}
		else if (_value > mStartPoint(_index) && _value > mEndPoint(_index)) {
			if (mStartPoint(_index) > mEndPoint(_index)) {
				_value = mStartPoint(_index);
			}
			else {
				_value = mEndPoint(_index);
			}
		}		
		
		return (_value - mStartPoint(_index))/(mEndPoint(_index)-mStartPoint(_index));
	}

	double EETraj::evalValue(double _time, int _index) {
		Eigen::VectorXd tVector(4);
		tVector(0) = _time*_time*_time;
		tVector(1) = _time*_time;
		tVector(2) = _time;
		tVector(3) = 1.0;

		Eigen::VectorXd controlPoint(4);
		controlPoint(0) = mStartPoint(_index);
		controlPoint(1) = mEndPoint(_index);
		controlPoint(2) = mStartDir(_index);
		controlPoint(3) = mEndDir(_index);

// 		std::cout << "t vector:" << std::endl << tVector.transpose() << std::endl;
// 		std::cout << "basis matrix:" << std::endl << mBasisMatrix << std::endl;
// 		std::cout << "control point:" << std::endl << controlPoint << std::endl;
// 
// 		std::cout << "return result:" << std::endl << tVector.transpose()*mBasisMatrix*controlPoint << std::endl;
// 
		return tVector.transpose()*mBasisMatrix*controlPoint;
	}

	double EETraj::evalTangent(double _time, int _index) {
		Eigen::VectorXd tVector(4);
		tVector(0) = 3.0*_time*_time;
		tVector(1) = 2.0*_time;
		tVector(2) = 1.0;
		tVector(3) = 0.0;

		Eigen::VectorXd controlPoint(4);
		controlPoint(0) = mStartPoint(_index);
		controlPoint(1) = mEndPoint(_index);
		controlPoint(2) = mStartDir(_index);
		controlPoint(3) = mEndDir(_index);

		return tVector.transpose()*mBasisMatrix*controlPoint;
	}

	Eigen::Vector3d EETraj::evalNextTarget(Eigen::Vector3d _curPoint) {
		Eigen::Vector3d nextTarget = _curPoint;
		double t = 0.0;
		for (int i = 0; i < 3; ++i) {
			if (t < evalTime(_curPoint(i),i)) {
				t = evalTime(_curPoint(i),i);
			}			
		}
		t += mStep;
		if (t > 1.0) {
			t = 1.0;
		}
		for (int i = 0; i < 3; ++i) {
			nextTarget(i) = evalValue(t,i);
		}
		
		return nextTarget;
	}

	Eigen::Vector3d EETraj::evalNextTargetVel(Eigen::Vector3d _curPoint) {
		Eigen::Vector3d nextTargetVel;
		double t = 0.0;
		for (int i = 0; i < 3; ++i) {
			if (t < evalTime(_curPoint(i),i)) {
				t = evalTime(_curPoint(i),i);
			}			
		}
		t += mStep;
		if (t > 1.0) {
			t = 1.0;
		}
		for (int i = 0; i < 3; ++i) {
			nextTargetVel(i) = evalTangent(t,i);
		}

		return nextTargetVel;
	}

	void EETraj::evalSamplePoint() {
		Eigen::Vector3d samplePoint;
		double t = 0.0;
		while (t <= 1.0) {
			for (int i = 0; i < 3; ++i) {
				samplePoint(i) = evalValue(t,i);
			}			
			t += mStep;
			mSamplePoints.push_back(samplePoint);
		}		
	}
} // namespace tasks
