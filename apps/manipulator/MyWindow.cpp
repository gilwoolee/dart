#include "MyWindow.h"
#include "dynamics/BodyNodeDynamics.h"
#include "dynamics/ContactDynamics.h"
#include "dynamics/ConstraintDynamics.h"
#include "kinematics/Dof.h"
#include "kinematics/Joint.h"
#include "kinematics/Transformation.h"
#include "collision/CollisionDetector.h"
#include "collision/fcl_mesh/FCLMESHCollisionDetector.h"
#include "math/UtilsMath.h"
#include "utils/Timer.h"
#include "math/UtilsRotation.h"
#include "yui/GLFuncs.h"
#include "Controller.h"
#include "optimizer/ObjectiveBox.h"
#include "optimizer/snopt/SnoptSolver.h"
#include "optimizer/Var.h"
#include "IKProblem.h"
#include "PositionConstraint.h"
#include "OrientationConstraint.h"
#include "ContactForceProblem.h"
#include "ForceProjConstraint.h"
#include "ForceTorqueConstraint.h"
#include "ForceMagniConstraint.h"
#include "ForcePlaneConstraint.h"
#include "LCPInvProblem.h"
#include "LCPConstraint.h"
#include "BoundEllipsoidProblem.h"
#include "EllipsoidVolumeConstraint.h"
#include "InEllipsoidConstraint.h"

using namespace Eigen;
using namespace dynamics;
using namespace utils;
using namespace tasks;
using namespace optimizer;
using namespace kinematics;
#ifdef WIN32
using namespace nv;
#endif

const char * const MyWindow::mFileName = "roll_cube";
#ifdef WIN32
nv::GlutUIContext ui;
#endif
float initPose[] = {-0.8000000119, 0, 0, 0, 0, 0, 0, -0.7428935766, 0, 0, 0, 0, 1.072659612, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.8000000119, 0, 0.200000003};
float initObjPose[] = {0.08, 0.05, 0.43, 0.0, 0.0, 0.0};

static utils::Timer tIter("timeIter");
static utils::Timer tInternal("timeInternal");
static utils::Timer tExternal("timeExternal");
static utils::Timer tSimulation("timeSimulation");
static utils::Timer tHandIK("timeHandIK");
static utils::Timer tSetPose("timeSetPose");
static utils::Timer tContact("timeContact");
static utils::Timer tJointLimit("timeJointLimit");

double extTorque = 0.0;

void MyWindow::initDyn()
{
    mDofs.resize(mSkels.size());
    mDofVels.resize(mSkels.size());
	mDofAccs.resize(mSkels.size());

    for (unsigned int i = 0; i < mSkels.size(); i++) {
        mDofs[i].resize(mSkels[i]->getNumDofs());
        mDofVels[i].resize(mSkels[i]->getNumDofs());
		mDofAccs[i].resize(mSkels[i]->getNumDofs());
        mDofs[i].setZero();
        mDofVels[i].setZero();
		mDofAccs[i].setZero();
    }
    
    // initial position of the ground
    mDofs[0][1] = -0.3;
	
    // initial pose for hand
    for (int i = 0; i < mSkels[1]->getNumDofs(); i++)
        mDofs[1][i] = initPose[i];

	// modify hand pose based on rolling, related to roll direction
    //mDofs[1][2] = -mAngles[0];
    
    // initial position of the box
	for (int i = 0; i < mSkels[2]->getNumDofs(); i++)
		mDofs[2][i] = initObjPose[i];

    for (unsigned int i = 0; i < mSkels.size(); i++) {
        mSkels[i]->initDynamics();
        mSkels[i]->setPose(mDofs[i], false, false);
        mSkels[i]->computeDynamics(mGravity, mDofVels[i], false);
	}

	// modify box pose based on rolling, related to roll direction
    Vector3d localPos(-0.01,-(0.02+0.011),0.06);
	Vector3d worldPos = mSkels[1]->getNode(4)->evalWorldPos(localPos);
	mDofs[2].head(3) = worldPos;

	for (unsigned int i = 0; i < mSkels.size(); i++) {
		mSkels[i]->initDynamics();
		mSkels[i]->setPose(mDofs[i], false, false);
		mSkels[i]->computeDynamics(mGravity, mDofVels[i], false);
	}

	// the the initial target angle of the palm, related to roll direction	
	setHandAngle(mAngles[0]);
	mPreOri = mSkels[1]->getPose().head(3);
	mPreContactEdge = 0;
	
	mUpFace = 2;
	
	mPreHighCorners.resize(2);
	mCurHighCorners.resize(2);
	mPreHighCorners[0] = dart_math::xformHom(mSkels[1]->getNode(4)->getWorldInvTransform(), mSkels[2]->getNode(0)->evalWorldPos(mCorners[1][0]));
	mPreHighCorners[1] = dart_math::xformHom(mSkels[1]->getNode(4)->getWorldInvTransform(), mSkels[2]->getNode(0)->evalWorldPos(mCorners[2][0]));
	mCurHighCorners[0] = mPreHighCorners[0];
	mCurHighCorners[1] = mPreHighCorners[1];

	mRollDir = 0;

	// set joint limit
	mSkels[1]->getDof(2)->setMax(1.0);
	mSkels[1]->getDof(2)->setMin(-1.0);
	mSkels[1]->getDof(3)->setMax(0.5);
	mSkels[1]->getDof(3)->setMin(-0.5);
	mSkels[1]->getDof(4)->setMax(1.0);
	mSkels[1]->getDof(4)->setMin(-1.0);
	mSkels[1]->getDof(5)->setMax(0.5);
	mSkels[1]->getDof(5)->setMin(-0.5);
	mSkels[1]->getDof(6)->setMax(0.5);
	mSkels[1]->getDof(6)->setMin(-0.5);
	mSkels[1]->getDof(7)->setMax(1.2);
	mSkels[1]->getDof(7)->setMin(-1.2);
	mSkels[1]->getDof(8)->setMax(1.7);
	mSkels[1]->getDof(8)->setMin(-0.4);
	mSkels[1]->getDof(9)->setMax(0.5);
	mSkels[1]->getDof(9)->setMin(-0.5);
	mSkels[1]->getDof(10)->setMax(1.7);
	mSkels[1]->getDof(10)->setMin(-0.4);
	mSkels[1]->getDof(11)->setMax(1.7);
	mSkels[1]->getDof(11)->setMin(-0.4);
	mSkels[1]->getDof(12)->setMax(1.2);
	mSkels[1]->getDof(12)->setMin(-0.3);
	mSkels[1]->getDof(13)->setMax(1.8);
	mSkels[1]->getDof(13)->setMin(-0.5);
	mSkels[1]->getDof(14)->setMax(1.7);
	mSkels[1]->getDof(14)->setMin(-0.4);
	mSkels[1]->getDof(15)->setMax(1.8);
	mSkels[1]->getDof(15)->setMin(-0.5);
	mSkels[1]->getDof(16)->setMax(1.8);
	mSkels[1]->getDof(16)->setMin(-0.5);
	mSkels[1]->getDof(17)->setMax(1.0);
	mSkels[1]->getDof(17)->setMin(-1.0);
	mSkels[1]->getDof(18)->setMax(1.5);
	mSkels[1]->getDof(18)->setMin(-0.4);
	mSkels[1]->getDof(19)->setMax(1.8);
	mSkels[1]->getDof(19)->setMin(-0.5);
	mSkels[1]->getDof(20)->setMax(1.5);
	mSkels[1]->getDof(20)->setMin(-0.4);
	mSkels[1]->getDof(21)->setMax(1.5);
	mSkels[1]->getDof(21)->setMin(-0.4);
	mSkels[1]->getDof(22)->setMax(1.8);
	mSkels[1]->getDof(22)->setMin(-0.5);
	mSkels[1]->getDof(23)->setMax(1.5);
	mSkels[1]->getDof(23)->setMin(-0.4);
	mSkels[1]->getDof(24)->setMax(1.5);
	mSkels[1]->getDof(24)->setMin(-0.4);


    // set the ground to be an immobile object; it will still participate in collision
    mSkels[0]->setImmobileState(true);

	// set self collidable
	mSkels[1]->setSelfCollidable(true);

    // create a collision handler
	// set the number of friction basis
	mFrictionBasis = 4;
	mConstraintHandle = new ConstraintDynamics(mSkels, mTimeStep, 1.5, mFrictionBasis);

	mInContactVec.resize(mFingerNum);
	mContactForceVec.resize(mFingerNum);
	mFingerTargetContactForceVec.resize(mFingerNum);
	mFingerContactPointVec.resize(mFingerNum);
	mContactForces.resize(mFingerNum);
	mContactNormals.resize(mFingerNum);
	mContactPointsBallStart.resize(mFingerNum);
	mContactPointsBallEnd.resize(mFingerNum);
	mTargetContactPoints.resize(mFingerNum);
	mFingerContactPoints.resize(mFingerNum);
	mFingerTargetContactForces.resize(mFingerNum);

	mFingerNames.resize(mFingerNum);
	mFingerRootNames.resize(mFingerNum);
	
	mPalmName = "palm";
	mFingerNames[0] = "thdistal";
	mFingerNames[1] = "ffdistal";
	mFingerNames[2] = "mfdistal";
	mFingerNames[3] = "rfdistal";
	mFingerNames[4] = "lfdistal";
	
	mFingerRootNames[0] = "thbase";
	mFingerRootNames[1] = "ffknuckle";
	mFingerRootNames[2] = "mfknuckle";
	mFingerRootNames[3] = "rfknuckle";
	mFingerRootNames[4] = "lfknuckle";
	mFingerTipIndices.resize(mFingerNum);
	for (int i = 0; i < mFingerNum; ++i) {
		mFingerTipIndices[i] = mSkels[1]->getNode(mFingerNames[i].c_str())->getSkelIndex();
	}
	
	// create controller
	
	mController = new Controller(mSkels[1], mTimeStep, mTasks, mFingerNum, mFingerRootNames, mFingerTipIndices,mPalmName);
	

	// plan related
	
	mController->mOriFlag = true;
	mTargetOri = Eigen::Vector3d(0.0,0.0,1.0).normalized();
	mPalmObjAngle = 0.0;


	// initialize task
	/*
	updateTaskArray();	
	mController->resetController(mTasks); // every time update task array, need to reset controller to adapt to new tasks
	*/

	// initialize target contact point	
	
	updateContactPoint();

	// initialize hand pose	
	for (int i = 0; i < mFingerNum; ++i) { // if the initial configuration is all the fingers touch the object
		mContactPointsBallStart[i] = cartesianToBall(dart_math::xformHom(mSkels[2]->getNode(0)->getWorldInvTransform(), mFingerContactPoints[i]),mObjRadius);	
	}
	updateHandPose();	
	
// 	mSkels[1]->setPose(mController->mFingerEndPose, true, true);
	

	mConstraintHandle->evaluateConstraint();
	
	// solve bounding ellipsoid
// 	solveBoundEllipsoid();
	mBoundEllipsoidR(0) = mBoundEllipsoidR(1) = mBoundEllipsoidR(2) = 0.025;
}

void MyWindow::solveBoundEllipsoid()
{
	// an example of solving the bounding ellipsoid
	std::vector<Eigen::Vector3d> featurePoints;
	Eigen::Vector3d firstPoint = Vector3d(mObjRadius,mObjRadius,mObjRadius);
	Eigen::Vector3d secondPoint = Vector3d(-mObjRadius,-mObjRadius,-mObjRadius);
	featurePoints.push_back(firstPoint);
	featurePoints.push_back(secondPoint);
	BoundEllipsoidProblem prob(featurePoints);
	snopt::SnoptSolver solver(&prob);
	solver.solve();
	for (int i = 0; i < 3; ++i) {
		std::cout << i << " : " << prob.vars()[i]->mVal << std::endl;
		mBoundEllipsoidR(i) = prob.vars()[i]->mVal;
		mBoundEllipsoidR(i) += 0.005;
	}	
}

void MyWindow::setObjInitPose()
{
	mSkels[2]->setPose(mDofs[2], false, false);
	mSkels[2]->computeDynamics(mGravity, mDofVels[2], false);
}

void MyWindow::setHandInitPose()
{
	mSkels[1]->setPose(mDofs[1], false, false);
	mSkels[1]->computeDynamics(mGravity, mDofVels[1], false);
}

void MyWindow::saveHandInitPose()
{
	string fileName = "init_pose.txt";
	ofstream outFile(fileName.c_str());
	outFile.precision(10);
	for (int i = 0; i < mSkels[1]->getNumDofs(); ++i) {
		outFile << mDofs[1][i] << ", ";
	}
}

VectorXd MyWindow::getState() {
    VectorXd state(mIndices.back() * 2);    
    for (unsigned int i = 0; i < mSkels.size(); i++) {
        int start = mIndices[i] * 2;
        int size = mDofs[i].size();
        state.segment(start, size) = mDofs[i];
        state.segment(start + size, size) = mDofVels[i];
    }
    return state;
}

VectorXd MyWindow::evalDeriv() {
    VectorXd deriv = VectorXd::Zero(mIndices.back() * 2);   

    VectorXd extForce = mSkels[2]->getExternalForces().transpose();
    extForce(4) = extTorque;
    mSkels[2]->setExternalForces(extForce);

    for (unsigned int i = 0; i < mSkels.size(); i++) {
        // skip immobile objects in forward simulation
        if (mSkels[i]->getImmobileState())
            continue;
        int start = mIndices[i] * 2;
        int size = mDofs[i].size();

		VectorXd qddot;
		qddot = (mSkels[i]->getMassMatrix()).fullPivHouseholderQr().solve(-mSkels[i]->getCombinedVector() + mSkels[i]->getExternalForces() + mConstraintHandle->getTotalConstraintForce(i) + mSkels[i]->getInternalForces());		

        mSkels[i]->clampRotation(mDofs[i], mDofVels[i]);
        deriv.segment(start, size) = mDofVels[i] + (qddot * mTimeStep); // set velocities
        deriv.segment(start + size, size) = qddot; // set qddot (accelerations)	
		mDofAccs[i] = qddot;

    }

//     std::cout << mSkels[2]->getExternalForces().transpose() << std::endl;

    return deriv;
}

void MyWindow::setState(const Eigen::VectorXd& newState) {
    for (unsigned int i = 0; i < mSkels.size(); i++) {
        int start = mIndices[i] * 2;
        int size = mDofs[i].size();
        mDofs[i] = newState.segment(start, size);
        mDofVels[i] = newState.segment(start + size, size);
    }
}

void MyWindow::setPose() {
	
	kinematics::BodyNode *wrist = mSkels[1]->getNode(mPalmName.c_str());
	int contactEdgeIndex = evalContactEdge();

	if (contactEdgeIndex == mPreContactEdge || contactEdgeIndex == -1) {
	  //setHandTrans(mPreOri,mEdges[contactEdgeIndex]);
	}
	else if (contactEdgeIndex != -1) {
		mPreContactEdge = contactEdgeIndex;
	}
	mPreOri = mSkels[1]->getPose().head(3);

	if (mRollNum < mN-1) {
		if (contactEdgeIndex == mRollNum%mEdges.size()) {
			Eigen::Vector3d contactPos = mSkels[2]->getNode(0)->evalWorldPos(mEdges[contactEdgeIndex]); // contact edge position in world coordinate
			// if change the roll direction, the condition will be changed accordingly, related to roll direction
			if ((mSkels[2]->getWorldCOM()(2)-contactPos(2) > 0.005 && mRollNum == 0) || (mSkels[2]->getWorldCOM()(2)-contactPos(2) > 0.002 && mRollNum > 0)) {
				Eigen::Vector3d liftEdge = mSkels[2]->getNode(0)->evalWorldPos(mEdges[(mRollNum+mEdges.size()-1)%mEdges.size()]);
				Eigen::Vector3d dropEdge = mSkels[2]->getNode(0)->evalWorldPos(mEdges[(mRollNum+mEdges.size()+1)%mEdges.size()]);
				Eigen::Vector3d contactEdge = mSkels[2]->getNode(0)->evalWorldPos(mEdges[(mRollNum)%mEdges.size()]);
				double liftAngle = 0.0;
				liftAngle = atan((liftEdge(1)-contactEdge(1))/(contactEdge(2)-liftEdge(2)));
				double dropAngle = 0.0;
				dropAngle = atan((dropEdge(1)-contactEdge(1))/(dropEdge(2)-contactEdge(2)));
				if (liftAngle > mAngles[mRollNum+1] && dropAngle > -mAngles[mRollNum+1]) {
					mRollNum++;
					setHandAngle(mAngles[mRollNum]);
				}
			}
		}		  	
	}
	else if (mRollNum == mN-1 && (evalEdgeInContact((mRollNum)%mEdges.size()) || evalUpFace() == (mRollNum+mEdges.size()/2)%mEdges.size())) {
	  setHandAngle(0.0);
	}

	// calculate up face if know the high corners local coordinates
	if (evalUpFace() != mUpFace) {
		mUpFace = evalUpFace();
	}

	for (unsigned int i = 0; i < mSkels.size(); i++) {
		if (mSkels[i]->getImmobileState()) {
			// need to update node transformation for collision
			mSkels[i]->setPose(mDofs[i], true, false);
		} else {
			// need to update first derivatives for collision		
			mSkels[i]->setPose(mDofs[i], true, true);
			mSkels[i]->computeDynamics(mGravity, mDofVels[i], true);
		}
	}
}

void MyWindow::resize(int w, int h)
{
#ifdef WIN32
	ui.reshape(w,h);
#endif
	mWinWidth = w;
	mWinHeight = h;

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glViewport(0, 0, mWinWidth, mWinHeight);
	gluPerspective(mPersp, (double)mWinWidth/(double)mWinHeight, 0.1,10.0);

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	int smallValue = w<h?w:h;
	mTrackBall.setCenter(Vector2d(w*0.5,h*0.5));
	mTrackBall.setRadius(smallValue/2.5);

	glutPostRedisplay();
}

void MyWindow::displayTimer(int _val)
{	
    int numIter = 10;
    if (mPlay) {
        mPlayFrame += 20;
        if (mPlayFrame >= mBakedStates.size())
            mPlayFrame = 0;
        glutPostRedisplay();
        glutTimerFunc(mDisplayTimeout, refreshTimer, _val);  
    }else if (mSim) {       
// 		tHandIK.startTimer();
		updateHandPose();
// 		tHandIK.stopTimer();
        for (int i = 0; i < numIter; i++) {
// 			tIter.startTimer();
			setPose();
// 			updateHandPose();
			updateContact();
// 			tInternal.startTimer();
			// evaluate all the other hand control force before evaluating object control force, and updateIntForce is inside updateConact, then comment
			updateIntForce(); // evaluate the hand control force including object control force, the force evaluation is not inside updateContact
// 			tInternal.stopTimer();
// 			tExternal.startTimer();
			updateExtForce();
// 			tExternal.stopTimer();
// 			tSimulation.startTimer();
            mIntegrator.integrate(this, mTimeStep);
// 			tSimulation.stopTimer();
			bake();
            mSimFrame++;
// 			tIter.stopTimer();	
			
        }
        mForce.setZero();

        glutPostRedisplay();
        glutTimerFunc(mDisplayTimeout, refreshTimer, _val);

// 		tIter.print();
// 		tInternal.print();
// 		tExternal.print();
// 		tSimulation.print();
// 		tHandIK.print();
// 		tSetPose.print();
// 		tContact.print();
// 		tJointLimit.print();
    }
}

Eigen::Vector3d MyWindow::ballToCartesian(Eigen::VectorXd _ball)
{
	Eigen::Vector3d cartesian;
	cartesian(0) = mBoundEllipsoidR(0)*cos(_ball(0))*cos(_ball(1));
	cartesian(1) = mBoundEllipsoidR(1)*cos(_ball(0))*sin(_ball(1));
	cartesian(2) = mBoundEllipsoidR(2)*sin(_ball(0));
	return cartesian;
}

Eigen::Vector3d MyWindow::ballToCartesian(Eigen::VectorXd _ball, double _radius)
{
	Eigen::Vector3d cartesian;
	cartesian(0) = _radius*cos(_ball(0))*cos(_ball(1));
	cartesian(1) = _radius*cos(_ball(0))*sin(_ball(1));
	cartesian(2) = _radius*sin(_ball(0));
	return cartesian;
}

Eigen::Vector2d MyWindow::cartesianToBall(Eigen::Vector3d _cartesian)
{
	Eigen::Vector2d ball = Vector2d::Zero();
	ball(0) = asin(_cartesian(2)/mBoundEllipsoidR(2));
	ball(1) = asin(_cartesian(1)/(mBoundEllipsoidR(1)*cos(ball(0))));

	if ((_cartesian(0)-mBoundEllipsoidR(0)*cos(ball(0))*cos(ball(1)))*(_cartesian(0)-mBoundEllipsoidR(0)*cos(ball(0))*cos(ball(1))) < 0.00001) {
		return ball;
	}
	
	if (ball(1) > 0.0) {
		ball(1) = M_PI - ball(1);
	}
	else if (ball(1) < 0.0) {
		ball(1) = -M_PI - ball(1);
	}
	if ((_cartesian(0)-mBoundEllipsoidR(0)*cos(ball(0))*cos(ball(1)))*(_cartesian(0)-mBoundEllipsoidR(0)*cos(ball(0))*cos(ball(1))) < 0.00001) {
		return ball;
	}

	if (ball(0) > 0.0) {
		ball(0) = M_PI - ball(0);
	}
	else if (ball(0) < 0.0) {
		ball(0) = -M_PI - ball(0);
	}
	ball(1) = asin(_cartesian(1)/(mBoundEllipsoidR(1)*cos(ball(0))));
	if ((_cartesian(0)-mBoundEllipsoidR(0)*cos(ball(0))*cos(ball(1)))*(_cartesian(0)-mBoundEllipsoidR(0)*cos(ball(0))*cos(ball(1))) < 0.00001) {
		return ball;
	}

	if (ball(1) > 0.0) {
		ball(1) = M_PI - ball(1);
	}
	else if (ball(1) < 0.0) {
		ball(1) = -M_PI - ball(1);
	}
	if ((_cartesian(0)-mBoundEllipsoidR(0)*cos(ball(0))*cos(ball(1)))*(_cartesian(0)-mBoundEllipsoidR(0)*cos(ball(0))*cos(ball(1))) < 0.00001) {
		return ball;
	}

	return ball;
}

Eigen::Vector2d MyWindow::cartesianToBall(Eigen::Vector3d _cartesian, double _radius)
{
	Eigen::Vector2d ball = Vector2d::Zero();

	if (_cartesian(2)/_radius < 1.0 && _cartesian(2)/_radius > -1.0) {
		ball(0) = asin(_cartesian(2)/_radius);
	}
	else if (_cartesian(2)/_radius > 0.99) {
		ball(0) = asin(0.99);
	}	
	else if (_cartesian(2)/_radius < -0.99) {
		ball(0) = asin(-0.99);
	}

	if (_cartesian(1)/(_radius*cos(ball(0))) < 1.0 && _cartesian(1)/(_radius*cos(ball(0))) > -1.0) {
		ball(1) = asin(_cartesian(1)/(_radius*cos(ball(0))));
	}
	else if (_cartesian(1)/(_radius*cos(ball(0))) > 0.99) {
		ball(1) = asin(0.99);
	}	
	else if (_cartesian(1)/(_radius*cos(ball(0))) < -0.99) {
		ball(1) = asin(-0.99);
	}	

	if ((_cartesian(0)-_radius*cos(ball(0))*cos(ball(1)))*(_cartesian(0)-_radius*cos(ball(0))*cos(ball(1))) < 0.00001) {
		return ball;
	}

	if (ball(1) > 0.0) {
		ball(1) = M_PI - ball(1);
	}
	else if (ball(1) < 0.0) {
		ball(1) = -M_PI - ball(1);
	}
	if ((_cartesian(0)-_radius*cos(ball(0))*cos(ball(1)))*(_cartesian(0)-_radius*cos(ball(0))*cos(ball(1))) < 0.00001) {                    
		return ball;
	}

	if (ball(0) > 0.0) {
		ball(0) = M_PI - ball(0);
	}
	else if (ball(0) < 0.0) {
		ball(0) = -M_PI - ball(0);
	}
	ball(1) = asin(_cartesian(1)/(_radius*cos(ball(0))));
	if ((_cartesian(0)-_radius*cos(ball(0))*cos(ball(1)))*(_cartesian(0)-_radius*cos(ball(0))*cos(ball(1))) < 0.00001) {
		return ball;
	}

	if (ball(1) > 0.0) {
		ball(1) = M_PI - ball(1);
	}
	else if (ball(1) < 0.0) {
		ball(1) = -M_PI - ball(1);
	}
	if ((_cartesian(0)-_radius*cos(ball(0))*cos(ball(1)))*(_cartesian(0)-_radius*cos(ball(0))*cos(ball(1))) < 0.00001) {
		return ball;
	}

	return ball;
}

Eigen::Vector3d MyWindow::objToEllipsoid(Eigen::Vector3d _obj)
{
	double t = 0.0;
	t = pow(1/((_obj(0)*_obj(0)/(mBoundEllipsoidR(0)*mBoundEllipsoidR(0)))+(_obj(1)*_obj(1)/(mBoundEllipsoidR(1)*mBoundEllipsoidR(1)))+(_obj(2)*_obj(2)/(mBoundEllipsoidR(2)*mBoundEllipsoidR(2)))),0.5);
	if (t < 0.0) {
		t = -t;
	}
	return t*_obj;
}

void MyWindow::draw()
{
	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
	
	if (mDrawModel) {
		mSkels[2]->draw(mRI);		
	}

	if (!mSim) {
		if (mPlayFrame < mBakedStates.size()) {
            for (unsigned int i = 0; i < mSkels.size(); i++) {
                int start = mIndices[i];
                int size = mDofs[i].size();

				mSkels[1]->getJoint(0)->getTransform(0)->getDof(0)->setValue(mRootTrans[mPlayFrame](0));
				mSkels[1]->getJoint(0)->getTransform(0)->getDof(1)->setValue(mRootTrans[mPlayFrame](1));
				mSkels[1]->getJoint(0)->getTransform(0)->getDof(2)->setValue(mRootTrans[mPlayFrame](2));
				mSkels[1]->getJoint(0)->updateStaticTransform();

                mSkels[i]->setPose(mBakedStates[mPlayFrame].segment(start, size), false, false);
            }

			int sumDofs = mIndices[mSkels.size()]; 
			int nContact = (mBakedStates[mPlayFrame].size() - sumDofs) / 6;
			for (int i = 0; i < nContact; i++) {
				Vector3d v = mBakedStates[mPlayFrame].segment(sumDofs + i * 6, 3);
				Vector3d f = mBakedStates[mPlayFrame].segment(sumDofs + i * 6 + 3, 3) / 10.0;

				// draw contact force
				glBegin(GL_LINES);
				mRI->setPenColor(Vector3d(0.2, 0.8, 0.2));
				glVertex3f(v[0], v[1], v[2]);
				glVertex3f(v[0] - f[0], v[1] - f[1], v[2] - f[2]);
				glEnd();

				// draw in contact point and break contact point
				mRI->pushMatrix();
				glTranslated(v[0], v[1], v[2]);
				if (f.norm() > 0.0001) {
					mRI->setPenColor(Vector3d(0.8, 0.2, 0.2));
				}
				else {
					mRI->setPenColor(Vector3d(0.2, 0.8, 0.2));
				}
				mRI->drawEllipsoid(Vector3d(0.002, 0.002, 0.002));
				mRI->popMatrix();
			}
		}
	}else{
		for (int k = 0; k < mConstraintHandle->getNumContacts(); k++) {
			Vector3d v = mConstraintHandle->getCollisionChecker()->getContact(k).point;
			Vector3d n = mConstraintHandle->getCollisionChecker()->getContact(k).normal / 10.0;
			Vector3d f = mConstraintHandle->getCollisionChecker()->getContact(k).force / 100.0;
			/*
			mRI->setPenColor(Vector3d(0.2, 0.2, 0.8));
			glBegin(GL_LINES);
			glVertex3f(v[0], v[1], v[2]);
			glVertex3f(v[0] + f[0], v[1] + f[1], v[2] + f[2]);
			glEnd();
			mRI->setPenColor(Vector3d(0.2, 0.8, 0.2));
			mRI->pushMatrix();
			glTranslated(v[0], v[1], v[2]);
			mRI->drawEllipsoid(Vector3d(0.002, 0.002, 0.002));
			mRI->popMatrix();
			*/
		}
	}


	if (mDrawBoundEllipsoid) {
		for (int i = 0; i < mFingerNum; ++i) {
			if (mController->mActiveFingers[i]) {
				Eigen::Vector2d startPointB = mContactPointsBallStart[i];
				Eigen::Vector2d endPointB = mContactPointsBallEnd[i];
				Eigen::Vector2d middlePointB;

				Eigen::Vector3d startPointC = ballToCartesian(startPointB);
				Eigen::Vector3d endPointC = ballToCartesian(endPointB);
				Eigen::Vector3d middlePointC;

				Eigen::Vector3d startPointCW = mSkels[2]->getNode(0)->evalWorldPos(startPointC);
				Eigen::Vector3d endPointCW = mSkels[2]->getNode(0)->evalWorldPos(endPointC);
				Eigen::Vector3d middlePointCW;

				// draw the bounding ball
				/*
				mRI->setPenColor(Vector3d(0.0, 0.0, 1.0));
				mRI->pushMatrix();
				glTranslated(mDofs[2][0], mDofs[2][1], mDofs[2][2]);
				glScaled(mBoundEllipsoidR(0),mBoundEllipsoidR(1),mBoundEllipsoidR(2));
				glutSolidSphere(1.0,64,64);
				mRI->popMatrix();
				*/

				mRI->setPenColor(Vector3d(1.0, 0.0, 0.0));
				mRI->pushMatrix();
				glTranslated(startPointCW(0), startPointCW(1), startPointCW(2));
				mRI->drawEllipsoid(Vector3d(0.01, 0.01, 0.01));
				mRI->popMatrix();

				// draw the end point
				/*
				mRI->setPenColor(Vector3d(1.0, 1.0, 0.0));
				mRI->pushMatrix();
				glTranslated(endPointCW(0), endPointCW(1), endPointCW(2));
				mRI->drawEllipsoid(Vector3d(0.01, 0.01, 0.01));
				mRI->popMatrix();
				*/
				
				for (int j = 1; j < 10; ++j) {
					middlePointCW = evalFingerTipTraj(startPointB,endPointB,j,10);
					mRI->setPenColor(Vector3d(0.0, 1.0, 0.0));
					mRI->pushMatrix();
					glTranslated(middlePointCW(0), middlePointCW(1), middlePointCW(2));
					mRI->drawEllipsoid(Vector3d(0.01, 0.01, 0.01));
					mRI->popMatrix();
				}
			}
		}
	}

	mSkels[1]->draw(mRI);

	if (mDrawIK) {
		// draw IK result
		if (mSim) {
			Eigen::VectorXd currentPose;
			currentPose = mSkels[1]->getPose();
			mSkels[1]->setPose(mController->mFingerEndPose,true,false);
			mSkels[1]->draw(mRI,Vector4d(1.0,0.0,1.0,1.0),false);
			mSkels[1]->setPose(currentPose,true,true);
		}	
	}

	glDisable(GL_LIGHTING);
#ifdef WIN32
	doUI();	
#endif
	glEnable(GL_LIGHTING);
	
	// display the frame count in 2D text
	glDisable(GL_LIGHTING);
	char buff[64];
	if (!mSim) 
		sprintf(buff, "%d", mPlayFrame);
	else
		sprintf(buff, "%d", mSimFrame+10);
	string frame(buff);
	glColor3f(0.0,0.0,0.0);
	yui::drawStringOnScreen(0.02f,0.02f,frame);
	glEnable(GL_LIGHTING);

	// screen shot
	int fr = mSimFrame;
	string scrShotName;
	stringstream convert;
	convert << fr;

	scrShotName = mFileName;
	if (fr < 10) {
		scrShotName += "000" + convert.str() + ".tga";
	}
	else if (fr > 9 && fr < 100) {
		scrShotName += "00" + convert.str() + ".tga";
	}
	else if (fr > 99 && fr < 1000) {
		scrShotName += "0" + convert.str() + ".tga";
	}
	else {
		scrShotName += convert.str() + ".tga";
	}
	if (mSim && mSimFrame%50 == 0) {
	  //yui::screenShot(640,480,scrShotName.c_str(),false);
	}	

}

void MyWindow::keyboard(unsigned char key, int x, int y)
{
    Eigen::VectorXd pose = mSkels[1]->getPose();
    Eigen::VectorXd objPose = mSkels[2]->getPose();
    Eigen::VectorXd extForce = mSkels[2]->getExternalForces();
    switch(key){
    case ' ': // use space key to play or stop the motion
        mSim = !mSim;
        if (mSim) {
            mPlay = false;
            glutTimerFunc( mDisplayTimeout, refreshTimer, 0);
        }
        break;
	case 's':
		saveBake();
		break;
	case 'l':
		loadBake();
		break;
	case 'h':
		saveHandInitPose();
		break;
    case 'p': // playBack
        mPlay = !mPlay;
        if (mPlay) {
            mSim = false;
            glutTimerFunc( mDisplayTimeout, refreshTimer, 0);
        }
        break;
    case '[': // step backward
        if (!mSim) {
            mPlayFrame--;
            if(mPlayFrame < 0)
                mPlayFrame = 0;
            glutPostRedisplay();
        }
        break;
    case ']': // step forwardward
        if (!mSim) {
            mPlayFrame++;
            if(mPlayFrame >= mBakedStates.size())
                mPlayFrame = 0;
            glutPostRedisplay();
        }
        break;
    case 'v': // show or hide markers
        mShowMarkers = !mShowMarkers;
        break;
	case 'i':
		updateContactPoint();
		for (int i = 0; i < mFingerNum; ++i) { // if the initial configuration is all the fingers touch the object
			mContactPointsBallStart[i] = cartesianToBall(dart_math::xformHom(mSkels[2]->getNode(0)->getWorldInvTransform(), mFingerContactPoints[i]),mObjRadius);	
		}
		updateHandPose();	
		mSkels[1]->setPose(mController->mFingerEndPose, true, true);
		mDofs[1] = mController->mFingerEndPose;
		break;
	case 'o':
		mDrawOri = !mDrawOri;
		glutPostRedisplay();
		break;
	case 'k':
		mDrawIK = !mDrawIK;
		glutPostRedisplay();
		break;
	case 'm':
		mDrawModel = !mDrawModel;
		glutPostRedisplay();
		break;
	case 't':
		mDrawTarget = !mDrawTarget;
		glutPostRedisplay();
		break;
	case 'b':
		mDrawBoundEllipsoid = !mDrawBoundEllipsoid;
		glutPostRedisplay();
		break;
	case 'f':
		mDrawContactForce = !mDrawContactForce;
		glutPostRedisplay();
		break;
        case 'n': // choose next DoF		
		if (mActiveDofIndex == mDofNum-1) {
			mActiveDofIndex = 0;
		}
		else {
			mActiveDofIndex++;
		}
		std::cout << "DoF " << mActiveDofIndex << std::endl;
		break;
    case '=': // increase DoF value
	  	pose(mActiveDofIndex) = pose(mActiveDofIndex) + 0.1;
		mDofs[1][mActiveDofIndex] = mDofs[1][mActiveDofIndex] + 0.1;
	  	mSkels[1]->setPose(pose,true,true);
	  	std::cout << "value: " << pose(mActiveDofIndex) << std::endl;
	  	break;
	case '-': // decrease DoF value
	  	pose(mActiveDofIndex) = pose(mActiveDofIndex) - 0.1;
		mDofs[1][mActiveDofIndex] = pose(mActiveDofIndex) - 0.1;
	  	mSkels[1]->setPose(pose,true,true);
	  	std::cout << "value: " << pose(mActiveDofIndex) << std::endl;
	  	break;
    case 'N': // choose next DoF of the object		
		if (mObjActiveDofIndex == mObjDofNum-1) {
			mObjActiveDofIndex = 0;
		}
		else {
			mObjActiveDofIndex++;
		}
		std::cout << "DoF " << mObjActiveDofIndex << std::endl;
		break;
    case '+': // increase DoF value of the object
	  	objPose(mObjActiveDofIndex) = objPose(mObjActiveDofIndex) + 0.01;
		mDofs[2][mObjActiveDofIndex] = mDofs[2][mObjActiveDofIndex] + 0.01;
	  	mSkels[2]->setPose(objPose,true,true);
	  	std::cout << "value: " << objPose(mObjActiveDofIndex) << std::endl;
	  	break;
	case '_': // decrease DoF value of the object
	    objPose(mObjActiveDofIndex) = objPose(mObjActiveDofIndex) - 0.01;
		mDofs[2][mObjActiveDofIndex] = mDofs[2][mObjActiveDofIndex] - 0.01;
	  	mSkels[2]->setPose(objPose,true,true);
	  	std::cout << "value: " << objPose(mObjActiveDofIndex) << std::endl;   
		break;
	case 'x':
	  /*
	  extForce(4) = extForce(4) + 0.1;	  
	  mSkels[2]->setExternalForces(extForce);
	  */
	  extTorque = extTorque+0.1;
		break;
	case 'z':
	  /*
	  extForce(4) = extForce(4) - 0.1;
	  mSkels[2]->setExternalForces(extForce);
	  */
	  extTorque = extTorque-0.1;
		break;
    default:
        Win3D::keyboard(key,x,y);
    }

	
    glutPostRedisplay();
}

void MyWindow::click(int button, int state, int x, int y)
{
	mMouseDown = !mMouseDown;
	int mask = glutGetModifiers();
	if (mMouseDown)	{
		if(mMouseDown && mask == GLUT_ACTIVE_SHIFT){
			mZooming = true;
		}
		else if (mMouseDown && mask == GLUT_ACTIVE_ALT) {
			mRotate = true;
			mTrackBall.startBall(x,mWinHeight-y);
		}
		else if (button == GLUT_RIGHT_BUTTON) {
			mTranslate = true;
		}
		mMouseX = x;
		mMouseY = y;
	} 
	else {
		mTranslate = false;
		mRotate = false;
		mZooming = false;
	}
#ifdef WIN32
	ui.mouse(button,state,x,y);
#endif
	glutPostRedisplay();
}

void MyWindow::drag(int x, int y)
{
	double deltaX = x - mMouseX;
	double deltaY = y - mMouseY;

	mMouseX = x;
	mMouseY = y;

	if(mRotate){
		if(deltaX!=0 || deltaY!=0)
			mTrackBall.updateBall(x,mWinHeight-y);
	}
	if(mTranslate){
		Matrix3d rot = mTrackBall.getRotationMatrix();
		mTrans += rot.transpose()*Vector3d(deltaX, -deltaY, 0.0);
	}
	if(mZooming){
		mZoom += deltaY*0.01;
	}
#ifdef WIN32
	ui.mouseMotion(x,y);
#endif
	glutPostRedisplay();
}

void MyWindow::bake()
{
    int nContact = mConstraintHandle->getNumContacts();
    VectorXd state(mIndices.back() + 6 * nContact);
    for (unsigned int i = 0; i < mSkels.size(); i++)
        state.segment(mIndices[i], mDofs[i].size()) = mDofs[i];
    for (int i = 0; i < nContact; i++) {
        int begin = mIndices.back() + i * 6;
        state.segment(begin, 3) = mConstraintHandle->getCollisionChecker()->getContact(i).point;
        state.segment(begin + 3, 3) = mConstraintHandle->getCollisionChecker()->getContact(i).force;
    }
    mBakedStates.push_back(state);
	Vector3d rootTrans;
    rootTrans(0) = mSkels[1]->getJoint(0)->getTransform(0)->getDof(0)->getValue();
	rootTrans(1) = mSkels[1]->getJoint(0)->getTransform(0)->getDof(1)->getValue();
	rootTrans(2) = mSkels[1]->getJoint(0)->getTransform(0)->getDof(2)->getValue();
	mRootTrans.push_back(rootTrans);

}

void MyWindow::saveBake()
{
	string fileName = "rotate_ODE.txt";
	ofstream outFile(fileName.c_str());
	outFile.precision(10);
	outFile << mBakedStates.size() << std::endl;
	for (int i = 0; i < mBakedStates.size(); ++i) {
		outFile << mBakedStates[i].size() << "  ";
		outFile << mBakedStates[i].transpose() << std::endl;
	}

	std::cout << "save finished" << std::endl;
}

bool MyWindow::loadBake()
{
	mBakedStates.clear();

	string fileName = mBakeFileName;
	ifstream inFile(fileName.c_str());
	if (inFile.fail()) {
		return false;
	}
	inFile.precision(10);
	string buffer;
	inFile >> buffer;
	int nFrame = atoi(buffer.c_str());
	mBakedStates.resize(nFrame);

	for (int i = 0; i < nFrame; ++i) {
		inFile >> buffer;
		int stateSize = atoi(buffer.c_str());
		mBakedStates[i] = VectorXd::Zero(stateSize);
		for (int j = 0; j < stateSize; ++j) {
			inFile >> buffer;
			mBakedStates[i][j] = atof(buffer.c_str());
		}
	}
	
	return true;
}

// this function should assign tasks for controller, based on high level requirement, the trajectory of object, and actual motion of object
void MyWindow::updateTaskArray()
{
	
}

Eigen::Vector3d MyWindow::evalFingerTipTraj(Eigen::VectorXd _start, Eigen::VectorXd _end, int _curFrame, int _totalFrame)
{
	Eigen::Vector2d startPointB = _start;
	Eigen::Vector2d endPointB = _end;
	Eigen::Vector2d middlePointB;

	Eigen::Vector3d middlePointC;
	Eigen::Vector3d middlePointCW;

	double i = (double)(_curFrame)/(double)(_totalFrame);
	middlePointB = (endPointB-startPointB)*i+startPointB;

	for (int i = 0; i < 2; ++i) {
		if (endPointB(i)-startPointB(i) < -2.0) {
			middlePointB = endPointB;
		}
		else if (endPointB(i)-startPointB(i) > 2.0) {
			middlePointB = endPointB;
		}		
	}	

	middlePointC = ballToCartesian(middlePointB);
	middlePointCW = mSkels[2]->getNode(0)->evalWorldPos(middlePointC);

	return Vector3d(middlePointCW(0),middlePointCW(1),middlePointCW(2));
}

// this function should assign reference hand pose based on task and IK, can utilize the bounding sphere to avoid collision, and can be called on the fly as update task array 
void MyWindow::updateHandPose()
{
	Eigen::VectorXd currentPose;
	currentPose = mSkels[1]->getPose();
	// solve IK
	IKProblem prob(mSkels[1],mFingerNames);
	snopt::SnoptSolver solver(&prob);

	// set the position target for fingers, if the finger is not related to task, we can set the current position of the finger as the target, otherwise, we can use the contact point or the tip trajectory
	PositionConstraint* p;
	OrientationConstraint* ori;
	Vector3d target = Vector3d::Zero();
	for (int i = 0; i < mFingerNum; ++i) {
		p = prob.getConstraint(i);
		target = mSkels[2]->getNode(0)->evalWorldPos(mTargetContactPoints[i]);
		p->setTarget(target);

		
		if (mController->mActiveSimFrame[i] < mController->mActiveNumFrame[i] && mController->mActiveFingers[i]) {
			mController->mActiveSimFrame[i] += 3;
		}
		

		ori = prob.getOriConstraint(i);
		Eigen::Vector3d orientationVector = Eigen::Vector3d(0.0,0.0,-1.0);
		target = orientationVector;
		ori->setTarget(target);
	}


	solver.solve();
	for (int i = 0; i < mSkels[1]->getNumDofs(); ++i) {
		mController->mFingerEndPose[i] = prob.vars()[i]->mVal;
	}

	mSkels[1]->setPose(currentPose, true, true);

}

// this function should assign the contact points and contact forces, based on the contact point from collision detection, and do contact planning
void MyWindow::updateContact()
{

	mConstraintHandle->evaluateConstraint();

	mContactIndices.clear();
	mContactPoints.clear();
	mTargetContactForces.clear();

	for (int i = 0; i < mFingerNum; ++i) {
		mFingerTargetContactForces[i] = Vector3d::Zero();
		mFingerContactPoints[i] = Vector3d::Zero();
	}

	Eigen::Matrix4d transformMatrix = mSkels[2]->getNode(0)->getWorldTransform();
	Eigen::Matrix3d rotationMatrix = transformMatrix.topLeftCorner(3,3);

	std::vector<std::vector<Eigen::Vector3d> > contactPointsOnHand;
	std::vector<std::vector<Eigen::Vector3d> > contactNormalsOnHand;
	std::vector<std::vector<int> > contactIndicesOnHand;
	std::vector<int> numContact;
	std::vector<double> avePenetration;
	std::vector<int> contactEraseTotal;
	std::vector<int> contactEraseCount;
	std::vector<int> contactLeastKeep;
	contactPointsOnHand.resize(mFingerNum);
	contactNormalsOnHand.resize(mFingerNum);
	contactIndicesOnHand.resize(mFingerNum);
	numContact.resize(mFingerNum);
	avePenetration.resize(mFingerNum);
	contactEraseTotal.resize(mFingerNum);
	contactEraseCount.resize(mFingerNum);
	contactLeastKeep.resize(mFingerNum);

	for (int i = 0; i < mFingerNum; ++i) {
		numContact[i] = 0;
		avePenetration[i] = 0.0;
		contactEraseTotal[i] = 20;
		contactEraseCount[i] = 0;
		contactLeastKeep[i] = 4;
	}
	
	// calculate the contact number for each finger
	for (int i = 0; i < mConstraintHandle->getNumContacts(); ++i) {
		for (int j = 0; j < mFingerNum; ++j) {
			if (strcmp(mConstraintHandle->getCollisionChecker()->getContact(i).collisionNode1->getBodyNode()->getName(),mFingerNames[j].c_str()) == 0) {
				numContact[j]++;
				break;
			}
		}
	}

	// only keep part of contacts for each finger
	for (int i = mConstraintHandle->getNumContacts()-1; i >= 0; --i) {
		for (int j = 0; j < mFingerNum; ++j) {
			if (strcmp(mConstraintHandle->getCollisionChecker()->getContact(i).collisionNode1->getBodyNode()->getName(),mFingerNames[j].c_str()) == 0) {
				if (contactEraseCount[j] < contactEraseTotal[j] && contactEraseCount[j] < numContact[j]-contactLeastKeep[j]) {
					mConstraintHandle->getCollisionChecker()->mContacts.erase(mConstraintHandle->getCollisionChecker()->mContacts.begin()+i);
					contactEraseCount[j]++;
					break;
				}
			}
		}
	}
	
	for (int i = 0; i < mFingerNum; ++i) {
		numContact[i] = 0;
	}
// 	std::cout << "#contact:  " << mConstraintHandle->getNumContacts() << std::endl;

	// the original contact information
	for (int i = 0; i < mConstraintHandle->getNumContacts(); ++i) {
		for (int j = 0; j < mFingerNum; ++j) {
			if (strcmp(mConstraintHandle->getCollisionChecker()->getContact(i).collisionNode1->getBodyNode()->getName(),mFingerNames[j].c_str()) == 0) {
				numContact[j]++;
				contactPointsOnHand[j].push_back(mConstraintHandle->getCollisionChecker()->getContact(i).point);
				contactNormalsOnHand[j].push_back(mConstraintHandle->getCollisionChecker()->getContact(i).normal);
				contactIndicesOnHand[j].push_back(i);
				avePenetration[j] += mConstraintHandle->getCollisionChecker()->getContact(i).penetrationDepth;
				break;
			}	
		}
	}

	// average the contact point and penetration
	for (int i = 0; i < mFingerNum; ++i) {
		for (int j = 0; j < numContact[i]; ++j) {
			mFingerContactPoints[i] = mFingerContactPoints[i] + contactPointsOnHand[i][j];
		}
		if (numContact[i] > 0) {
			mFingerContactPoints[i] = mFingerContactPoints[i]/numContact[i];
			avePenetration[i] = avePenetration[i]/numContact[i];
		}		
	}

	// update finger state	
	Vector3d dirToThumb(-1.0,0.0,0.0);
	Vector3d dirToLF(1.0,0.0,0.0);
	double deviateAngleToThumb = acos(mObjOri.dot(dirToThumb)/(mObjOri.norm()*dirToThumb.norm()));
	double deviateAngleToLF = acos(mObjOri.dot(dirToLF)/(mObjOri.norm()*dirToLF.norm()));

	if (abs(deviateAngleToLF-1.57) < 0.01) {
	  for (int i = 0; i < mFingerNum; ++i) {
	    mController->mRestFingers[i] = true;
	    mController->mActiveFingers[i] = false;
	    mController->mContactFingers[i] = false;
	    mController->mInContactFingers[i] = false;
		mController->mActiveSimFrame[i] = 0;
	  }
	}
	// set the contact finger from acitve finger
	for (int i = 0; i < mFingerNum; ++i) {
		if (mController->mActiveSimFrame[i] >=  mController->mActiveNumFrame[i]) {
			// find the moment the finger changes from active to contact
			if (mController->mActiveFingers[i] == true) {
				mController->mTrackFingers[i] = true;	
			}
			mController->mActiveFingers[i] = false;
			mController->mContactFingers[i] = true;
			mController->mActiveSimFrame[i] = mController->mActiveNumFrame[i];
			mController->mTrackSimFrame[i]++;
		}	
	}
	
	// set the in contact fingers
	for (int i = 0; i < mFingerNum; ++i) {
		if (mController->mContactFingers[i] && numContact[i] > 0) {
			mController->mInContactFingers[i] = true;
			mController->mInContactFrame[i]++;
			mController->mTrackFingers[i] = false;
			mController->mControlFlag = true;
			mController->mTrackSimFrame[i] = 0;
		}
		else if (mController->mContactFingers[i] && mController->mTrackSimFrame[i] >= mController->mTrackNumFrame[i]) {
			mController->mTrackFingers[i] = false;
			mController->mInContactFingers[i] = false;
			mController->mControlFlag = true;
			mController->mTrackSimFrame[i] = 0;

		}
		else {
			mController->mInContactFingers[i] = false;
		}
	}
	
	// update the orientation of the object
	kinematics::BodyNode *object = mSkels[2]->getNode(0);
	Eigen::Matrix4d objTransformMatrix = object->getWorldTransform();
	Eigen::Matrix3d objRotationMatrix = objTransformMatrix.topLeftCorner(3,3);
	mObjOri = objRotationMatrix*Vector3d(0.0,1.0,0.0);

	// update the orientation of the palm
	kinematics::BodyNode *wrist = mSkels[1]->getNode(mPalmName.c_str());
	Eigen::Matrix4d palmTransformMatrix = wrist->getWorldTransform();
	Eigen::Matrix3d palmRotationMatrix = palmTransformMatrix.topLeftCorner(3,3);
	Eigen::Vector3d orientationVector = palmRotationMatrix*Vector3d(0.0,-1.0,0.0);

	// update the angle between the object and the palm
	double angle = acos(orientationVector.dot(mObjOri)/(orientationVector.norm()*mObjOri.norm()));
	if (abs(angle - mPalmObjAngle) < 0.001) {
		mController->mOriSimFrame++;
		
		if (mController->mOriSimFrame > mController->mOriNumFrame && mController->mOriFlag == true) { // the relative orientation between palm and object does not change

			mController->mOriFlag = false;
			mController->mOriSimFrame = 0;

		}
			
	}
	mPalmObjAngle = angle;

	//updateContactPoint();
	
	// set start point in ball coordinate
	for (int i = 0; i < mFingerNum; ++i) {
		if (numContact[i] > 0) {
			mContactPointsBallStart[i] = cartesianToBall(dart_math::xformHom(mSkels[2]->getNode(0)->getWorldInvTransform(), mFingerContactPoints[i]),mObjRadius);
		}		
		else { // if the finger has no contact with object, then use the target contact position as start position
			mContactPointsBallStart[i] = mContactPointsBallEnd[i];
		}
	}

		
	// solve the contact force
	// the normal information is based on the object
	std::vector<Eigen::Vector3d> normals;
	Eigen::Vector3d normalVector;
	for (int i = 0; i < mFingerNum; ++i) {
		if (mController->mInContactFingers[i]) {
			normalVector = -dart_math::xformHom(mSkels[2]->getNode(0)->getWorldInvTransform(), mFingerContactPoints[i]);
			normalVector = normalVector.normalized();
			normals.push_back(rotationMatrix*normalVector);
			mContactNormals[i] = rotationMatrix*normalVector;
			mContactPoints.push_back(mFingerContactPoints[i]);
			mContactIndices.push_back(mFingerTipIndices[i]);
		}	
	}
	
	// the force and torque should be based on the hand frame
	Eigen::VectorXd forceTorqueConstraint = VectorXd::Zero(6);
	forceTorqueConstraint = updateForceTorqueConstraint();
	Eigen::Vector3d totalForce = forceTorqueConstraint.head(3);
	totalForce = totalForce;
	Eigen::Vector3d totalTorque = forceTorqueConstraint.tail(3);
	Eigen::Vector3d objCOM = mSkels[2]->getWorldCOM();

	std::vector<bool> fingerFlag;
	fingerFlag.resize(mFingerNum);
	for (int i = 0; i < mFingerNum; ++i) {
		fingerFlag[i] = true;
	}
	
	// have no feedback for contact force
	int kp = 0.0;

	// solve the desired contact forces using approximated contact points
	if (mContactPoints.size() > 0) {
		ContactForceProblem prob(mContactPoints, normals, totalForce, totalTorque, objCOM);
		snopt::SnoptSolver solver(&prob);
		solver.solve();

		VectorXd z = solver.getState();

		for (int i = 0; i < mContactPoints.size(); ++i) {
			for (int j = 0; j < mFingerNum; ++j) {
				if (fingerFlag[j] && mController->mInContactFingers[j]) {
					fingerFlag[j] = false;
					mTargetContactForces.push_back(z.segment(i*3,3));
 					std::cout << "ID " << j << " contact force:" << std::endl << mTargetContactForces[i].transpose() << std::endl;
					break;
				}
			}
		}
	}	
	
}

void MyWindow::updateTaskTarget()
{
	// update task target example
	// state of hand
	Eigen::VectorXd state(mDofs[1].size()+mDofVels[1].size());
	state.head(mDofs[1].size()) = mDofs[1];
	state.tail(mDofVels[1].size()) = mDofVels[1];

	// other force
	Eigen::VectorXd otherForce = Eigen::VectorXd::Zero(mSkels[1]->getNumDofs());
	otherForce = mSkels[1]->getExternalForces();
	otherForce += mController->mGravityCompensationForce+mController->mObjControlForce+mController->mTrackForce+mController->mDampForce/*+mController->mOriForce+mController->mMaintainForce*/;
	for (int i = 0; i < mTasks.size(); ++i) {
		if (mTasks[i]->mTaskType == tasks::MAINTAIN) {
			dynamic_cast<tasks::MaintainTask*>(mTasks[i])->updateTask(state,Eigen::Vector3d(0.0,0.0,0.0),otherForce);
		}
		else if (mTasks[i]->mTaskType == tasks::ORI) {
			dynamic_cast<tasks::TrackOriTask*>(mTasks[i])->updateTask(state,Eigen::Vector3d(angleX,angleY,angleZ).normalized(),otherForce);
		}		
		else if (mTasks[i]->mTaskType == tasks::EE) {
			Eigen::Vector3d curPos;
			int endEffector = dynamic_cast<tasks::TrackEETask*>(mTasks[i])->getEEIndex();
			curPos = mSkels[1]->getNode(dynamic_cast<tasks::TrackEETask*>(mTasks[i])->getEEIndex())->getWorldCOM();
			if (i == 0) {
				curPos += dynamic_cast<tasks::TrackEETask*>(mTasks[i])->getEETraj()->getEndPoint() - mSkels[2]->getNode(0)->evalWorldPos(mTargetContactPoints[0]);
			}
			else if (i == 1) {
				curPos += dynamic_cast<tasks::TrackEETask*>(mTasks[i])->getEETraj()->getEndPoint() - mSkels[2]->getNode(0)->evalWorldPos(mTargetContactPoints[1]);
			}
			else if (i == 2) {
				curPos += dynamic_cast<tasks::TrackEETask*>(mTasks[i])->getEETraj()->getEndPoint() - mSkels[2]->getNode(0)->evalWorldPos(mTargetContactPoints[2]);
			}

			Eigen::Vector3d curTarget;
			if (i == 0) {
				curTarget = dynamic_cast<tasks::TrackEETask*>(mTasks[i])->getEETraj()->evalNextTarget(curPos) - mSkels[2]->getNode(0)->evalWorldPos(mTargetContactPoints[0]);
			}
			else if (i == 1) {
				curTarget = dynamic_cast<tasks::TrackEETask*>(mTasks[i])->getEETraj()->evalNextTarget(curPos) - mSkels[2]->getNode(0)->evalWorldPos(mTargetContactPoints[1]);
			}
			else if (i == 2) {
				curTarget = dynamic_cast<tasks::TrackEETask*>(mTasks[i])->getEETraj()->evalNextTarget(curPos) - mSkels[2]->getNode(0)->evalWorldPos(mTargetContactPoints[2]);
			}
			
			Eigen::Vector3d curTargetVel;
			curTargetVel = dynamic_cast<tasks::TrackEETask*>(mTasks[i])->getEETraj()->evalNextTargetVel(curPos);
			dynamic_cast<tasks::TrackEETask*>(mTasks[i])->setTargetVel(curTargetVel);
			if (i == 0) {
				dynamic_cast<tasks::TrackEETask*>(mTasks[i])->setFinalTarget(mSkels[2]->getNode(0)->evalWorldPos(mTargetContactPoints[0]));
			}
			else if (i == 1) {
				dynamic_cast<tasks::TrackEETask*>(mTasks[i])->setFinalTarget(mSkels[2]->getNode(0)->evalWorldPos(mTargetContactPoints[1]));
			}
			else if (i == 2) {
				dynamic_cast<tasks::TrackEETask*>(mTasks[i])->setFinalTarget(mSkels[2]->getNode(0)->evalWorldPos(mTargetContactPoints[2]));
			}

			// not use trajectory
			if (i == 0) {
				curTarget = mSkels[2]->getNode(0)->evalWorldPos(mTargetContactPoints[0]);
			}
			else if (i == 1) {
				curTarget = mSkels[2]->getNode(0)->evalWorldPos(mTargetContactPoints[1]);
			}
			else if (i == 2) {
				curTarget = mSkels[2]->getNode(0)->evalWorldPos(mTargetContactPoints[2]);
			}
			else if (i == 3) {
				curTarget = mSkels[2]->getNode(0)->evalWorldPos(mTargetContactPoints[3]);
			}
			else if (i == 4) {
				curTarget = mSkels[2]->getNode(0)->evalWorldPos(mTargetContactPoints[4]);
			}
			curTargetVel = Eigen::Vector3d(0.0,0.0,0.0);

			dynamic_cast<tasks::TrackEETask*>(mTasks[i])->updateTask(state,curTarget,otherForce);
		}
	}

}

void MyWindow::updateIntForce()
{
        mController->computeTorques(mDofs[1], mDofVels[1], mDofAccs[1], mDofs[2], mDofVels[2], mContactPoints, mTargetContactForces, mContactIndices, mTargetOri, mObjOri);	
	mSkels[1]->setInternalForces(mController->getTorques());
}

void MyWindow::updateExtForce()
{
  
	// compute all the constraint force, including contact force and joint limit force
  bool ODEFlag = true;

	mConstraintHandle->applyConstraintForcesHand(ODEFlag);

	std::vector<int> numContact;
	numContact.resize(mFingerNum);
	for (int i = 0; i < mFingerNum; ++i) {
		mContactForces[i] = Vector3d::Zero();
		numContact[i] = 0;
	}	

	// tangent relative velocity to be baked
	Eigen::VectorXd bakeTanRelVel = VectorXd::Zero(mConstraintHandle->getNumContacts()*3);

	for (int i = 0; i < mConstraintHandle->getNumContacts(); ++i) {
		for (int j = 0; j < mFingerNum; ++j) {
			if (strcmp(mConstraintHandle->getCollisionChecker()->getContact(i).collisionNode1->getBodyNode()->getName(),mFingerNames[j].c_str()) == 0) {
				numContact[j]++;
				mContactForces[j] = mContactForces[j] - mConstraintHandle->getCollisionChecker()->getContact(i).force;
				break;
			}
		}
		Vector3d normalForce = mConstraintHandle->getCollisionChecker()->getContact(i).force.dot(mConstraintHandle->getCollisionChecker()->getContact(i).normal.normalized())*mConstraintHandle->getCollisionChecker()->getContact(i).normal.normalized();
		Vector3d tangentForce = mConstraintHandle->getCollisionChecker()->getContact(i).force-normalForce;

		MatrixXd B = mConstraintHandle->getTangentBasisMatrix(mConstraintHandle->getCollisionChecker()->getContact(i).point, mConstraintHandle->getCollisionChecker()->getContact(i).normal);
		bakeTanRelVel.segment(i*3,3) = mConstraintHandle->mB.col(i*mFrictionBasis).dot(mConstraintHandle->mQDot)*B.col(0);
		for (int j = 1; j < mFrictionBasis/2; ++j) {
			bakeTanRelVel.segment(i*3,3) += mConstraintHandle->mB.col(i*mFrictionBasis+j*2).dot(mConstraintHandle->mQDot)*B.col(j*2);
		}	
		// bake the tangent relative velocity
		mBakedTanRelVels.push_back(bakeTanRelVel);
	}

	mController->mConstraintForce = mConstraintHandle->getTotalConstraintForce(1);
}

Eigen::VectorXd MyWindow::updateForceTorqueConstraint()
{

	// example of setting force torque constraint
	// update initial direction
	mFirstInitDir = mSkels[2]->getNode(0)->evalWorldPos(mFirstInitPoint)-Eigen::Vector3d(mDofs[2][0],mDofs[2][1],mDofs[2][2]);
	mSecondInitDir = mSkels[2]->getNode(0)->evalWorldPos(mSecondInitPoint)-Eigen::Vector3d(mDofs[2][0],mDofs[2][1],mDofs[2][2]);
	
	Vector3d v = applyHandTransformInvDir(dart_math::xformHomDir(mSkels[2]->getNode(0)->getWorldTransform(),static_cast<dynamics::BodyNodeDynamics*>(mSkels[2]->getNode(0))->mVelBody));
	Vector3d p = applyHandTransformInv(mSkels[2]->getNode(0)->getWorldCOM());
	double w = applyHandTransformInvDir(dart_math::xformHomDir(mSkels[2]->getNode(0)->getWorldTransform(),static_cast<dynamics::BodyNodeDynamics*>(mSkels[2]->getNode(0))->mOmegaBody)).norm();
	
	Eigen::Vector3d totalForce = Vector3d::Zero();
	Eigen::Vector3d totalTorque = Vector3d::Zero();
	Eigen::VectorXd forceTorqueConstraint = VectorXd::Zero(6);

	// the force should be transformed from hand frame to world frame
	forceTorqueConstraint.head(3) = applyHandTransformDir(totalForce);
	forceTorqueConstraint.tail(3) = applyHandTransformDir(totalTorque);

	forceTorqueConstraint(2) = -10.0;


	return forceTorqueConstraint;
}

void MyWindow::updateContactPoint()
{
	// example of target contact point
	// set target contact point and end point in ball coordinates
	double theta = 0.0;
	double phi = 0.0;
	double r = mObjRadius; // the radius of the approximate sphere

	
	theta = 2.0;
	phi = 1.2;
	Vector3d center(mDofs[2][0],mDofs[2][1],mDofs[2][2]);
	Vector3d position = center + Vector3d(r*cos(theta)*sin(phi),r*cos(phi),-r*sin(theta)*sin(phi));
	Vector3d localPosition = dart_math::xformHom(mSkels[2]->getNode(0)->getWorldInvTransform(), position);
	Vector2d localBallPosition = cartesianToBall(localPosition, r);
	mContactPointsBallEnd[0] = localBallPosition;
	mTargetContactPoints[0] = ballToCartesian(localBallPosition, r);

	theta = -2.5;
	phi = 1.5;
	position = center + Vector3d(r*cos(theta)*sin(phi),r*cos(phi),-r*sin(theta)*sin(phi));
	localPosition = dart_math::xformHom(mSkels[2]->getNode(0)->getWorldInvTransform(), position);
	localBallPosition = cartesianToBall(localPosition, r);
	mContactPointsBallEnd[1] = localBallPosition;
	mTargetContactPoints[1] = ballToCartesian(localBallPosition, r);

	theta = -1.8;
	phi = 1.5;
	position = center + Vector3d(r*cos(theta)*sin(phi),r*cos(phi),-r*sin(theta)*sin(phi));
	localPosition = dart_math::xformHom(mSkels[2]->getNode(0)->getWorldInvTransform(), position);
	localBallPosition = cartesianToBall(localPosition, r);
	mContactPointsBallEnd[2] = localBallPosition;
	mTargetContactPoints[2] = ballToCartesian(localBallPosition, r);

	theta = -1.0;
	phi = 1.5;
	position = center + Vector3d(r*cos(theta)*sin(phi),r*cos(phi),-r*sin(theta)*sin(phi));
	localPosition = dart_math::xformHom(mSkels[2]->getNode(0)->getWorldInvTransform(), position);
	localBallPosition = cartesianToBall(localPosition, r);
	mContactPointsBallEnd[3] = localBallPosition;
	mTargetContactPoints[3] = ballToCartesian(localBallPosition, r);

	theta = 0.5;
	phi = 1.5;
	position = center + Vector3d(r*cos(theta)*sin(phi),r*cos(phi),-r*sin(theta)*sin(phi));
	localPosition = dart_math::xformHom(mSkels[2]->getNode(0)->getWorldInvTransform(), position);
	localBallPosition = cartesianToBall(localPosition, r);
	mContactPointsBallEnd[4] = localBallPosition;
	mTargetContactPoints[4] = ballToCartesian(localBallPosition, r);
	
	
}

Eigen::Matrix4d MyWindow::evalHandTransform()
{
	return mSkels[1]->getNode(1)->getWorldTransform();
}

Eigen::Matrix4d MyWindow::evalHandTransformInv()
{
	return mSkels[1]->getNode(1)->getWorldInvTransform();
}

Eigen::Vector3d MyWindow::applyHandTransform(Eigen::Vector3d _x)
{
	return evalHandTransform().topLeftCorner(3,3)*_x + evalHandTransform().col(3).head(3);
}

Eigen::Vector3d MyWindow::applyHandTransformDir(Eigen::Vector3d _v)
{
	return evalHandTransform().topLeftCorner(3,3)*_v;
}

Eigen::Vector3d MyWindow::applyHandTransformInv(Eigen::Vector3d _x)
{
	return evalHandTransformInv().topLeftCorner(3,3)*_x + evalHandTransformInv().col(3).head(3);
}

Eigen::Vector3d MyWindow::applyHandTransformInvDir(Eigen::Vector3d _v)
{
	return evalHandTransformInv().topLeftCorner(3,3)*_v;
}

Eigen::Vector4d MyWindow::matrixToAxisAngle(Eigen::Matrix3d& _m)
{
	Eigen::Quaterniond q = dart_math::matrixToQuat(_m);
	Eigen::Vector3d exp = dart_math::quatToExp(q);
	Eigen::Vector4d a;
	a(3) = exp.norm();
	a.head(3) = exp.normalized();
	return a;
}

Eigen::Matrix3d MyWindow::axisAngleToMatrix(Eigen::Vector4d& _v)
{
	Eigen::Vector3d exp = _v(3)*_v.head(3);
	Eigen::Quaterniond q = dart_math::expToQuat(exp);
	Eigen::Matrix3d m = dart_math::quatToMatrix(q);
	return m;
}

Eigen::Matrix3d MyWindow::applyAxisAngle(Eigen::Vector4d& _v1, Eigen::Vector4d& _v2)
{
	Eigen::Matrix3d m1 = axisAngleToMatrix(_v1);
	Eigen::Matrix3d m2 = axisAngleToMatrix(_v2);
	return m2*m1;
}

Eigen::Vector3d MyWindow::evalVelOnObj(Eigen::Vector3d& _l)
{
	Eigen::MatrixXd Jv = MatrixXd::Zero(3,6);
	for (unsigned int i=0; i<6; i++) {
		VectorXd Ji = dart_math::xformHom(mSkels[2]->getNode(0)->getDerivWorldTransform(i),_l);
		Jv(0, i) = Ji(0);
		Jv(1, i) = Ji(1);
		Jv(2, i) = Ji(2);
	}
	return Jv*mDofVels[2];
}

bool MyWindow::forceAchievable(int _index, Eigen::Vector3d _point, Eigen::Vector3d _force)
{
	int numDepDofs = mSkels[1]->getNode(_index)->getNumDependentDofs()-4;
	MatrixXd J = MatrixXd::Zero(3, numDepDofs);
	for(int j=4; j<numDepDofs+4; j++) {
		J.col(j-4) = dart_math::xformHom(mSkels[1]->getNode(_index)->getDerivWorldTransform(j),_point);
	}

	MatrixXd B = J*J.transpose();
	FullPivLU<MatrixXd> lu_decompB(B);
	if (lu_decompB.rank() < 3) {
		return false;
	}
	
	MatrixXd A = B.inverse()*J;
	MatrixXd Aug = MatrixXd::Zero(3,A.cols()+1);
	Aug.block(0,0,3,A.cols()) = A;
	Aug.block(0,A.cols(),3,1) = _force;

	FullPivLU<MatrixXd> lu_decompA(A);
	FullPivLU<MatrixXd> lu_decompAug(Aug);
	if (lu_decompA.rank() == lu_decompAug.rank()) {
		return true;
	}
	else {
		return false;
	}
	
}

void MyWindow::setHandAngle(double _angle)
{
	// change the rotation axis to change the roll direction
	Vector3d palmRotateAxis(1.0,0.0,0.0);
	Eigen::Quaternion<double> q;
	q = (Eigen::Quaternion<double>)Eigen::AngleAxis<double>(_angle, palmRotateAxis);

	Vector3d palmInitOri = Vector3d(0.0,1.0,0.0).normalized();
	Vector3d palmOri = q*palmInitOri;

	angleX = palmOri(0);
	angleY = palmOri(1);
	angleZ = palmOri(2);	
}

void MyWindow::setHandTrans(Eigen::Vector3d _preOri, Eigen::Vector3d _axis)
{
	VectorXd pose = mSkels[1]->getPose();
	Vector3d curOri = pose.head(3);
	VectorXd prePose = pose;
	prePose.head(3) = _preOri;
	mSkels[1]->setPose(prePose);
	kinematics::BodyNode *wrist = mSkels[1]->getNode(mPalmName.c_str());
	Eigen::Matrix4d preWorldTransformation = wrist->getWorldTransform();
	Eigen::Matrix3d preWorldRotation = preWorldTransformation.topLeftCorner(3,3);
	Eigen::Vector3d preWorldTranslation = preWorldTransformation.block(0,3,3,1);
	mSkels[1]->setPose(pose);
	Eigen::Matrix4d curWorldTransformation = wrist->getWorldTransform();
	Eigen::Matrix3d curWorldRotation = curWorldTransformation.topLeftCorner(3,3);
	Eigen::Vector3d curWorldTranslation = preWorldRotation*_axis+preWorldTranslation-curWorldRotation*_axis;
	mSkels[1]->getJoint(0)->getTransform(0)->getDof(0)->setValue(mSkels[1]->getJoint(0)->getTransform(0)->getDof(0)->getValue()+(curWorldTranslation(0) - preWorldTranslation(0)));
	mSkels[1]->getJoint(0)->getTransform(0)->getDof(1)->setValue(mSkels[1]->getJoint(0)->getTransform(0)->getDof(1)->getValue()+(curWorldTranslation(1) - preWorldTranslation(1)));
	mSkels[1]->getJoint(0)->getTransform(0)->getDof(2)->setValue(mSkels[1]->getJoint(0)->getTransform(0)->getDof(2)->getValue()+(curWorldTranslation(2) - preWorldTranslation(2)));
	mSkels[1]->getJoint(0)->updateStaticTransform();
	mSkels[1]->setPose(pose);
}

int MyWindow::evalContactEdge()
{
	std::vector<bool> inContactEdges;
	inContactEdges.resize(mEdges.size());
	for (int i = 0; i < mEdges.size(); ++i) {
		inContactEdges[i] = false;
	}

	for (int i = 0; i < mConstraintHandle->getNumContacts(); ++i) {
		Eigen::Vector3d contactPos = mConstraintHandle->getCollisionChecker()->getContact(i).point;
		Vector3d contactLocalPos = dart_math::xformHom(mSkels[2]->getNode(0)->getWorldInvTransform(), contactPos);
		for (int j = 0; j < mEdges.size(); ++j) {
			if ((contactLocalPos.tail(2)-mEdges[j].tail(2)).norm() < 0.001) {
				inContactEdges[j] = true;
			}
		}
	}

	int contactEdgeIndex = -1;
	for (int i = 0; i < mEdges.size(); ++i) {
		if (inContactEdges[i] == true) {
			for (int j = 0; j < mEdges.size(); ++j) {
				if (j != i && inContactEdges[j]) { // there are more than one contact edges
					return -1;
				}
			}
			contactEdgeIndex = i;
		}
	}

	return contactEdgeIndex;
}

bool MyWindow::evalEdgeInContact(int _edgeIndex)
{
	for (int i = 0; i < mConstraintHandle->getNumContacts(); ++i) {
		Eigen::Vector3d contactPos = mConstraintHandle->getCollisionChecker()->getContact(i).point;
		Vector3d contactLocalPos = dart_math::xformHom(mSkels[2]->getNode(0)->getWorldInvTransform(), contactPos);
			if ((contactLocalPos.tail(2)-mEdges[_edgeIndex].tail(2)).norm() < 0.001) {
				return true;
			}
	}
	return false;
}

Eigen::Matrix3d MyWindow::evalObjOri()
{
  Eigen::Vector3d oriAxisX = (mCorners[2][1]-mCorners[2][0]).normalized();
  Eigen::Vector3d oriAxisY = (mCorners[3][1]-mCorners[2][1]).normalized();
  Eigen::Vector3d oriAxisZ = oriAxisX.cross(oriAxisY);
  Eigen::Vector3d axisX = (mSkels[2]->getNode(0)->evalWorldPos(mCorners[2][1])-mSkels[2]->getNode(0)->evalWorldPos(mCorners[2][0])).normalized();
  Eigen::Vector3d axisY = (mSkels[2]->getNode(0)->evalWorldPos(mCorners[3][1])-mSkels[2]->getNode(0)->evalWorldPos(mCorners[2][1])).normalized();
  Eigen::Vector3d axisZ = axisX.cross(axisY);
  Eigen::Matrix3d oriRotMat;
  oriRotMat.col(0) = oriAxisX;
  oriRotMat.col(1) = oriAxisY;
  oriRotMat.col(2) = oriAxisZ;
  Eigen::Matrix3d rotMat;
  rotMat.col(0) = axisX;
  rotMat.col(1) = axisY;
  rotMat.col(2) = axisZ;
  return rotMat*oriRotMat.inverse();
}

int MyWindow::evalUpFace() 
{
  int highVerIndex = -1;

  double height = 10.0;
  Eigen::Matrix3d rotMat = evalObjOri();
  Eigen::Vector3d transVec(mDofs[2](0),mDofs[2](1),mDofs[2](2));
  for (int i = 0; i < mCorners.size(); ++i) {
	  if (dart_math::xformHom(mSkels[1]->getNode(4)->getWorldInvTransform(), rotMat*mCorners[i][0]+transVec)(1) < height) {
		  height = dart_math::xformHom(mSkels[1]->getNode(4)->getWorldInvTransform(), rotMat*mCorners[i][0]+transVec)(1);
		  highVerIndex = i;
	  }
  }

  if (dart_math::xformHom(mSkels[1]->getNode(4)->getWorldInvTransform(), rotMat*mCorners[(highVerIndex+1)%mCorners.size()][0]+transVec)(1) < dart_math::xformHom(mSkels[1]->getNode(4)->getWorldInvTransform(), rotMat*mCorners[(highVerIndex-1)%mCorners.size()][0]+transVec)(1)) {
	  return ((highVerIndex+1)%mCorners.size());
  }
  else return highVerIndex;
}

// can be implemented as get every corners global coordinate, translate to local coordinate, and sort based on the coordinates in other dimensions
void MyWindow::evalHighCorners()
{
	int upFaceIndex = evalUpFace();
	mCurHighCorners[0] = dart_math::xformHom(mSkels[1]->getNode(4)->getWorldInvTransform(), mSkels[2]->getNode(0)->evalWorldPos(mCorners[(upFaceIndex-1)%mCorners.size()][0]));
	mCurHighCorners[1] = dart_math::xformHom(mSkels[1]->getNode(4)->getWorldInvTransform(), mSkels[2]->getNode(0)->evalWorldPos(mCorners[upFaceIndex][0]));
}

bool MyWindow::evalCornerChange()
{
	double cornerDis = 0.0;
	cornerDis = cornerDis + (mCurHighCorners[0]-mPreHighCorners[0]).norm() + (mCurHighCorners[1]-mPreHighCorners[1]).norm();
	if (cornerDis > 0.01) {
		return true;
	}
	else return false;
}

void MyWindow::evalRollDir()
{
	if (mCurHighCorners[0](2) > mPreHighCorners[0](2)) {
		mRollDir = 1;
	}
	else if (mCurHighCorners[0](2) < mPreHighCorners[0](2)) {
		mRollDir = -1;
	}
	else {
		mRollDir = 0;
	}
}
