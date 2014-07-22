#ifndef _MYWINDOW_
#define _MYWINDOW_

#include "yui/Win3D.h"
#include "simulation/SimWindow.h"

class MyWindow : public simulation::SimWindow
{
 public:
 MyWindow(): SimWindow() 
        {
            mForce = Eigen::Vector3d::Zero();
			mRollNum = 0;
        }
    virtual ~MyWindow() {}
    
    virtual void timeStepping();
    virtual void drawSkels();
    virtual void keyboard(unsigned char key, int x, int y);
    void setInitVel(Eigen::Vector3d _vel);
    void evalN();
    void evalGeometry();
    void evalInertia();
    void evalAngles();
	void setGroundAngle(double _angle, Eigen::Vector3d _axis);
	double getAngle(int _index);
	int evalContactEdge();
 private:
    Eigen::Vector3d mForce;
    std::vector<double> mAngles;
    int mN;
    Eigen::Vector3d mInitVel;
    std::vector<double> mStartVels; // the velocity magnitude at the beginning of a rolling cycle
    std::vector<double> mEndVels; // the velocity magnitude at the end of a rolling cycle
    Eigen::Vector3d mCOM; // the COM with respect to local coordinate
    std::vector<Eigen::Vector3d> mEdges; // the edges of the polygon (pivoting edges) in the local coordinate, use the represent point in the 2D plane
    std::vector<double> mRs; // distance between the pivoting edge and COM
    std::vector<double> mAlphas; // the angle between COM and pivoting edge with the previous face
    std::vector<double> mPhis; // the angle between COM and pivoting edge with the next face
    std::vector<Eigen::Matrix3d> mIs; // the inertia matrices with respect to each pivoting edge
	int mRollNum;
};

#endif
