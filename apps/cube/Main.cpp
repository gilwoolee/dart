#include "MyWindow.h"
#include "kinematics/FileInfoSkel.hpp"
#include "dynamics/ConstraintDynamics.h"
#include "utils/Paths.h"
#include "robotics/parser/dart_parser/DartLoader.h"
#include "dynamics/SkeletonDynamics.h"

using namespace kinematics;
using namespace dynamics;
using namespace simulation;
using namespace std;
using namespace Eigen;

int main(int argc, char* argv[])
{
    // load a skeleton file
    FileInfoSkel<SkeletonDynamics> model, model2;
    model.loadFile(DART_DATA_PATH"/skel/ground2.skel", SKEL);
    model2.loadFile(DART_DATA_PATH"/skel/cube1.skel", SKEL);
    
    // create and initialize the world
    World *myWorld = new World();
    Vector3d gravity(0.0, -9.81, 0.0);
    myWorld->setGravity(gravity);
    myWorld->getCollisionHandle()->mMu = 1.5;

    ((SkeletonDynamics*)model.getSkel())->setImmobileState(true);
    myWorld->addSkeleton((SkeletonDynamics*)model.getSkel());
    myWorld->addSkeleton((SkeletonDynamics*)model2.getSkel());

    // create a window and link it to the world
    MyWindow window;
    window.setWorld(myWorld);
    window.evalN();
    window.evalGeometry();
    window.evalInertia();
    window.setInitVel(Eigen::Vector3d(0.0,0.0,0.0));
    window.evalAngles();

	VectorXd initPose = myWorld->getSkeleton(0)->getPose();
	initPose[5] = window.getAngle(0);
	myWorld->getSkeleton(0)->setPose(initPose);
	
	initPose.setZero();
	Vector3d localPos(-0.5,0.02+0.5*0.05,0.0);
	Vector3d worldPos = myWorld->getSkeleton(0)->getNode(0)->evalWorldPos(localPos);
	initPose.head(3) = worldPos;
	initPose[5] = window.getAngle(0);
	myWorld->getSkeleton(1)->setPose(initPose);

    
    cout << "space bar: simulation on/off" << endl;
    cout << "'p': playback/stop" << endl;
    cout << "'[' and ']': play one frame backward and forward" << endl;
    cout << "'v': visualization on/off" << endl;
    cout << "'1'--'4': programmed interaction" << endl;

    glutInit(&argc, argv);
    window.initWindow(640, 480, "Boxes");
    glutMainLoop();

    return 0;
}
