#include "MyWindow.h"
#include "kinematics/FileInfoSkel.hpp"
#include "kinematics/BodyNode.h"
#include "kinematics/Shape.h"
#include "utils/Paths.h"
#include "robotics/parser/dart_parser/DartLoader.h"
#include "dynamics/SkeletonDynamics.h"
#include <fstream>
#include <iostream>

using namespace kinematics;
using namespace dynamics;
using namespace Eigen;
using namespace std;

int main(int argc, char* argv[])
{  
    FileInfoSkel<SkeletonDynamics> model, model2, model3;
    model.loadFile(DART_DATA_PATH"/skel/ground2.skel", SKEL);
    model3.loadFile(DART_DATA_PATH"/skel/cube1.skel", SKEL);

    Vector3d red(1.0, 0.0, 0.0);
    Vector3d gray(0.9, 0.9, 0.9);
    model.getSkel()->getNode(0)->getShape(0)->setColor(gray);
    model3.getSkel()->getNode(0)->getShape(0)->setColor(red);

    DartLoader dl;
    std::string handSkeletonFile(DART_DATA_PATH"other/shadow_hand.urdf");
    dynamics::SkeletonDynamics* handSkeleton = dl.parseSkeleton(handSkeletonFile);

	MyWindow window((SkeletonDynamics*)model.getSkel(), handSkeleton, (SkeletonDynamics*)model3.getSkel(), NULL);
   
    glutInit(&argc, argv);
    window.initWindow(640, 480, "Manipulator");
#ifdef WIN32
	glewInit();
#endif
    glutMainLoop();

    return 0;
}
