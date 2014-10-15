/*
 * Copyright (c) 2014, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Yunfei Bai <ybai30@mail.gatech.edu>,
 *            Jeongseok Lee <jslee02@gmail.com>
 *
 * Georgia Tech Graphics Lab and Humanoid Robotics Lab
 *
 * Directed by Prof. C. Karen Liu and Prof. Mike Stilman
 * <karenliu@cc.gatech.edu> <mstilman@cc.gatech.edu>
 *
 * This file is provided under the following "BSD-style" License:
 *   Redistribution and use in source and binary forms, with or
 *   without modification, are permitted provided that the following
 *   conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 *   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 */

#include <fstream>
#include <iostream>

#include "dart/dynamics/BodyNode.h"
#include "dart/dynamics/Shape.h"
#include "dart/dynamics/Skeleton.h"
#include "dart/constraint/ConstraintSolver.h"
#include "dart/collision/bullet/BulletCollisionDetector.h"
#include "dart/simulation/World.h"
#include "dart/utils/Paths.h"
#include "dart/utils/SkelParser.h"
#include "dart/utils/urdf/DartLoader.h"
#include "dart/utils/sdf/SdfParser.h"
#include "dart/utils/sdf/SoftSdfParser.h"

#include "MyWindow.h"
#include "Controller.h"

using namespace std;
using namespace Eigen;

using namespace dart;
using namespace dynamics;
using namespace simulation;
using namespace utils;

int main(int argc, char* argv[])
{
  // URDF loader
  DartLoader dl;

  // Setting colors
  Vector3d gray(0.9, 0.9, 0.9);
  Vector3d red(1.0, 0.0, 0.0);

  // Load skeleton files
  std::string groundPath = DART_DATA_PATH"skel/ground3.skel";
  Skeleton*   groundSkel = SkelParser::readSkeleton(groundPath);
  groundSkel->getBodyNode(0)->getVisualizationShape(0)->setColor(gray);

  std::string cubePath1 = DART_DATA_PATH"skel/cube1.skel";
  Skeleton*   cubeSkel1 = SkelParser::readSkeleton(cubePath1);
  cubeSkel1->getBodyNode(0)->getVisualizationShape(0)->setColor(red);

  std::string cubePath2 = DART_DATA_PATH"skel/cube2.skel";
  Skeleton*   cubeSkel2 = SkelParser::readSkeleton(cubePath2);
  cubeSkel2->getBodyNode(0)->getVisualizationShape(0)->setColor(red);

  std::string cubePath3 = DART_DATA_PATH"skel/cube3.skel";
  Skeleton*   cubeSkel3 = SkelParser::readSkeleton(cubePath3);
  cubeSkel3->getBodyNode(0)->getVisualizationShape(0)->setColor(red);

  std::string cubePath4 = DART_DATA_PATH"skel/cube4.skel";
  Skeleton*   cubeSkel4 = SkelParser::readSkeleton(cubePath4);
  cubeSkel4->getBodyNode(0)->getVisualizationShape(0)->setColor(red);

  std::string pathArmAndHandUrdf = DART_DATA_PATH"urdf/shadow_hand_john/model_arm_and_hand.urdf";
  Skeleton*   armAndHandUrdf     = dl.parseSkeleton(pathArmAndHandUrdf);

  std::string pathArmAndHandSdf = DART_DATA_PATH"urdf/shadow_hand_john/model_arm_and_hand.sdf";
  Skeleton*   armAndHandSdf     = SoftSdfParser::readSkeleton(pathArmAndHandSdf);

  std::string pathForearmAndHandUrdf = DART_DATA_PATH"urdf/shadow_hand_john/model_forearm_and_hand_JS.urdf";
  Skeleton*   forearmAndHandUrdf     = dl.parseSkeleton(pathForearmAndHandUrdf);

  std::string pathForearmAndHandSdf = DART_DATA_PATH"urdf/shadow_hand_john/model_forearm_and_hand_JS.sdf";
  Skeleton*   forearmAndHandSdf     = SoftSdfParser::readSkeleton(pathForearmAndHandSdf);

  std::string pathForearmAndSoftHandSdf = DART_DATA_PATH"urdf/shadow_hand_john/model_forearm_and_soft_hand.sdf";
  Skeleton*   forearmAndSoftHandSdf     = SoftSdfParser::readSkeleton(pathForearmAndSoftHandSdf);

  // Create empty soft world
  World* world = new World;

//  world->getConstraintSolver()->setCollisionDetector(new dart::collision::BulletCollisionDetector());

  world->addSkeleton(groundSkel);
//  world->addSkeleton(armAndHandUrdf);
//  world->addSkeleton(armAndHandSdf);
//  world->addSkeleton(forearmAndHandUrdf);
//  world->addSkeleton(forearmAndHandSdf);
  world->addSkeleton(forearmAndSoftHandSdf);
//  world->addSkeleton(cubeSkel1);
  world->addSkeleton(cubeSkel2);
//  world->addSkeleton(cubeSkel3);
//  world->addSkeleton(cubeSkel4);

  // Create window and run main loop
  Controller* controller
      = new Controller(world, groundSkel, forearmAndSoftHandSdf, cubeSkel2);



  MyWindow window(controller);
  window.setWorld(world);
  glutInit(&argc, argv);
  window.initWindow(640, 480, "Manipulator");
#ifdef WIN32
  glewInit();
#endif
  glutMainLoop();

  return 0;
}
