/*
 * Copyright (c) 2014, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Jeffrey T. Bingham <bingjeff@gmail.com>,
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

#include <iostream>

#include "dart/math/Helpers.h"
#include "dart/dynamics/Skeleton.h"
#include "dart/simulation/World.h"
#include "dart/utils/Paths.h"
#include "dart/utils/urdf/DartLoader.h"

#include "apps/inertiaBot/ManualController.h"
#include "apps/inertiaBot/AutomaticController.h"
#include "apps/inertiaBot/CompositeController.h"
#include "apps/inertiaBot/MyWindow.h"

/* TODO List
  - Motion recorder: record trajectory and export it to text file
  - Write matlab code that read the text file
 */

using namespace std;
using namespace Eigen;
using namespace dart;
using namespace math;
using namespace dynamics;
using namespace simulation;
using namespace utils;

int main(int argc, char* argv[])
{
  // Load a URDF
  DartLoader dl;

  std::string groundBotFileName(DART_DATA_PATH"urdf/ground.urdf");
//  std::string inertiaBotFileName(DART_DATA_PATH"urdf/inertiabot3.urdf");
  std::string inertiaBotFileName(DART_DATA_PATH"urdf/inertiabot3_paddles.urdf");
  Skeleton* ground     = dl.parseSkeleton(groundBotFileName);
  Skeleton* inertiaBot = dl.parseSkeleton(inertiaBotFileName);

  // create and initialize the world
  World* world = new World();

//  Vector3d gravity(0.0, -1e-1, 0.0);
  Vector3d gravity(0.0, -0.157, 0.0);
//  Vector3d gravity(0.0, -0.05, 0.0);
//    Vector3d gravity(0.0, -9.81 / 5.0, 0.0);
//  Vector3d gravity(0.0, -9.81, 0.0);
//  Vector3d gravity(0.0, 0.0, 0.0);

  world->setGravity(gravity);
  world->addSkeleton(inertiaBot);
  world->addSkeleton(ground);
  world->setTimeStep(0.001);

  // Create controller
  CompositeController* controller
      = new CompositeController(inertiaBot, world->getConstraintSolver());

  // Create a window and link it to the world
  MyWindow window;
  window.setWorld(world);

  // Initial pose
  VectorXd q = inertiaBot->getPositions();
  q[0] = DART_RADIAN * 45.0;  // z-axis orientation
  q[1] = 0.0;                  // x-axis translation
//  q[2] = 5.0;                  // y-axis translation
  q[2] = 2.20;                  // y-axis translation
//  q[2] = 0.5;                  // y-axis translation
  q[3] = DART_RADIAN *  0.0;   // angle of joint 1
  q[4] = DART_RADIAN * -0.0;   // angle of joint 2
  inertiaBot->setPositions(q);

  // Set target impact pose
  Vector3d impactPose = Vector3d::Zero();
  impactPose[0] = DART_RADIAN * 75.0;
  impactPose[1] = DART_RADIAN *  0.0;
  impactPose[2] = DART_RADIAN * -0.0;
  controller->getAutomaticController()->setImpactPose(impactPose);

  // Big-triangle-path or Small-triangle-path
  controller->getAutomaticController()->setUseSmallTrianglePath(true);

  // On-Off falling controller (able to on-off by pressing 'a' during rolling
  // motion)
  controller->getAutomaticController()->onOffFallingController(true);

  // On-Off rolling controller (able to on-off by pressing 'a' during rolling
  // motion)
  controller->getAutomaticController()->onOffRollingController(true);

  //
  controller->getAutomaticController()->setReplanningThreshold(
        DART_RADIAN * 5.0);

  //
  controller->getAutomaticController()->setStationaryCasePlanningTime(5.0);

  //
  controller->getAutomaticController()->setRollingJointAngle(DART_RADIAN * 90.0);

  // Test code
  math::seedRand();

  std::cout << "space bar: simulation on/off" << std::endl;
  std::cout << "'p': playback/stop" << std::endl;
  std::cout << "'[' and ']': play one frame backward and forward" << std::endl;
  std::cout << "'v': visualization on/off" << std::endl;
  std::cout << "'0': save trajectories" << std::endl;
  std::cout << "'g': manual control" << std::endl;
  std::cout << "'h': automatic control" << std::endl;
  std::cout << "'r': replan" << std::endl;

  glutInit(&argc, argv);
  window.setController(controller);
  window.initWindow(640, 480, "InertiaBot");
  glutMainLoop();

  return 0;
}
