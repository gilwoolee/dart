/*
 * Copyright (c) 2014, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Jeongseok Lee <jslee02@gmail.com>
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
 *   * This code incorporates portions of Open Dynamics Engine
 *     (Copyright (c) 2001-2004, Russell L. Smith. All rights
 *     reserved.) and portions of FCL (Copyright (c) 2011, Willow
 *     Garage, Inc. All rights reserved.), which were released under
 *     the same BSD license as below
 *
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

#include "dart/utils/Paths.h"
#include "dart/utils/SkelParser.h"
#include "dart/simulation/World.h"

#include "apps/inertiaPlanning/MyWindow.h"
#include "apps/inertiaPlanning/Controller.h"

int main(int argc, char* argv[])
{
  // load a skeleton file
  // create and initialize the world
  dart::simulation::World* myWorld
      = dart::utils::SkelParser::readWorld(DART_DATA_PATH"skel/2bodies.skel");
  assert(myWorld != NULL);

  Eigen::Vector3d gravity(0.0, -9.81, 0.0);
  Eigen::Vector3d zero = Eigen::Vector3d::Zero();

  myWorld->setGravity(zero);

  // create controller
  Controller* controller = new Controller(myWorld->getSkeleton("robot"));
  controller->setInitState(0, DART_RADIAN*160.0, DART_RADIAN*60.0, 0);
//  controller->setFinalState(DART_RADIAN*90.0, 0, DART_RADIAN*10.0, 0);
  controller->setDesiredRotation(DART_RADIAN*180.0);
  controller->setDuration(6.0);
  controller->init();
//  controller->off();

  // create a window and link it to the world
  MyWindow window;
  window.setWorld(myWorld);
  window.setController(controller);
  window.init();
//  window.setController(myController);

  std::cout << "space bar: simulation on/off" << std::endl;
  std::cout << "'p': playback/stop" << std::endl;
  std::cout << "'[' and ']': play one frame backward and forward" << std::endl;
  std::cout << "'v': visualization on/off" << std::endl;
  std::cout << "'1'--'6': programmed interaction" << std::endl;

  glutInit(&argc, argv);
  window.initWindow(640, 480, "2 bodies");
  glutMainLoop();

  return 0;
}
