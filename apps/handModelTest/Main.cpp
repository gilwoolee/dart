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
#include "dart/simulation/World.h"
#include "dart/utils/Paths.h"
#include "dart/utils/SkelParser.h"
#include "dart/utils/urdf/DartLoader.h"
#include "dart/utils/sdf/SdfParser.h"
#include "dart/utils/sdf/SoftSdfParser.h"

#include "MyWindow.h"

using namespace std;
using namespace Eigen;

using namespace dart;
using namespace dynamics;
using namespace simulation;
using namespace utils;

int main(int argc, char* argv[])
{
  World* world = new World();

  // URDF loader
  DartLoader dl;

  // Load skeleton files
  std::string handPathJohn     = DART_DATA_PATH"urdf/shadow_hand_john/model_arm_and_hand.sdf";
  std::string handPathJohn2    = DART_DATA_PATH"urdf/shadow_hand_john/model_forearm_and_hand_modified_for_test.sdf";
  std::string handPathJohnSoft = DART_DATA_PATH"urdf/shadow_hand_john/model_hand_only_soft.sdf";
  Skeleton* handSkel     = SoftSdfParser::readSkeleton(handPathJohn);
//  Skeleton* handSkel2    = SoftSdfParser::readSkeleton(handPathJohn2);
//  Skeleton* handSkelSoft = SoftSdfParser::readSkeleton(handPathJohnSoft);

  std::cout << handSkel << std::endl;

  world->addSkeleton(handSkel);

//  std::cout << "Num of skeletons: " << world->getNumSkeletons();

  // Create window and run main loop
  MyWindow window;
  window.setWorld(world);
  glutInit(&argc, argv);
  window.initWindow(640, 480, "Manipulator");
#ifdef WIN32
  glewInit();
#endif
  glutMainLoop();

  return 0;
}
