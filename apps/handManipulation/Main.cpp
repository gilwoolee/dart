/*
 * Copyright (c) 2013-2014, Georgia Tech Research Corporation
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

#include "MyWindow.h"

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

  // Load skeleton files
  const std::string& groundPath = DART_DATA_PATH"skel/ground2.skel";
  const std::string& cubePath   = DART_DATA_PATH"skel/cube1.skel";
  const std::string& handPath   = DART_DATA_PATH"urdf/shadow_hand.urdf";
  Skeleton* groundSkel = SkelParser::readSkeleton(groundPath);
  Skeleton* cubeSkel   = SkelParser::readSkeleton(cubePath);
  Skeleton* handSkel   = dl.parseSkeleton(handPath);

  // Setting colors
  Vector3d gray(0.9, 0.9, 0.9);
  Vector3d red(1.0, 0.0, 0.0);
  groundSkel->getBodyNode(0)->getVisualizationShape(0)->setColor(gray);
  cubeSkel->getBodyNode(0)->getVisualizationShape(0)->setColor(red);

  // Create window and run main loop
  MyWindow window(groundSkel, handSkel, cubeSkel, NULL);
  glutInit(&argc, argv);
  window.initWindow(640, 480, "Manipulator");
#ifdef WIN32
  glewInit();
#endif
  glutMainLoop();

  return 0;
}
