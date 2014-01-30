/*
 * Copyright (c) 2014, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Jeongseok Lee <jslee02@gmail.com>
 *
 * Geoorgia Tech Graphics Lab and Humanoid Robotics Lab
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

#include <Eigen/Dense>
#include <gtest/gtest.h>

#include "TestHelpers.h"

#include "dart/common/Console.h"
#include "dart/math/Geometry.h"
#include "dart/math/Helpers.h"
#include "dart/dynamics/BodyNode.h"
#include "dart/dynamics/Skeleton.h"
#include "dart/simulation/World.h"
#include "dart/utils/SkelParser.h"
#include "dart/utils/Paths.h"

//==============================================================================
TEST(BODYNODE, Inertia)
{
  using namespace Eigen;
  using namespace dart;
  using namespace math;
  using namespace dynamics;

  int numRandomTest = 100;

  double minDiagonalTerm =    0.0;
  double maxDiagonalTerm = 1000.0;

  double minOffDiagonalTerm = -500.0;
  double maxOffDiagonalTerm =  500.0;

  double minMass = 0.001;
  double maxMass = 10000;

  double minOffset = -1000.0;
  double maxOffset = -1000.0;

  BodyNode* bodyNode = new BodyNode("Random Inertia BodyNode");

  for (int i = 0; i < numRandomTest; ++i)
  {
    // Random transformation
    Isometry3d T;

    // Random inertia
    double ixx = random(minDiagonalTerm, maxDiagonalTerm);
    double iyy = random(minDiagonalTerm, maxDiagonalTerm);
    double izz = random(minDiagonalTerm, maxDiagonalTerm);

    double ixy = random(minOffDiagonalTerm, maxOffDiagonalTerm);
    double ixz = random(minOffDiagonalTerm, maxOffDiagonalTerm);
    double iyz = random(minOffDiagonalTerm, maxOffDiagonalTerm);

    // Random mass
    double mass = random(minMass, maxMass);

    // Random offset
    Vector3d offset(random(minOffset, maxOffset),
                    random(minOffset, maxOffset),
                    random(minOffset, maxOffset));

    bodyNode;
    bodyNode->setInertia(ixx, iyy, izz, ixy, ixz, iyz);
    bodyNode->setMass(mass);
    bodyNode->setLocalCOM(offset);

    Matrix6d I = bodyNode->getInertia();
  }
}

//==============================================================================
int main(int argc, char* argv[])
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

