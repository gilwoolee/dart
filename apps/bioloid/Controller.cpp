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

#include "apps/bioloid/Controller.h"

#include <iostream>
#include <fstream>

using namespace std;

using namespace Eigen;

using namespace dart;
using namespace constraint;
using namespace dynamics;
using namespace simulation;

Controller::Controller(Skeleton* _skel,
                       World* _world)
  : mSkel(_skel),
    mWorld(_world)
{
}

Controller::~Controller()
{
}

void Controller::prestep(double _currentTime)
{
}

void Controller::activate(double _currentTime)
{
}

void Controller::deactivate(double _currentTime)
{
}

void Controller::update(double _time)
{
  mCurrentTime = _time;
}

void Controller::keyboard(unsigned char _key)
{
}

Skeleton* Controller::getSkeleton()
{
  return mSkel;
}

void Controller::printDebugInfo() const
{
  std::cout << "[BioloidGP]"  << std::endl
            << " NUM NODES : " << mSkel->getNumBodyNodes() << std::endl
            << " NUM DOF   : " << mSkel->getNumDofs() << std::endl
            << " NUM JOINTS: " << mSkel->getNumBodyNodes() << std::endl;

  for(size_t i = 0; i < mSkel->getNumBodyNodes(); ++i)
  {
    Joint* joint = mSkel->getJoint(i);
    BodyNode* body = mSkel->getBodyNode(i);
    BodyNode* parentBody = mSkel->getBodyNode(i)->getParentBodyNode();

    std::cout << "  Joint [" << i << "]: "
              << joint->getName()
              << " (" << joint->getNumDofs() << ")"
              << std::endl;
    if (parentBody != NULL)
    {
      std::cout << "    Parent body: " << parentBody->getName() << std::endl;
    }

    std::cout << "    Child body : " << body->getName() << std::endl;
  }
}

void Controller::printAngularVelocityAndMomentumNorms(const string& _fileName)
{
  std::ofstream file;
  std::string fullName = std::string(DART_DATA_PATH) + _fileName;
  file.open(fullName.c_str());

  int nFrames = mAngVelocityNorms.size();

  for (int i = 0; i < nFrames; ++i)
  {
    file << mTime[i] << " "
         << mAngVelocityNorms[i] << " "
         << mAngMomentumNorms[i] << std::endl;
  }

  file.close();

  std::cout << "Angular info is saved to [" << _fileName << "]" << std::endl;
}
