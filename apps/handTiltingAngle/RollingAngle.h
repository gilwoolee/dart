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

#ifndef APPS_HANDTILTINGANGLE_ROLLINGANGLE_H_
#define APPS_HANDTILTINGANGLE_ROLLINGANGLE_H_

#include "dart/gui/SimWindow.h"

#include "dart/dynamics/BodyNode.h"

/// class MyWindow
class RollingAngle
{
public:
  ///
  RollingAngle();

  ///
  RollingAngle(dart::dynamics::BodyNode* _body,
               const Eigen::Vector3d& _gravity);

  ///
  virtual ~RollingAngle();

  ///
  void setBodyNode(dart::dynamics::BodyNode* _body);

  ///
  void setGravity(const Eigen::Vector3d& _gravity);

  ///
  void updateAngles();

  ///
  int getNumAngles() const;

  ///
  double getAngle(int _index);

  ///
  const std::vector<Eigen::Vector3d> getEdges() const;

protected:
  ///
  void setInitVel(const Eigen::Vector3d& _vel);

  ///
  void evalN();

  ///
  void evalGeometry();

  ///
  void evalInertia();

  ///
  void evalAngles();

private:
  ///
  dart::dynamics::BodyNode* mBodyNode;

  ///
  Eigen::Vector3d mGravity;

  ///
  Eigen::Vector3d mForce;

  ///
  std::vector<double> mAngles;

  /// Number of desired rollings. Note add 1 to the number of desired rollings.
  /// For example, use 3 for 2 rollings.
  int mN;

  ///
  Eigen::Vector3d mInitVel;

  /// the velocity magnitude at the beginning of a rolling cycle
  std::vector<double> mStartVels;

  /// the velocity magnitude at the end of a rolling cycle
  std::vector<double> mEndVels;

  /// the COM with respect to local coordinate
  Eigen::Vector3d mCOM;

  /// the edges of the polygon (pivoting edges) in the local coordinate, use
  /// the represent point in the 2D plane
  std::vector<Eigen::Vector3d> mEdges;

  /// distance between the pivoting edge and COM
  std::vector<double> mRs;

  /// the angle between COM and pivoting edge with the previous face
  std::vector<double> mAlphas;

  /// the angle between COM and pivoting edge with the next face
  std::vector<double> mPhis;

  /// the inertia matrices with respect to each pivoting edge
  std::vector<Eigen::Matrix3d> mIs;

  ///
  int mRollNum;
};

#endif  // APPS_HANDTILTINGANGLE_ROLLINGANGLE_H_
