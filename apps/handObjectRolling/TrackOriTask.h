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

#ifndef	TRACK_ORI_TASK_H
#define TRACK_ORI_TASK_H

#include <string>
#include <Eigen/Dense>

#include "Task.h"

namespace tasks {

/// class TrackOriTask
class TrackOriTask : public Task
{
public:
  /// Constructor
  /// \param[in] _model
  /// \param[in] _eeName
  /// \param[in] _name
  TrackOriTask(const std::string& _name,
               dart::dynamics::Skeleton* _model,
               dart::dynamics::BodyNode* _targetBodyNode);

  /// Destructor
  virtual ~TrackOriTask();

  ///
  virtual void evalTorque();

  ///
  virtual void evalTaskFinish();

  ///
  void updateTask(Eigen::VectorXd _state,
                  Eigen::Vector3d _target,
                  Eigen::VectorXd _gravityCompensation);

  ///
  virtual Eigen::MatrixXd getNullSpace() const;

  ///
  virtual Eigen::MatrixXd getTaskSpace() const;

  ///
  Eigen::Vector3d getTarget();

  ///
  const std::string& getEEName() const;

  ///
  void setTarget(Eigen::Vector3d _target);

  ///
  Eigen::Vector3d evalTaskError();

  ///
  void setAccumulateError(Eigen::Vector3d _accumulateError);

protected:
  /// Target of the end effector
  Eigen::Vector3d mTarget;

  /// Name of the end effector
  std::string mEndEffectorName;

  ///
  double mPGain;

  ///
  double mVGain;

  ///
  double mIGain;

  ///
  Eigen::VectorXd mOtherForce;

  ///
  Eigen::Vector3d mAccumulateError;

  ///
  dart::dynamics::BodyNode* mBodyNode;
};

} // namespace tasks

#endif // #ifndef TRACK_ORI_TASK_H

