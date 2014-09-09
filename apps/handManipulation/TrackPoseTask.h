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

#ifndef	TRACK_POSE_TASK_H
#define TRACK_POSE_TASK_H

#include <vector>
#include "Task.h"

namespace tasks {

/// class TrackPoseTask
class TrackPoseTask : public Task
{
public:
  /// Constructor
  TrackPoseTask(dart::dynamics::Skeleton* _model,
                const std::vector<int>& _dofIndices,
                char *_name);

  /// Destructor
  virtual ~TrackPoseTask();

  ///
  virtual void evalTorque();

  ///
  virtual void evalTaskFinish();

  ///
  void updateTask(Eigen::VectorXd _state,
                  const std::vector<double>& _targets,
                  const std::vector<int>& _dofIndices);

  ///
  void updateState(Eigen::VectorXd _state);

  ///
  virtual Eigen::MatrixXd getNullSpace() const;

  ///
  virtual Eigen::MatrixXd getTaskSpace() const;

  ///
  void setDamp(bool _damp);

  ///
  void setSigmoid(bool _sigmoid);

public:
  ///
  std::vector<double> mTargets;

  /// The index of the dof needs to be tracked, has the same order as mTarget
  std::vector<int> mDofIndices;

  ///
  std::vector<double> mPGains;

  ///
  std::vector<double> mVGains;

  ///
  std::vector<double> mSmallPGains;

  ///
  std::vector<double> mSmallVGains;

  /// if true, then just have damping term
  bool mDamp;

  /// If true, then use small gain
  bool mSmallGain;

  ///
  bool mSigmoidVGain;
};

} // namespace tasks

#endif // #ifndef TRACK_POSE_TASK_H

