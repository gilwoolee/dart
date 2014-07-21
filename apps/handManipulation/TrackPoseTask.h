#ifndef	TRACK_POSE_TASK_H
#define TRACK_POSE_TASK_H

#include <vector>
#include "Task.h"

namespace tasks {

class TrackPoseTask : public Task {
public:
  TrackPoseTask(dart::dynamics::Skeleton* _model,
                const std::vector<int>& _dofIndices,
                char *_name);

  virtual void evalTorque();
  virtual void evalTaskFinish();
  void updateTask(Eigen::VectorXd _state, const std::vector<double>& _targets, const std::vector<int>& _dofIndices);
  void updateState(Eigen::VectorXd _state);
  virtual Eigen::MatrixXd getNullSpace() const;
  virtual Eigen::MatrixXd getTaskSpace() const;
  void setDamp(bool _damp);
  void setSigmoid(bool _sigmoid);

public:
  std::vector<double> mTargets;
  std::vector<int> mDofIndices; // the index of the dof needs to be tracked, has the same order as mTarget
  std::vector<double> mPGains;
  std::vector<double> mVGains;
  std::vector<double> mSmallPGains;
  std::vector<double> mSmallVGains;
  bool mDamp;                   // if true, then just have damping term
  bool mSmallGain;              // if true, then use small gain
  bool mSigmoidVGain;
};

} // namespace tasks

#endif // #ifndef TRACK_POSE_TASK_H

