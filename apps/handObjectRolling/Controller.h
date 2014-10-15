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

#ifndef APPS_HANDOBJECTROLLING_CONTROLLER_H_
#define APPS_HANDOBJECTROLLING_CONTROLLER_H_

#include <vector>

#include <Eigen/Dense>

#include "RollingAngle.h"

extern double gAngleX;
extern double gAngleY;
extern double gAngleZ;
extern double gHandPosX;
extern double gHandPosY;
extern double gHandPosZ;
extern double gObjPosX;
extern double gObjPosY;
extern double gObjPosZ;

namespace dart {
namespace dynamics {
class BodyNode;
class Skeleton;
}  // namespace dynamics
namespace constraint {
class ConstraintSolver;
class WeldJointConstraint;
}  // namespace constraint
namespace collision {
class CollisionDetector;
}
namespace simulation {
class World;
}
}  // namespace dart

class StateMachine;

/// \brief In-hand manipulation controller
class Controller
{
public:
  /// \brief Constructor
  Controller(
      dart::simulation::World*  _world,
      dart::dynamics::Skeleton* _ground,
      dart::dynamics::Skeleton* _shadowHand,
      dart::dynamics::Skeleton* _object);

  /// Destructor
  virtual ~Controller();

  /// \brief
  void setInitialTransformation();

  /// \brief
  void backupInitialStates();

  /// \brief
  void restoreInitialStates();

  /// \brief Called before every simulation time step in MyWindow class.
  ///
  /// Compute control force and apply it to Atlas robot
  virtual void update(double _currentTime);

  //----------------------------------------------------------------------------
  // Target object
  //----------------------------------------------------------------------------

  void setObjectInfo(std::vector<Eigen::Vector3d> _edges, double _halfDepthX);

  //----------------------------------------------------------------------------
  // Task information
  //----------------------------------------------------------------------------

  /// \brief
  ///
  /// This function should assign reference hand pose based on task and IK, can
  /// utilize the bounding sphere to avoid collision, and can be called on the
  /// fly as update task array.
  void updateHandPose() {}

  /// \biref
  void setPose();

  /// \brief
  ///
  /// This function should assign the contact points and contact forces, based
  /// on the contact point from collision detection, and do contact planning.
  void updateContact();

  /// \brief Compute gAngleX, gAngleY, gAngleZ
  void setHandAngle(double _angle);

  /// \brief Get edge number in contact, or -1 if multiple edges are in contact
  int evalContactEdge();

  /// \brief Return true if _edgeIndex is in contact
  bool isEdgeInContact(int _edgeIndex);

  /// \brief Return the orientation of the object
  Eigen::Matrix3d evalObjOri();

  /// \brief
  int evalUpFace();

  //----------------------------------------------------------------------------
  // Compute control forces
  //----------------------------------------------------------------------------

  /// \brief Update control force for the hand
  void computeHandTotalControlForce();

  /// \brief
  void computeHandConstraintForce();

  /// \brief
  void computeHandObjControlForce();

  /// \brief
  void computeTrackForce();

  /// \brief
  void computeTaskForce();

  /// \brief
  void computeHandTaskForce();

  /// \brief Update control force for the hand
  void computeHandGravityCompensationForce();

  /// \brief
  void computeHandDampForce();

  /// \brief
  void computeHandOriForce();

  /// \brief
  void computeHandMaintainForce();

  //----------------------------------------------------------------------------
  // Utility function
  //----------------------------------------------------------------------------



  /// \brief
  Eigen::VectorXd computeOtherForces();



  /// \brief Keyboard control
  void keyboard(unsigned char _key, int _x, int _y, double _currentTime);

  /// \brief Print debug information
  void printDebugInfo() const;

  /// \brief Reset
  void reset();

private:
  RollingAngle mRollingAngleEvaluator;

  /// \brief World
  dart::simulation::World* mWorld;

  /// \brief Ground
  dart::dynamics::Skeleton* mGround;

  /// \brief Shadow hand
  dart::dynamics::Skeleton* mHand;

  /// \brief Palm of the shadow hand
  dart::dynamics::BodyNode* mPalm;

  /// Object skeleton to be manipulated
  dart::dynamics::Skeleton* mObjectSkel;

  /// Object body node to be manipulated
  dart::dynamics::BodyNode* mObject;

  /// Conllision detector
  dart::constraint::ConstraintSolver* mConstratinSolver;

  /// Conllision detector
  dart::collision::CollisionDetector* mCollisionDetector;

  ///
  double mTimestep;

  ///
  int mHandDof;


  ///
  int mFingerNum;

  ///
  std::vector<std::string> mFingerNames;

  ///
  std::vector<std::string> mFingerRootNames;

  ///
  std::vector<int> mFingerTipIndices;

  /// Initial generalized positions of the ground
  Eigen::VectorXd mGroundInitialPositions;

  /// Initial generalized positions of the hand
  Eigen::VectorXd mHandInitialPositions;

  /// Initial generalized positions of the object
  Eigen::VectorXd mObjectInitialPositions;

  /// Backup state of the object
  Eigen::VectorXd mObjectStateBackup;

  /// Backup state of the hand
  Eigen::VectorXd mHandStateBackup;

  //----------------------------------------------------------------------------
  // Task information
  //----------------------------------------------------------------------------

  /// the edges of the polygon (pivoting edges) in the local coordinate, use the
  /// represent point in the 2D plane
  std::vector<Eigen::Vector3d> mEdges;

  /// the corners of the polygon in the local coodinate, the first dimension is
  /// corresponding to edges, the second dimension is two corners for an edge,
  /// the first one has positive coordinate (close to little finger), the second
  /// one has negative coordinate
  std::vector<std::vector<Eigen::Vector3d> > mCorners;

  /// Current rolling number
  int mRollNum;

  ///
  int mRollBackNum;

  /// Total number of rollings to control
  int mN;

  ///
  int mBackN;

  ///
  int mRollBackIndex;

  /// Target tilting angles to roll the object on the hand
  std::vector<double> mAngles;

  ///
  int mPreContactEdge;

  ///
  Eigen::Vector4d mPreOri;

  ///
  int mUpFace;

  ///
  std::vector<Eigen::Vector3d> mPreHighCorners;

  ///
  std::vector<Eigen::Vector3d> mCurHighCorners;

  ///
  int mRollDir;

  //----------------------------------------------------------------------------
  // Control
  //----------------------------------------------------------------------------

  /// Backup state of the hand
  Eigen::VectorXd mHandControlForce;

  /// Gravity compensation forces;
  Eigen::VectorXd mHandGravityCompensationForce;

  ///
  Eigen::VectorXd mHandObjControlForce;

  ///
  Eigen::VectorXd mHandOriForce;

  ///
  Eigen::VectorXd mHandTrackForce;

  ///
  Eigen::VectorXd mHandTaskForce;

  ///
  Eigen::VectorXd mHandMaintainForce;

  ///
  Eigen::VectorXd mHandConstraintForce;

  ///
  Eigen::VectorXd mHandSPDDampForce;

  //----------------------------------------------------------------------------

  Eigen::MatrixXd mKd;
  Eigen::MatrixXd mKp;

  //----------------------------------------------------------------------------

  ///
  bool mOriFlag;

  ///
  Eigen::Vector3d mPreOriTarget;

  ///
  Eigen::Vector3d mAccumulateOriError;

  ///
  Eigen::VectorXd mDesiredDofs;

  ///
  Eigen::VectorXi mOriDofs;
};

#endif  // APPS_HANDOBJECTROLLING_CONTROLLER_H_
