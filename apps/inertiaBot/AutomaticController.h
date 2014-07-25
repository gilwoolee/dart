/*
 * Copyright (c) 2014, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Jeffrey T. Bingham <bingjeff@gmail.com>,
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

#ifndef APPS_INERTIABOT_AUTOMATICCONTROLLER_H_
#define APPS_INERTIABOT_AUTOMATICCONTROLLER_H_

#include <Eigen/Dense>
#include <nlopt.h>

#include "dart/math/MathTypes.h"

#include "apps/inertiaBot/Controller.h"
#include "apps/inertiaBot/Motion.h"

namespace dart {
namespace dynamics {
class Skeleton;
}  // namespace dynamics
}  // namespace dart

typedef struct
{
    double a, b;
} my_constraint_data;

double myfunc(unsigned n, const double *x, double *grad, void *my_func_data);
double myconstraint(unsigned n, const double *x, double *grad, void *data);
void doOpt();

/// \brief
class AutomaticController : public Controller
{
public:
  enum ControlPhase
  {
    CP_FALLING,
    CP_ROLLING
  };

  /// \brief Constructor
  AutomaticController(dart::dynamics::Skeleton* _skel,
                      dart::constraint::ConstraintSolver* _constDyn);

  /// \brief Destructor
  virtual ~AutomaticController();

  //------------------------- class Controller ---------------------------------
  // Documentation inherited.
  virtual void prestep(double _currentTime);

  // Documentation inherited.
  virtual void activate(double _currentTime);

  // Documentation inherited.
  virtual void deactivate(double _currentTime);

  // Documentation inherited.
  virtual void update(double _time);

  // Documentation inherited.
  virtual const Eigen::VectorXd& getTorques() const;

  // Documentation inherited.
  virtual double getTorque(int _index) const;

  // Documentation inherited.
  virtual void keyboard(unsigned char _key);

  //----------------------------------------------------------------------------
  /// \brief Set impact pose.
  /// \param[in] _pos Impact pose [q0 q1 q2], q0 orientation of the root body.
  ///                 q1 and q2 are joint angles.
  void setImpactPose(const Eigen::Vector3d& _pos);

  /// \brief
  void setUseSmallTrianglePath(bool _v);

  /// \brief
  void onOffFallingController(bool _v);

  /// \brief
  void onOffRollingController(bool _v);

  /// \brief Set replanning threshold
  void setReplanningThreshold(double _v);

  /// \brief Get replanning threshold
  double getReplanningThreshold() const;

  /// \brief
  void setStationaryCasePlanningTime(double _v);

  /// \brief
  double getStationaryCasePlanningTime() const;

  /// \brief
  void setRollingJointAngle(double _v);

  /// \brief
  double getRollingJointAngle() const;
  //----------------------------------------------------------------------------

  /// \brief
  void doMotionPlanning(double _currentTime);

  /// \brief
  bool isReplaningNeeded(double _time, double _tol);

  /// \brief
  bool isHitAnything(double _time);

  /// \brief
  void changeControlPhase(ControlPhase _cp);

  /// \brief
  void evalTorquesFallingPD();

  /// \brief Augmented PD controller
  ///        "A Mathematical Introduction to Robotic Manipulation", p.194
  void evalTorquesFallingAPD();

  /// \brief Stable Proportional-Derivative Controllers
  void evalTorquesFallingSPD();

  // TODO(JS): Not implemented yet.
  /// \brief We assume contact force is given (by sensor) and solve torque.
  void evalTorquesRollingVer1();

  void _prepareToSolveRollingTorqueVer1();
  void _solveRollingTorqueVer1();
  static double _objectFuncRollingTorqueVer1(unsigned n, const double *x,
                                           double *grad, void *my_func_data);
  static double _constraintFuncRollingTorqueVer1(unsigned n, const double *x,
                                               double *grad, void *data);

  static double _constraintFuncRollingTorqueVer1_JS(unsigned n, const double *x,
                                               double *grad, void *data);

  static double _constraintFuncRollingTorqueVer1_Q1Lower(
      unsigned n, const double *x, double *grad, void *data);
  static double _constraintFuncRollingTorqueVer1_Q1Upper(
      unsigned n, const double *x, double *grad, void *data);
  static double _constraintFuncRollingTorqueVer1_Q2Lower(
      unsigned n, const double *x, double *grad, void *data);
  static double _constraintFuncRollingTorqueVer1_Q21Upper(
      unsigned n, const double *x, double *grad, void *data);

  static void _constraintFuncRollingTorqueVer1_KinematicLimits(
          unsigned          m,
            double    *result,
          unsigned          n,
      const double         *x,
            double  *gradient, /* NULL if not needed */
              void *func_data);

  static void _constraintFuncRollingTorqueVer1_KinematicLimits4(
          unsigned          m,
            double    *result,
          unsigned          n,
      const double         *x,
            double  *gradient, /* NULL if not needed */
              void *func_data);

  Eigen::VectorXd computeTauPD(dart::dynamics::Skeleton* _skel);

  void _finishToSolveRollingTorqueVer1();

  static Eigen::VectorXd computeDDQ(dart::dynamics::Skeleton* _skel,
                                    const Eigen::VectorXd& _tau);

  // TODO(JS): Not implemented yet.
  /// \brief We solve contact force and torque together.
  void evalTorquesRollingVer2();

  /// \brief
  /// \param[in] _qi
  /// \param[in] _qf
  /// \param[in] _w0
  /// \param[in] _time
  /// \param[in] _jointPosLimit Bounds on joint angle. (rad)
  /// \param[in] _jointVelLimit Bounds on joint velocity. (rad/sec)
  /// \return
  Eigen::MatrixXd threelinkOrientPlan_VER1(
      const Eigen::Vector3d& _qi,
      const Eigen::Vector3d& _qf,
      const Eigen::Vector3d& _wo,
      double _totalTime,
      double _jointPosLimit = DART_PI * 0.5,
      double _jointVelLimit = 10.0);

  Eigen::MatrixXd threelinkOrientPlanBigTriangle(
      const Eigen::Vector3d& _qi,
      const Eigen::Vector3d& _qf,
      const Eigen::Vector3d& _wo,
      double _startingTime,
      double _totalTime,
      double _jointPosLimit = DART_PI * 0.5,
      double _jointVelLimit = 10.0);

  Eigen::MatrixXd threelinkOrientPlanSmallTriangle(
      const Eigen::Vector3d& _qi,
      const Eigen::Vector3d& _qf,
      const Eigen::Vector3d& _wo,
      double _startingTime,
      double _totalTime,
      double _jointPosLimit = DART_PI * 0.5,
      double _jointVelLimit = 10.0);

  /// \brief
  const Eigen::MatrixXd& getKp() const;

  /// \brief
  const Eigen::MatrixXd& getKd() const;

  // TODO(JS): Not implemented yet.
  /// \brief
  static Eigen::MatrixXd evalMomemtumJacobian0(
      dart::dynamics::Skeleton* _skel, double _q1, double _q2);

  // TODO(JS): Not implemented yet.
  /// \brief
  static Eigen::Vector3d evalMomemtumJacobian1(
      dart::dynamics::Skeleton* _skel, double _q1, double _q2);

  // TODO(JS): Not implemented yet.
  /// \brief
  static Eigen::Vector3d evalMomemtumJacobian2(
      dart::dynamics::Skeleton* _skel, double _q1, double _q2);

  /// \brief Get momentum Jacobian. The dimention is (6 x 5).
  dart::math::Jacobian evalMomentumJacobian(const Eigen::VectorXd& _q,
                                            const Eigen::Vector3d& _position);

  /// \brief Get Jacobian of momentum at the COM of root body node.
  dart::math::Jacobian evalMomentumJacobianAtRootCOM(const Eigen::VectorXd& _q);

  /// \brief Get generalized momentum.
  Eigen::Vector6d evalMomemtum(const Eigen::VectorXd& _q,
                               const Eigen::VectorXd& _dq,
                               const Eigen::Vector3d& _position);

  /// \brief Get generalized momentum at the COM of root body node.
  Eigen::Vector6d evalMomemtumAtRootCOM(const Eigen::VectorXd& _q,
                                        const Eigen::VectorXd& _dq);

  /// \brief Get angle displacement of root body.
  /// \param[in] _Ho Initial angular momentum.
  static double oneStepRotation(
      dart::dynamics::Skeleton* _skel,
      double _q1, double _q2, double _dq1, double _dq2,
      const Eigen::Vector3d& _Ho = Eigen::Vector3d::Zero());

  // TODO(JS): Not implemented yet.
  /// \brief Get angle displacement of root body during linear motion from _qi
  ///        to _qf.
  /// \param[in] _resolution Resolution of computation of path integral.
  static double lineRotation(
      dart::dynamics::Skeleton* _skel,
      const Eigen::Vector2d& _qi, const Eigen::Vector2d& _qf,
      int _resolution = 1000);

  /// \brief Get angle displacement during cyclic motion with joint upper limit
  ///        _ub.
  /// \param[in] _b Joint position upper/lower limit.
  static double cycleRotationBigTriangle(dart::dynamics::Skeleton* _skel,
                                         double _b = DART_PI * 0.5);

  /// \brief Get angle displacement during cyclic motion with joint upper limit
  ///        _ub.
  /// \param[in] _b Joint position upper/lower limit.
  static double cycleRotationSmallTriangle(dart::dynamics::Skeleton* _skel,
                                           double _b = DART_PI * 0.5);

  /// \brief Get scale factor of a path length that achieve _dq over a path
  ///        length that achieve the maximum joint angle displacement of root
  ///        body. Binary search is use.
  /// \param[in] _dq    Target angle displacement of root body.
  /// \param[in] _b     Joint position upper/lower limit. This is used to get
  ///                   the maximum cycle area of joint trajectory.
  /// \param[in] _bound Bound of binary searching space for the scale.
  static double solveCycleScaleBigTriangle(
      dart::dynamics::Skeleton* _skel,
      double _dq, double _b,
      const Eigen::Vector2d& _bound = Eigen::Vector2d(0.0, 1.0),
      double _tol = 1e-3);

  /// \brief Get scale factor of a path length that achieve _dq over a path
  ///        length that achieve the maximum joint angle displacement of root
  ///        body. Binary search is use.
  /// \param[in] _dq    Target angle displacement of root body.
  /// \param[in] _b     Joint position upper/lower limit. This is used to get
  ///                   the maximum cycle area of joint trajectory.
  /// \param[in] _bound Bound of binary searching space for the scale.
  static double solveCycleScaleSmallTriangle(
      dart::dynamics::Skeleton* _skel,
      double _dq, double _b,
      const Eigen::Vector2d& _bound = Eigen::Vector2d(0.0, 1.0),
      double _tol = 1e-3);

  // TODO(JS): Not implemented yet.
  /// \brief Compute Jacobian of momentum
  Eigen::Vector2d computeMomentumJacobian(const Eigen::Vector2d& _q);

  // TODO(JS): Not implemented yet.
  /// \brief Compute differential of root body's anlgle.
  double computeAngleDisplacement(const Eigen::Vector2d& _q,
                                  const Eigen::Vector2d& _dq);

  /// \brief Compute linearly spaced vectors
  static Eigen::VectorXd linspace(double _val1, double _val2, int _numPoints);

  /// \brief Compute difference
  static Eigen::VectorXd diff(const Eigen::VectorXd& _val);

protected:

  /// \brief
  ControlPhase mControlPhase;

  /// \brief
  Eigen::Vector3d mImpactPose;

  /// \brief
  Eigen::VectorXd mTorques;

  /// \brief
  Eigen::MatrixXd mKp;

  /// \brief
  Eigen::MatrixXd mKd;

  /// \brief
  LinearInterpolationMotion mPlannedMotion;

  /// \brief
  double mPreOffset;

public:
  /// \brief Spring stiffness for COM
  double mComK;

  /// \brief Damping coefficient for COM
  double mComB;

  /// \brief COM height at the impact moment.
  double mZ0;

protected:
  /// \brief
  nlopt_opt opt;

  /// \brief
  bool mIsOnFallingController;

  /// \brief
  bool mIsOnRollingController;

private:
  /// \brief
  std::string getControlPhaseString(ControlPhase _cp);

  /// \brief
  double getPredictedFallTime();

  /// \brief
  double getPredictedFallTimeCalcF(double _a, double _v, double _x, double _t);

  /// \brief
  double getPredictedFallTimeCalcDF(double _a, double _v, double _t);

  /// \brief
  virtual void bake();

  /// \brief
  std::vector<Eigen::VectorXd> mBakedStates;

  /// \brief
  void printState(const std::string& _fileName);

  /// \brief
  bool mUseSmallTrianglePath;

  /// \brief
  double mReplanningThreshold;

  /// \brief
  double mStationaryCasePlanningTime;

  /// \brief
  double mRollingJointAngle;
};

#endif  // APPS_INERTIABOT_AUTOMATICCONTROLLER_H_
