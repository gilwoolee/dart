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

#ifndef APPS_HANDTILTINGANGLE_MYWINDOW_H_
#define APPS_HANDTILTINGANGLE_MYWINDOW_H_

#include "dart/gui/SimWindow.h"

/// class MyWindow
class MyWindow : public dart::gui::SimWindow
{
public:
    MyWindow(): SimWindow()
    {
        mForce = Eigen::Vector3d::Zero();
        mRollNum = 0;
    }
    virtual ~MyWindow() {}
    
    virtual void timeStepping();
    virtual void drawSkels();
    virtual void keyboard(unsigned char key, int x, int y);
    void setInitVel(Eigen::Vector3d _vel);
    void evalN();
    void evalGeometry();
    void evalInertia();
    void evalAngles();
    void setGroundAngle(double _angle, Eigen::Vector3d _axis);
    double getAngle(int _index);
    int evalContactEdge();
private:
    Eigen::Vector3d mForce;
    std::vector<double> mAngles;
    int mN;
    Eigen::Vector3d mInitVel;
    std::vector<double> mStartVels; // the velocity magnitude at the beginning of a rolling cycle
    std::vector<double> mEndVels; // the velocity magnitude at the end of a rolling cycle
    Eigen::Vector3d mCOM; // the COM with respect to local coordinate
    std::vector<Eigen::Vector3d> mEdges; // the edges of the polygon (pivoting edges) in the local coordinate, use the represent point in the 2D plane
    std::vector<double> mRs; // distance between the pivoting edge and COM
    std::vector<double> mAlphas; // the angle between COM and pivoting edge with the previous face
    std::vector<double> mPhis; // the angle between COM and pivoting edge with the next face
    std::vector<Eigen::Matrix3d> mIs; // the inertia matrices with respect to each pivoting edge
    int mRollNum;
};

#endif  // APPS_HANDTILTINGANGLE_MYWINDOW_H_
