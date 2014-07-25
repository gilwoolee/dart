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

#ifndef APPS_INERTIABOT_MYWINDOW_H_
#define APPS_INERTIABOT_MYWINDOW_H_

#include <list>

#include <Eigen/Core>
#include <tinyxml2.h>

#include "dart/gui/SimWindow.h"
#include "apps/inertiaBot/Controller.h"
#include "apps/inertiaBot/CompositeController.h"

/// \brief
class MyWindow : public dart::gui::SimWindow
{
public:
  /// \brief
  MyWindow();

  /// \brief
  virtual ~MyWindow();

  /// \brief
  void setController(CompositeController* _controller);

  /// \brief
  virtual void timeStepping();

  /// \brief
  virtual void drawSkels();

  /// \brief
  virtual void keyboard(unsigned char _key, int _x, int _y);

  /// \brief
  void printResult();

protected:
  /// \brief
//  virtual void bake();

private:
  /// \brief
  void loadXmlScript(const char* _xmlFileName);

  /// \brief
  void loadXmlCommand(tinyxml2::XMLElement* _rootElem,
                      const char* _strCmdName,
                      std::list<double> (&_vecCmd)[4]);

  /// \brief
  std::list<double> parseTextVector(const char* _txtVector);

  /// \brief
  Eigen::VectorXd computeDamping();

  /// \brief
  std::list<double> mCommands[4];

  /// \brief
  CompositeController* mController;

  /// \brief
  bool mVisibleCollisionShape;

  /// \brief
  bool mVisibleInertiaEllipsoid;

  /// \brief
  bool mVisibleCOM;

  /// \brief
  bool mVisibleAngularMomentum;

  /// \brief
  bool mVisibleLinearMomentum;

  /// \brief
  bool mSave;
};

#endif  // APPS_INERTIABOT_MYWINDOW_H_
