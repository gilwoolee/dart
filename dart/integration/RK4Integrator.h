/*
 * Copyright (c) 2011-2014, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Kristin Siu <kasiu@gatech.edu>,
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

#ifndef DART_INTEGRATION_RK4INTEGRATOR_H_
#define DART_INTEGRATION_RK4INTEGRATOR_H_

#include "dart/integration/Integrator.h"

namespace dart {
namespace integration {

/// \brief
template<typename State, typename Deriv>
class RK4Integrator : public Integrator<State, Deriv>
{
public:
  // Documentation inherited.
  static void integrate(IntegrableSystem<State, Deriv>* _system, double _dt)
  {
    // TODO(kasiu): Slow. Needs to be optimized.

    // calculates the four weighted deltas
    Deriv deriv = _system->evalDeriv();
    State x = _system->getState();
    Deriv k1 = deriv * _dt;

    _system->setState(x + (k1 * 0.5));
    deriv = _system->evalDeriv();
    Deriv k2 = deriv * _dt;

    _system->setState(x + (k2 * 0.5));
    deriv = _system->evalDeriv();
    Deriv k3 = deriv * _dt;

    _system->setState(x + k3);
    deriv = _system->evalDeriv();
    Deriv k4 = deriv * _dt;

    _system->setState(x + ((1.0/6.0) * (k1 + (2.0 * k2) + (2.0 * k3) + k4)));
  }
};

}  // namespace integration
}  // namespace dart

#endif  // DART_INTEGRATION_RK4INTEGRATOR_H_
