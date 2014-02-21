/*
 * Copyright (c) 2014, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Karen Liu <karenliu@cc.gatech.edu>,
 *            Jeongseok Lee <jslee02@gmail.com>
 *
 * Geoorgia Tech Graphics Lab and Humanoid Robotics Lab
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

#include "dart/constraint_test/Constraint.h"

#include <cstring>
#include <iostream>

namespace dart {
namespace constraint {

#define dPAD(a) (((a) > 1) ? ((((a)-1)|3)+1) : (a))

//==============================================================================
ODELcp::ODELcp(int _n)
{
  nSkip = dPAD(_n);

  A  = new double[_n * nSkip];
  b  = new double[_n];
  w  = new double[_n];
  x  = new double[_n];
  lb = new double[_n];
  ub = new double[_n];
  frictionIndex = new int[_n];
  dim = _n;

//  std::memset(A, 0, _n * nSkip * sizeof(double));
  std::memset(frictionIndex, -1, _n * sizeof(int));
}

//==============================================================================
ODELcp::~ODELcp()
{
  delete[] A;
  delete[] b;
  delete[] w;
  delete[] x;
  delete[] lb;
  delete[] ub;
  delete[] frictionIndex;
}

//==============================================================================
void ODELcp::print()
{
  std::cout << "A: " << std::endl;
  for (int i = 0; i < dim; ++i)
  {
    for (int j = 0; j < nSkip; ++j)
    {
      std::cout << A[i * nSkip + j] << " ";
    }
    std::cout << std::endl;
  }
  std::cout << std::endl;

  std::cout << "b: " << std::endl;
  for (int i = 0; i < dim; ++i)
  {
    std::cout << b[i] << " ";
  }
  std::cout << std::endl;

  std::cout << "w: " << std::endl;
  for (int i = 0; i < dim; ++i)
  {
    std::cout << w[i] << " ";
  }
  std::cout << std::endl;

  std::cout << "x: " << std::endl;
  for (int i = 0; i < dim; ++i)
  {
    std::cout << x[i] << " ";
  }
  std::cout << std::endl;

  std::cout << "lb: " << std::endl;
  for (int i = 0; i < dim; ++i)
  {
    std::cout << lb[i] << " ";
  }
  std::cout << std::endl;

  std::cout << "ub: " << std::endl;
  for (int i = 0; i < dim; ++i)
  {
    std::cout << ub[i] << " ";
  }
  std::cout << std::endl;

  std::cout << "frictionIndex: " << std::endl;
  for (int i = 0; i < dim; ++i)
  {
    std::cout << frictionIndex[i] << " ";
  }
  std::cout << std::endl;
}

//==============================================================================
ConstraintTEST::ConstraintTEST(ConstraintType _type)
  : mDim(0)
{
}

//==============================================================================
ConstraintTEST::~ConstraintTEST()
{
}

//==============================================================================
int ConstraintTEST::getDimension() const
{
  return mDim;
}

}  // namespace constraint
}  // namespace dart