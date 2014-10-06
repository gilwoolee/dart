#include "ForceSpreadConstraint.h"
using namespace Eigen;

#include <math.h>

#include "dart/optimizer/Function.h"
#include "dart/optimizer/Solver.h"
#include "dart/math/Helpers.h"

namespace dart {
namespace optimizer {

//==============================================================================
ForceSpreadConstraint::ForceSpreadConstraint(
//    std::vector<Var *>& _var,
    int _firstForceIndex,
    int _secondForceIndex)
  : ScalarFunction(),
    mFirstForceIndex(_firstForceIndex),
    mSecondForceIndex(_secondForceIndex)
{
//  mNumRows = 1;

//  mWeight = VectorXd::Ones(mNumRows);
//  mConstTerm = VectorXd::Zero(mNumRows);
//  mCompletion = VectorXd::Zero(mNumRows);
}

//==============================================================================
double ForceSpreadConstraint::eval(Eigen::Map<const VectorXd>& _x)
{
  size_t firstForceIdx0 = mFirstForceIndex * 3 + 0;
  size_t firstForceIdx1 = mFirstForceIndex * 3 + 1;
  size_t firstForceIdx2 = mFirstForceIndex * 3 + 2;

  size_t secondForceIdx0 = mSecondForceIndex * 3 + 0;
  size_t secondForceIdx1 = mSecondForceIndex * 3 + 1;
  size_t secondForceIdx2 = mSecondForceIndex * 3 + 2;

  double firstForceMag = _x[firstForceIdx0] * _x[firstForceIdx0]
                         + _x[firstForceIdx1] * _x[firstForceIdx1]
                         + _x[firstForceIdx2] * _x[firstForceIdx2];

  double secondForceMag = _x[secondForceIdx0] * _x[secondForceIdx0]
                          + _x[secondForceIdx1] * _x[secondForceIdx1]
                          + _x[secondForceIdx2] * _x[secondForceIdx2];

  double w = 0.1;
  double ret = w * firstForceMag-secondForceMag;

  return ret;
}

//==============================================================================
void ForceSpreadConstraint::evalGradient(Eigen::Map<const VectorXd>& _x,
                                         Eigen::Map<VectorXd> _grad)
{
  size_t firstForceIdx0 = mFirstForceIndex * 3 + 0;
  size_t firstForceIdx1 = mFirstForceIndex * 3 + 1;
  size_t firstForceIdx2 = mFirstForceIndex * 3 + 2;

  size_t secondForceIdx0 = mSecondForceIndex * 3 + 0;
  size_t secondForceIdx1 = mSecondForceIndex * 3 + 1;
  size_t secondForceIdx2 = mSecondForceIndex * 3 + 2;

  double dP = eval(_x);

  double w = 0.1;

  for (size_t i = 0; i < _x.size(); ++i)
  {
    double J = 0.0;

    if (firstForceIdx0 == i)
    {
      J = _x[firstForceIdx0];
      J *= w;
      _grad[i] += dP * J;
    }
    else if (firstForceIdx1 == i)
    {
      J = _x[firstForceIdx1];
      J *= w;
      _grad[i] += dP * J;
    }
    else if (firstForceIdx2 == i)
    {
      J = _x[firstForceIdx2];
      J *= w;
      _grad[i] += dP * J;
    }
    else if (secondForceIdx0 == i)
    {
      J = _x[secondForceIdx0];
      J *= w;
      _grad[i] += dP * J;
    }
    else if (secondForceIdx1 == i)
    {
      J = _x[secondForceIdx1];
      J *= w;
      _grad[i] += dP * J;
    }
    else if (secondForceIdx2 == i)
    {
      J = _x[secondForceIdx2];
      J *= w;
      _grad[i] += dP * J;
    }
  }
}

}  // namespace optimizer
}  // namespace dart
