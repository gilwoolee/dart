#ifndef FORCE_SPREAD_CONSTRAINT_H
#define FORCE_SPREAD_CONSTRAINT_H

#include <vector>

#include "dart/optimizer/Function.h"

namespace dart {
namespace optimizer {

//class Var;

class ForceSpreadConstraint : public ScalarFunction
{
public:
  /// Constructor
  ForceSpreadConstraint(/*std::vector<Var*>& _var,*/
                        int _firstForceIndex,
                        int _secondForceIndex);

  /// Destructor
  virtual ~ForceSpreadConstraint() {}

  ///
  //virtual void fillJac(VVD, int) {}

  ///
  //virtual void fillJac(VVD, VVB, int);

  ///
  //virtual void fillObjGrad(std::vector<double>&);

  // Documentation inherited
  virtual double eval(Eigen::Map<const Eigen::VectorXd>& _x);

  /// \brief Evaluate and return the objective function at the point x
  virtual void evalGradient(Eigen::Map<const Eigen::VectorXd>& _x,
                            Eigen::Map<Eigen::VectorXd> _grad);

public:
  ///
  int mFirstForceIndex;

  ///
  int mSecondForceIndex;
};

}  // namespace optimizer
}  // namespace dart

#endif  // FORCE_SPREAD_CONSTRAINT_H

