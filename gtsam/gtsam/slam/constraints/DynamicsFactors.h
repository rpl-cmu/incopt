/**
 * @file DynamicsFactors.h
 * @brief dynamics factors
 * @author Paloma Sodhi
 */

// BEGIN CONSTRAINTS EDITS

#ifndef DYNAMICS_FACTORS_H_
#define DYNAMICS_FACTORS_H_

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

namespace gtsam {

// todo: IPDynamicsEqualityFactor

class IPDynamicsFactor : public NoiseModelFactor3<Vector2, Vector2, Vector1> {
 private:
  Matrix A_, B_, I_;

 public:
  /** Constructor
  */
  IPDynamicsFactor(Key xk1, Key xk, Key uk, const Matrix I, const Matrix A, const Matrix B,
                    SharedNoiseModel model)
      : NoiseModelFactor3(model, xk1, xk, uk), A_(A), B_(B), I_(I) {}

  /** error function
   * errVec = -I*xk1 + A*xk + B*uk
  */
  Vector evaluateError(const Vector2& xk1, const Vector2& xk, const Vector1& uk,
                       boost::optional<Matrix&> H1 = boost::none,
                       boost::optional<Matrix&> H2 = boost::none,
                       boost::optional<Matrix&> H3 = boost::none) const {
    
    // note: use boost optional like a pointer
    // only calculate Jacobian matrix when non-null pointer exists
    
    if (H1) *H1 = (Matrix22() << I_(0, 0), I_(0, 1), I_(1, 0), I_(1, 1)).finished();
    if (H2) *H2 = (Matrix22() << A_(0, 0), A_(0, 1), A_(1, 0), A_(1, 1)).finished();
    if (H3) *H3 = (Matrix21() << B_(0, 0), B_(1, 0)).finished();

    // return error vector
    // Vector2 errVec = I_ * xk1 + A_ * xk + B_ * uk;
    
    return (Vector2() << I_ * xk1 + A_ * xk + B_ * uk).finished();
  }
};

}  // namespace gtsam

#endif

// END CONSTRAINTS EDITS
