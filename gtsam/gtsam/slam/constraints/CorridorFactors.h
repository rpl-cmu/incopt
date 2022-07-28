/**
 * @file CorridorFactors.h
 * @brief corridor inequality constraint factors
 * @author Paloma Sodhi
 */

// BEGIN CONSTRAINTS EDITS

#ifndef CORRIDOR_FACTORS_H_
#define CORRIDOR_FACTORS_H_

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include "ConstraintUtils.h"

namespace gtsam {

class CorridorFloorPlan2dFactor : public NoiseModelFactor1<Pose2> {
 private:
  Vector3 lineParams_;

 public:
  /** Constructor
   * @param poseKey   associated pose variable key
   * @param model     noise model for corridor constraint, 1-dim
   * @param lineParams coeffs of 2D line eqn representing corridoor boundary (active at poseKey)
  */
  CorridorFloorPlan2dFactor(Key poseKey, const Vector3 lineParams, const SharedNoiseModel& model = nullptr) : NoiseModelFactor1(model, poseKey), lineParams_(lineParams) {
    isConstraintFactor_ = true;
    constraintFactorType_ = 2;  // inequality
  }

  /** error function
   * @param      curr pose in Pose2
   * @param H    optional Jacobian matrix, uses boost optional and has default null pointer
  */
  Vector evaluateError(const Pose2& p, boost::optional<Matrix&> H = boost::none) const {
    // note: use boost optional like a pointer
    // only calculate Jacobian matrix when non-null pointer exists
    
    if (H) *H = conutils::corridorFloorPlan2DErrorJacobian(p, lineParams_, false);

    // return error vector
    Vector1 errVec = conutils::corridorFloorPlan2DError(p, lineParams_);

    return (Vector1() << errVec(0)).finished();
  }

  /**
   * constraint params
   */
  Vector constraintParams() const {
    return lineParams_;
  }

  /**
   * constraint error
   */
  Vector constraintError(const Value& val) const {
    Pose2 p = val.cast<Pose2>();
    Vector1 errVec = conutils::corridorFloorPlan2DError(p, lineParams_);
    return errVec;
  }

  /**
   * constraint jacobian
   */
  Matrix constraintJacobian(const Value& val) const {
    Pose2 p = val.cast<Pose2>();
    Matrix G = conutils::corridorFloorPlan2DErrorJacobian(p, lineParams_, false);

    return G;
  }
};
}  // namespace gtsam

#endif

// END CONSTRAINTS EDITS
