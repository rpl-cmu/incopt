/**
 * @file ContactFactors.h
 * @brief contact equality constraint factors
 * @author Paloma Sodhi
 */

// BEGIN CONSTRAINTS EDITS

#ifndef CONTACT_FACTORS_H_
#define CONTACT_FACTORS_H_

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include "ConstraintUtils.h"

namespace gtsam {

class ContactMeasurement {
 public:
  Vector2 contactPoint__world_;
  Vector2 eeCenter__world_;
  Matrix objPolyShape_;
  double eeRadius_;

  ContactMeasurement(const Vector2 contactPoint__world, const Vector2 eeCenter__world,
                     const Matrix objPolyShape, const double eeRadius)
      : contactPoint__world_(contactPoint__world), eeCenter__world_(eeCenter__world), objPolyShape_(objPolyShape), eeRadius_(eeRadius) {}

  void write(std::ostream& out) const {
    out << "("
        << "contact_point" << contactPoint__world_ << " "
        << "ee_center" << eeCenter__world_ << " "
        << "ee_radius"
        << eeRadius_ << ")";
  }
};

// factor for planar pushing dataset generated using physics simulator
class ContactPush2dUnaryFactor : public NoiseModelFactor1<Pose2> {
 private:
  ContactMeasurement contactMeas_;

 public:
  /**
   * constructor
   */
  ContactPush2dUnaryFactor(Key poseKey, ContactMeasurement contactMeas, const SharedNoiseModel& model = nullptr)
      : NoiseModelFactor1(model, poseKey), contactMeas_(contactMeas) {
    isConstraintFactor_ = true;
    constraintFactorType_ = 1;  // equality
  }

  /**
   * evaluateError
   */
  Vector evaluateError(const Pose2& p, boost::optional<Matrix&> H = boost::none) const {
    // Vector errVec = conutils::contactPush2dUnaryError1D(p, contactMeas_.eeCenter__world_, contactMeas_.objPolyShape_, contactMeas_.eeRadius_);
    // if (H) *H = conutils::contactPush2dUnaryErrorJacobian1D(p, contactMeas_.eeCenter__world_, contactMeas_.objPolyShape_, contactMeas_.eeRadius_);

    Vector errVec = conutils::contactPush2dUnaryError3D(p, contactMeas_.eeCenter__world_, contactMeas_.contactPoint__world_, contactMeas_.objPolyShape_, contactMeas_.eeRadius_);
    if (H) *H = conutils::contactPush2dUnaryErrorJacobian3D(p, contactMeas_.eeCenter__world_, contactMeas_.contactPoint__world_, contactMeas_.objPolyShape_, contactMeas_.eeRadius_);

    return errVec;
  }

  /**
   * constraint error
   */
  Vector constraintError(const Value& val) const {
    Pose2 p = val.cast<Pose2>();
    Vector1 errVec = conutils::contactPush2dUnaryError1D(p, contactMeas_.eeCenter__world_, contactMeas_.objPolyShape_, contactMeas_.eeRadius_);
    return errVec;
  }
  /**
   * constraint jacobian
   */
  Matrix constraintJacobian(const Value& val) const {
    Pose2 p = val.cast<Pose2>();
    Matrix G = conutils::contactPush2dUnaryErrorJacobian1D(p, contactMeas_.eeCenter__world_, contactMeas_.objPolyShape_, contactMeas_.eeRadius_);
    return G;
  }

  Vector2 getPointPolyProj(const Pose2& p) {
    double dist;
    Vector2 pointPoly__obj;
    Point2 eeCenter__obj = p.transformTo(Point2(contactMeas_.eeCenter__world_));
    p_poly_dist(eeCenter__obj, contactMeas_.objPolyShape_, Matrix(), Matrix(),
                &dist, NULL, NULL, NULL, NULL, &pointPoly__obj, NULL, NULL, 0);

    Point2 pointPoly__world = p.transformFrom(Point2(pointPoly__obj));

    return pointPoly__world;
  }
};

// factor for sim arm2d dataset generated using matlab
class ContactCircular2dFactor : public NoiseModelFactor1<Pose2> {
 private:
  double tx_, ty_;  // tx, ty

 public:
  /** Constructor
   * @param poseKey   associated pose variable key
   * @param model     noise model for contact sensor, 1-dim
   * @param toolPos   curr tool/endeff position in Vector2
  */
  ContactCircular2dFactor(Key poseKey, const Vector2 toolPos, const SharedNoiseModel& model = nullptr)
      : NoiseModelFactor1(model, poseKey), tx_(toolPos.x()), ty_(toolPos.y()) {
    isConstraintFactor_ = true;
    constraintFactorType_ = 1;  // equality
  }

  /** error function
   * @param      curr pose in Pose2
   * @param H    optional Jacobian matrix, uses boost optional and has default null pointer
  */
  Vector evaluateError(const Pose2& p, boost::optional<Matrix&> H = boost::none) const {
    // note: use boost optional like a pointer
    // only calculate Jacobian matrix when non-null pointer exists
    if (H) *H = conutils::contactCircular2DErrorJacobian(p, tx_, ty_);

    // return error vector
    Vector1 errVec = conutils::contactCircular2DError(p, tx_, ty_);

    return (Vector1() << errVec(0)).finished();
  }

  /**
   * constraint params
   */
  Vector constraintParams() const {
    return (Vector2() << tx_, ty_).finished();
  }

  /**
   * constraint error
   */
  Vector constraintError(const Value& val) const {
    Pose2 p = val.cast<Pose2>();
    Vector1 errVec = conutils::contactCircular2DError(p, tx_, ty_);
    return errVec;
  }
  /**
   * constraint jacobian
   */
  Matrix constraintJacobian(const Value& val) const {
    Pose2 p = val.cast<Pose2>();
    Matrix G = conutils::contactCircular2DErrorJacobian(p, tx_, ty_);
    return G;
  }
};

}  // namespace gtsam

#endif

// END CONSTRAINTS EDITS
