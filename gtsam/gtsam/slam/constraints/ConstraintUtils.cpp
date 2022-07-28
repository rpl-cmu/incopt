// BEGIN CONSTRAINTS EDITS

#include "ConstraintUtils.h"

#include <Eigen/Dense>
#include <iostream>
#include <vector>

namespace gtsam {
namespace conutils {

Vector3 contactPush2dUnaryError3D(const Pose2& p, const Vector2& eeCenter__world,
                                  const Vector2& contactPoint__world, const Matrix& objPolyShape,
                                  const double& eeRadius) {
  Point2 eeCenter__obj = p.transformTo(Point2(eeCenter__world));
  Point2 contactPoint__obj = p.transformTo(Point2(contactPoint__world));

  double dist;
  Vector2 pointPoly__obj;
  p_poly_dist(eeCenter__obj, objPolyShape, Matrix(), Matrix(),
              &dist, NULL, NULL, NULL, NULL, &pointPoly__obj, NULL, NULL, 0);

  Vector3 errVec;
  errVec[0] = dist - eeRadius;
  errVec[1] = contactPoint__obj.x() - pointPoly__obj[0];
  errVec[2] = contactPoint__obj.y() - pointPoly__obj[1];

  // cout << "[conutils::contactPush2dUnaryError] errVec: " << errVec[0] << ", " << errVec[1] << ", " << errVec[2] << endl;

  return errVec;
}

Matrix33 contactPush2dUnaryErrorJacobian3D(const Pose2& p, const Vector2& eeCenter__world,
                                           const Vector2& contactPoint__world, const Matrix& objPolyShape,
                                           const double& eeRadius) {
  Matrix G(3, 3);

  bool useAnalytic = false;
  if (useAnalytic) {
    // todo
  } else {
    G = numericalDerivative11<Vector3, Pose2>(boost::bind(&contactPush2dUnaryError3D, _1,
                                                          eeCenter__world, contactPoint__world, 
                                                          objPolyShape, eeRadius), p);
  }

  return G;
}

Vector1 contactPush2dUnaryError1D(const Pose2& p, const Vector2& eeCenter__world,
                                  const Matrix& objPolyShape, const double& eeRadius) {
  double dist;
  Vector2 pointPoly__obj;
  Point2 eeCenter__obj = p.transformTo(Point2(eeCenter__world));
  p_poly_dist(eeCenter__obj, objPolyShape, Matrix(), Matrix(),
              &dist, NULL, NULL, NULL, NULL, &pointPoly__obj, NULL, NULL, 0);

  Vector1 errVec;
  errVec[0] = dist - eeRadius;
  // cout << "[conutils::contactPush2dUnaryError] errVec[0]: " << errVec[0] << endl;

  return errVec;
}

Matrix13 contactPush2dUnaryErrorJacobian1D(const Pose2& p, const Vector2& eeCenter__world,
                                           const Matrix& objPolyShape, const double& eeRadius) {
  Matrix G(1, 3);

  bool useAnalytic = false;
  if (useAnalytic) {
    // todo
  } else {
    G = numericalDerivative11<Vector1, Pose2>(boost::bind(&contactPush2dUnaryError1D, _1,
                                                          eeCenter__world, objPolyShape, eeRadius),
                                              p);
  }

  return G;
}

Vector1 contactCircular2DError(const Pose2& p, double tx, double ty) {
  double tool_r = 0.05;
  double object_r = 0.05;

  double error = pow((p.x() - tx), 2) + pow((p.y() - ty), 2) - pow((tool_r + object_r), 2);

  Vector1 error_vec;
  error_vec << 1 * error;

  return error_vec;
}

Matrix13 contactCircular2DErrorJacobian(const Pose2& p, double tx, double ty) {
  Matrix13 G;

  double tool_r = 0.05;
  double object_r = 0.05;

  // double den = 1/(2* sqrt(abs(pow((p(0) - t(0)), 2) + pow((p(1) - t(1)), 2) - pow((tool_r + object_r), 2))));

  double G11 = 1 * 2 * (p.x() - tx);
  double G12 = 1 * 2 * (p.y() - ty);
  double G13 = 0;
  G << G11, G12, G13;

  return G;
}

Vector1 corridorFloorPlan2DError(const Pose2& p, Vector3 lineParams) {
  double error = lineParams(0) * p.x() + lineParams(1) * p.y() + lineParams(2);  // <=0
  Eigen::VectorXd errorVec(1);
  errorVec << error;

  return errorVec;
}

Matrix13 corridorFloorPlan2DErrorJacobian(const Pose2& p, Vector3 lineParams, bool useAnalytic) {
  Matrix G(1, 3);

  // todo: debug analytic jacobian (incorrect values for non-zero theta vals)
  if (useAnalytic) {
    double G11 = lineParams(0);
    double G12 = lineParams(1);
    double G13 = 0;
    G << G11, G12, G13;
  } else {
    G = numericalDerivative11<Vector1, Pose2>(boost::bind(&corridorFloorPlan2DError, _1, lineParams), p);
  }

  return G;
}

}  // namespace conutils
}  // namespace gtsam

// END CONSTRAINTS EDITS
