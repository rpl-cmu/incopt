/**
 * @file ConstraintUtils.h
 * @brief constraint utils
 * @author Paloma Sodhi
 */

// BEGIN CONSTRAINTS EDITS

#ifndef CONSTRAINT_UTILS_H
#define CONSTRAINT_UTILS_H

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/geometry/Pose2.h>

#include <Eigen/Dense>
#include <iostream>
#include <vector>

#include "p_poly_dist.h"

namespace gtsam {

namespace conutils {

// planar pushing datasets
Vector3 contactPush2dUnaryError3D(const Pose2& p, const Vector2& eeCenter__world,
                                  const Vector2& contactPoint__world,
                                  const Matrix& objPolyShape, const double& eeRadius);
Matrix33 contactPush2dUnaryErrorJacobian3D(const Pose2& p, const Vector2& eeCenter__world,
                                           const Vector2& contactPoint__world,
                                           const Matrix& objPolyShape, const double& eeRadius);
Vector1 contactPush2dUnaryError1D(const Pose2& p, const Vector2& eeCenter__world,
                                  const Matrix& objPolyShape, const double& eeRadius);
Matrix13 contactPush2dUnaryErrorJacobian1D(const Pose2& p, const Vector2& eeCenter__world,
                                           const Matrix& objPolyShape, const double& eeRadius);

// dataset1: sim arm 2d
Vector1 contactCircular2DError(const Pose2& p, double tx, double ty);
Matrix13 contactCircular2DErrorJacobian(const Pose2& p, double tx, double ty);

// dataset2, dataset3: sim/real corridor 2d
Vector1 corridorFloorPlan2DError(const Pose2& p, Vector3 lineParams);
Matrix13 corridorFloorPlan2DErrorJacobian(const Pose2& p, Vector3 lineParams, bool useAnalytic = true);

}  // namespace conutils
}  // namespace gtsam

#endif

// END CONSTRAINTS EDITS
