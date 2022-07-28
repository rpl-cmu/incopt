/**
 * @file GaussianProcessInterpolatorPose2Vector.h
 * @brief Base and utils for Gaussian Process Interpolated measurement factor, works only in SE(2) + Vector space
 * @author Jing Dong
 */

#pragma once

#include <cpp/thirdparty/gpmp2/gp/GaussianProcessInterpolatorLie.h>
#include <cpp/thirdparty/gpmp2/geometry/Pose2Vector.h>

namespace incopt {

typedef GaussianProcessInterpolatorLie<Pose2Vector> GaussianProcessInterpolatorPose2Vector;

} // \ namespace incopt

