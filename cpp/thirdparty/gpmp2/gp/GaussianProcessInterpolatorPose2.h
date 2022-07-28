/**
 * @file GaussianProcessInterpolatorPose2.h
 * @brief Base and utils for Gaussian Process Interpolated measurement factor, works only in SE(2)
 * @author Jing Dong
 */

#pragma once

#include <cpp/thirdparty/gpmp2/gp/GaussianProcessInterpolatorLie.h>

#include <gtsam/geometry/Pose2.h>

namespace incopt {

typedef GaussianProcessInterpolatorLie<gtsam::Pose2> GaussianProcessInterpolatorPose2;

} // \ namespace incopt

