/**
 * @file GaussianProcessInterpolatorPose3.h
 * @brief Base and utils for Gaussian Process Interpolated measurement factor, works only in SE(3)
 * @author Jing Dong
 */

#pragma once

#include <cpp/thirdparty/gpmp2/gp/GaussianProcessInterpolatorLie.h>

#include <gtsam/geometry/Pose3.h>

namespace incopt {

typedef GaussianProcessInterpolatorLie<gtsam::Pose3> GaussianProcessInterpolatorPose3;

} // \ namespace incopt

