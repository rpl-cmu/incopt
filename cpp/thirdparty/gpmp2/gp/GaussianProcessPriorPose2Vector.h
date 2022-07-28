/**
 *  @file  GaussianProcessPriorPose2Vector.h
 *  @brief Pose2Vector GP prior
 *  @author Jing Dong
 **/

#pragma once

#include <cpp/thirdparty/gpmp2/gp/GaussianProcessPriorLie.h>
#include <cpp/thirdparty/gpmp2/geometry/Pose2Vector.h>


namespace incopt {

typedef GaussianProcessPriorLie<Pose2Vector> GaussianProcessPriorPose2Vector;

} // \ namespace incopt

