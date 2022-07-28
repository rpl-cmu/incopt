/**
 *  @file   ObstaclePlanarSDFFactorGPPointRobot.h
 *  @brief  Obstacle avoidance cost factor, for point robot, using signed distance field,
 *          and GP interpolation
 *  @author Mustafa Mukadam
 *  @date   July 20, 2016
 **/

#pragma once

#include <cpp/thirdparty/gpmp2/kinematics/PointRobotModel.h>
#include <cpp/thirdparty/gpmp2/gp/GaussianProcessInterpolatorLinear.h>
#include <cpp/thirdparty/gpmp2/obstacle/ObstaclePlanarSDFFactorGP.h>

namespace incopt {

// template uses PointRobotModel as robot type
typedef ObstaclePlanarSDFFactorGP<PointRobotModel, GaussianProcessInterpolatorLinear>
    ObstaclePlanarSDFFactorGPPointRobot;

}
