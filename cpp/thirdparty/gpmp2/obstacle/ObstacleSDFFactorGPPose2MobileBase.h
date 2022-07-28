/**
 *  @file   ObstacleSDFFactorGPPose2MobileBase.h
 *  @brief  Obstacle avoidance interpolated GP factor for 3D SE(2) base
 *  @author Mustafa Mukadam
 *  @date   Jan 23, 2018
 **/

#pragma once

#include <cpp/thirdparty/gpmp2/kinematics/Pose2MobileBaseModel.h>
#include <cpp/thirdparty/gpmp2/gp/GaussianProcessInterpolatorPose2.h>
#include <cpp/thirdparty/gpmp2/obstacle/ObstacleSDFFactorGP.h>

namespace incopt {

// template uses Pose2MobileBaseModel as robot type
typedef ObstacleSDFFactorGP<Pose2MobileBaseModel, GaussianProcessInterpolatorPose2>
    ObstacleSDFFactorGPPose2MobileBase;

}
