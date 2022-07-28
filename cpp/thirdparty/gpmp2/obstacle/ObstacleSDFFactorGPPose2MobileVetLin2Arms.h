/**
 *  @file   ObstacleSDFFactorGPPose2MobileVetLin2Arms.h
 *  @brief  Obstacle avoidance interpolated GP factor for 2 X 3D arms
 *          with SE(2) base and linear actuator
 *  @author Mustafa Mukadam
 *  @date   Sep 4, 2017
 **/

#pragma once

#include <cpp/thirdparty/gpmp2/kinematics/Pose2MobileVetLin2ArmsModel.h>
#include <cpp/thirdparty/gpmp2/gp/GaussianProcessInterpolatorPose2Vector.h>
#include <cpp/thirdparty/gpmp2/obstacle/ObstacleSDFFactorGP.h>

namespace incopt {

// template uses Pose2MobileVetLin2ArmsModel as robot type
typedef ObstacleSDFFactorGP<Pose2MobileVetLin2ArmsModel, GaussianProcessInterpolatorPose2Vector>
    ObstacleSDFFactorGPPose2MobileVetLin2Arms;

}
