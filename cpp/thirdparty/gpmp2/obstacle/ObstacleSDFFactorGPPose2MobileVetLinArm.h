/**
 *  @file   ObstacleSDFFactorGPPose2MobileVetLinArm.h
 *  @brief  Obstacle avoidance interpolated GP factor for 3D arm
 *          with SE(2) base and linear actuator
 *  @author Mustafa Mukadam
 *  @date   Sep 4, 2017
 **/

#pragma once

#include <cpp/thirdparty/gpmp2/kinematics/Pose2MobileVetLinArmModel.h>
#include <cpp/thirdparty/gpmp2/gp/GaussianProcessInterpolatorPose2Vector.h>
#include <cpp/thirdparty/gpmp2/obstacle/ObstacleSDFFactorGP.h>

namespace incopt {

// template uses Pose2MobileVetLinArmModel as robot type
typedef ObstacleSDFFactorGP<Pose2MobileVetLinArmModel, GaussianProcessInterpolatorPose2Vector>
    ObstacleSDFFactorGPPose2MobileVetLinArm;

}
