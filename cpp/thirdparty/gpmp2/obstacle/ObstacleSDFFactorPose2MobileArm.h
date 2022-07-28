/**
 *  @file   ObstacleSDFFactorPose2MobileArm.h
 *  @brief  Obstacle avoidance cost factor for 3D arm with SE(2) base
 *  @author Mustafa Mukadam
 *  @date   Nov 2, 2016
 **/

#pragma once

#include <cpp/thirdparty/gpmp2/kinematics/Pose2MobileArmModel.h>
#include <cpp/thirdparty/gpmp2/obstacle/ObstacleSDFFactor.h>

namespace incopt {

// template use Pose2MobileArmModel as robot type
typedef ObstacleSDFFactor<Pose2MobileArmModel> ObstacleSDFFactorPose2MobileArm;

}
