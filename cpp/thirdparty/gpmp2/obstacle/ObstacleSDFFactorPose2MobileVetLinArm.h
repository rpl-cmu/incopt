/**
 *  @file   ObstacleSDFFactorPose2MobileVetLinArm.h
 *  @brief  Obstacle avoidance cost factor for 3D arm
 *          with SE(2) base and linear actuator
 *  @author Mustafa Mukadam
 *  @date   Sep 4, 2017
 **/

#pragma once

#include <cpp/thirdparty/gpmp2/kinematics/Pose2MobileVetLinArmModel.h>
#include <cpp/thirdparty/gpmp2/obstacle/ObstacleSDFFactor.h>

namespace incopt {

// template use Pose2MobileVetLinArmModel as robot type
typedef ObstacleSDFFactor<Pose2MobileVetLinArmModel> ObstacleSDFFactorPose2MobileVetLinArm;

}
