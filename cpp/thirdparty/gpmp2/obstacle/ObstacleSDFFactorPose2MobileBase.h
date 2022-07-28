/**
 *  @file   ObstacleSDFFactorPose2MobileBase.h
 *  @brief  Obstacle avoidance cost factor for 3D SE(2) base
 *  @author Mustafa Mukadam
 *  @date   Jan 23, 2018
 **/

#pragma once

#include <cpp/thirdparty/gpmp2/kinematics/Pose2MobileBaseModel.h>
#include <cpp/thirdparty/gpmp2/obstacle/ObstacleSDFFactor.h>

namespace incopt {

// template use Pose2MobileBaseModel as robot type
typedef ObstacleSDFFactor<Pose2MobileBaseModel> ObstacleSDFFactorPose2MobileBase;

}
