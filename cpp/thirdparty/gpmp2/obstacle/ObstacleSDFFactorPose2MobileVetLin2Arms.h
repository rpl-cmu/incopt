/**
 *  @file   ObstacleSDFFactorPose2MobileVetLin2Arms.h
 *  @brief  Obstacle avoidance cost factor for 2 X 3D arms
 *          with SE(2) base and linear actuator
 *  @author Mustafa Mukadam
 *  @date   Sep 4, 2017
 **/

#pragma once

#include <cpp/thirdparty/gpmp2/kinematics/Pose2MobileVetLin2ArmsModel.h>
#include <cpp/thirdparty/gpmp2/obstacle/ObstacleSDFFactor.h>

namespace incopt {

// template use Pose2MobileVetLin2ArmsModel as robot type
typedef ObstacleSDFFactor<Pose2MobileVetLin2ArmsModel> ObstacleSDFFactorPose2MobileVetLin2Arms;

}
