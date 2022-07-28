/**
 *  @file  ObstaclePlanarSDFFactorPose2MobileArm.h
 *  @brief Obstacle avoidance cost factor, using Arm planar and signed distance field
 *  @author Jing Dong
 *  @date  Oct 13, 2016
 **/

#pragma once

#include <cpp/thirdparty/gpmp2/kinematics/Pose2MobileArmModel.h>
#include <cpp/thirdparty/gpmp2/obstacle/ObstaclePlanarSDFFactor.h>

namespace incopt {

// template use ArmModel as robot type
typedef ObstaclePlanarSDFFactor<Pose2MobileArmModel> ObstaclePlanarSDFFactorPose2MobileArm;

}


