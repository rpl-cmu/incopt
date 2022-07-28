/**
 *  @file   ObstaclePlanarSDFFactorGPPose2MobileArm.h
 *  @brief  Obstacle avoidance cost factor, for point robot, using signed distance field,
 *          and GP interpolation
 *  @author Jing Dong
 *  @date   Oct 14, 2016
 **/

#pragma once

#include <cpp/thirdparty/gpmp2/kinematics/Pose2MobileArmModel.h>
#include <cpp/thirdparty/gpmp2/gp/GaussianProcessInterpolatorPose2Vector.h>
#include <cpp/thirdparty/gpmp2/obstacle/ObstaclePlanarSDFFactorGP.h>

namespace incopt {

// template uses PointRobotModel as robot type
typedef ObstaclePlanarSDFFactorGP<Pose2MobileArmModel, GaussianProcessInterpolatorPose2Vector>
    ObstaclePlanarSDFFactorGPPose2MobileArm;

}
