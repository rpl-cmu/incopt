/**
 *  @file  ObstaclePlanarSDFFactorGPArm.h
 *  @brief Obstacle avoidance cost factor, using Arm planar, linear GP and signed distance field
 *  @author Jing Dong
 *  @date  May 29, 2016
 **/

#pragma once

#include <cpp/thirdparty/gpmp2/kinematics/ArmModel.h>
#include <cpp/thirdparty/gpmp2/gp/GaussianProcessInterpolatorLinear.h>
#include <cpp/thirdparty/gpmp2/obstacle/ObstaclePlanarSDFFactorGP.h>

namespace incopt {

// template use ArmModel as robot type
typedef ObstaclePlanarSDFFactorGP<ArmModel, GaussianProcessInterpolatorLinear> 
    ObstaclePlanarSDFFactorGPArm;

}


