/**
 *  @file  ArmModel.h
 *  @brief Arm with physical body, which represented by spheres
 *  @author Jing Dong
 *  @date  Dec 30, 2015
 **/

#pragma once

#include <cpp/thirdparty/gpmp2/kinematics/Arm.h>
#include <cpp/thirdparty/gpmp2/kinematics/RobotModel.h>

namespace incopt {

/**
 * Arm with physical body, which is represented by spheres
 * Used to check collisions
 */
typedef RobotModel<Arm> ArmModel;

}

