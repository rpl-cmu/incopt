/**
 *  @file   SelfCollisionArm.h
 *  @brief  Self collision cost factor for Arm
 *  @author Mustafa Mukadam
 *  @date   Sep 22, 2020
 **/


#pragma once

#include <cpp/thirdparty/gpmp2/kinematics/ArmModel.h>
#include <cpp/thirdparty/gpmp2/obstacle/SelfCollision.h>

namespace incopt {

// template use ArmModel as robot type
typedef SelfCollision<ArmModel> SelfCollisionArm;

}
