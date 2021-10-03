/************************************************************************
Copyright (c) 2018-2019, Unitree Robotics.Co.Ltd. All rights reserved.
Use of this source code is governed by the MPL-2.0 license, see LICENSE.
************************************************************************/
 /*!
  * @file body.h
  * @brief execute motor commands
  */ 

#ifndef _BODY_H_
#define _BODY_H_

#include "ros/ros.h"
#include "laikago_msgs/LowCmd.h"
#include "laikago_msgs/LowState.h"
#include "laikago_msgs/HighState.h"
#include "laikago_msgs/CheaterState.h"
#define PosStopF (2.146E+9f)
#define VelStopF (16000.f)

namespace laikago_model {
extern ros::Publisher servo_pub[12];
extern ros::Publisher highState_pub;
extern ros::Publisher lowState_pub;
extern laikago_msgs::LowCmd lowCmd;
extern laikago_msgs::LowState lowState;
extern laikago_msgs::HighState highState;

void paramInit();
void stand();
void motion_init();
void sendServoCmd();
void moveAllPosition(double* targetPos, double duration);

void motion_reset();
// Function for jumping
void paramJumpInit();
void JumpPrepare();
void Jump_Init();
void sendPrepareCmd();

};


#endif
