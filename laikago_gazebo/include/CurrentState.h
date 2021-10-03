/*!
 * @file CurrentState.h
 * @brief function as FSM of Aliengo robot
 * call back all data of current state
 * 0 /1 /2 /3
 * FR/FL/RR/RL
 */ 
#ifndef CURRENT_STATE_H
#define CURRENT_STATE_H

#include "ros/ros.h"
#include <stdio.h>
#include <stdlib.h>
#include "laikago_msgs/qp_controller_data_t.h"
#include "laikago_msgs/HighCmd.h"
#include "laikago_msgs/HighState.h"
#include "laikago_msgs/LowCmd.h"
#include "laikago_msgs/LowState.h"
#include "laikago_msgs/MotorCmd.h"
#include "laikago_msgs/MotorState.h"
#include "laikago_msgs/Cartesian.h"
#include "laikago_msgs/CheaterState.h"
#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/WrenchStamped.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Bool.h>
#include <vector>
#include <string.h>
#include <math.h>
#include "body.h"

using namespace std;
using namespace laikago_model;

//bool start_up = true;

class CurrentState {
    public:
        CurrentState();
        void cheaterCallback(const gazebo_msgs::ModelStates& msg);
        void imuCallback(const sensor_msgs::Imu& msg);
        void FRhipCallback(const laikago_msgs::MotorState& msg);
        void FRthighCallback(const laikago_msgs::MotorState& msg);
        void FRcalfCallback(const laikago_msgs::MotorState& msg);
        void FLhipCallback(const laikago_msgs::MotorState& msg);
        void FLthighCallback(const laikago_msgs::MotorState& msg);
        void FLcalfCallback(const laikago_msgs::MotorState& msg);
        void RRhipCallback(const laikago_msgs::MotorState& msg);
        void RRthighCallback(const laikago_msgs::MotorState& msg);
        void RRcalfCallback(const laikago_msgs::MotorState& msg);
        void RLhipCallback(const laikago_msgs::MotorState& msg);
        void RLthighCallback(const laikago_msgs::MotorState& msg);
        void RLcalfCallback(const laikago_msgs::MotorState& msg);
        void FRfootCallback(const geometry_msgs::WrenchStamped& msg);
        void FLfootCallback(const geometry_msgs::WrenchStamped& msg);
        void RRfootCallback(const geometry_msgs::WrenchStamped& msg);
        void RLfootCallback(const geometry_msgs::WrenchStamped& msg);
        
       // void HighStateCallback(const laikago_msgs::HighState& msg);
    private:
        ros::NodeHandle nm;
        ros::Subscriber servo_sub[12], footForce_sub[4], imu_sub, state_sub, qp_sub;
};
#endif