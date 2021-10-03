/*!
 * @file DRCL.cpp
 * @brief executable to operate Aliengo in gazebo
 */ 

#include "ros/ros.h"
#include <stdio.h>
#include <stdlib.h>
#include "laikago_msgs/HighCmd.h"
#include "laikago_msgs/HighState.h"
#include "laikago_msgs/LowCmd.h"
#include "laikago_msgs/LowState.h"
#include "laikago_msgs/MotorCmd.h"
#include "laikago_msgs/MotorState.h"
#include <geometry_msgs/WrenchStamped.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Bool.h>
#include <vector>
#include <string.h>
#include <math.h>
#include <nav_msgs/Odometry.h>
#include "../../include/body.h"
#include "../../include/OrientationEstimator.h"
#include "../../include/PositionVelocityEstimator.h"
#include "../../include/ContactEstimator.h"
#include "../../include/LegController.h"
#include "../../include/FSM.h"
#include "../../include/FootSwingTrajectory.h"

using namespace std;
using namespace laikago_model;

int main(int argc, char **argv){
    ros::init(argc, argv, "aliengo_gazebo_servo");

    CurrentState cs;
    ros::AsyncSpinner spinner(1);
    ros::Rate loop_rate(1000);
    spinner.start();
    usleep(300000); 

    
    
    ros::NodeHandle n;
    //ros::Publisher lowState_pub, highCmd_pub;

    //highCmd_pub = n.advertise<laikago_msgs::HighCmd>("laikago_gazebo/highCmd/Command",1);
    //highState_pub = n.advertise<laikago_msgs::HighState>("laikago_gazebo/highState/state", 1);
    //lowState_pub = n.advertise<laikago_msgs::LowState>("/laikago_gazebo/lowState/state", 1);
    servo_pub[0] = n.advertise<laikago_msgs::MotorCmd>("/laikago_gazebo/FR_hip_controller/command", 1);
    servo_pub[1] = n.advertise<laikago_msgs::MotorCmd>("/laikago_gazebo/FR_thigh_controller/command", 1);
    servo_pub[2] = n.advertise<laikago_msgs::MotorCmd>("/laikago_gazebo/FR_calf_controller/command", 1);
    servo_pub[3] = n.advertise<laikago_msgs::MotorCmd>("/laikago_gazebo/FL_hip_controller/command", 1);
    servo_pub[4] = n.advertise<laikago_msgs::MotorCmd>("/laikago_gazebo/FL_thigh_controller/command", 1);
    servo_pub[5] = n.advertise<laikago_msgs::MotorCmd>("/laikago_gazebo/FL_calf_controller/command", 1);
    servo_pub[6] = n.advertise<laikago_msgs::MotorCmd>("/laikago_gazebo/RR_hip_controller/command", 1);
    servo_pub[7] = n.advertise<laikago_msgs::MotorCmd>("/laikago_gazebo/RR_thigh_controller/command", 1);
    servo_pub[8] = n.advertise<laikago_msgs::MotorCmd>("/laikago_gazebo/RR_calf_controller/command", 1);
    servo_pub[9] = n.advertise<laikago_msgs::MotorCmd>("/laikago_gazebo/RL_hip_controller/command", 1);
    servo_pub[10] = n.advertise<laikago_msgs::MotorCmd>("/laikago_gazebo/RL_thigh_controller/command", 1);
    servo_pub[11] = n.advertise<laikago_msgs::MotorCmd>("/laikago_gazebo/RL_calf_controller/command", 1);

    std::cout << "initialize" << std::endl;
    double dt = 0.001; 

    // robot model
    Quadruped quad;
    quad.setQuadruped(1); // 1 for Aliengo, 2 for A1

    // state estimation
    StateEstimate stateEstimate;
    LegController* legController = new LegController(quad);

    StateEstimatorContainer* stateEstimator = new StateEstimatorContainer(
                       &lowState.imu, legController->data,&stateEstimate);
    stateEstimator->addEstimator<ContactEstimator>();
    stateEstimator->addEstimator<VectorNavOrientationEstimator>();
    stateEstimator->addEstimator<LinearKFPositionVelocityEstimator>();
    
   

    GaitScheduler<double>* gaitScheduler = new GaitScheduler<double>(dt);
    teleCmd* _tele = new teleCmd();
    DesiredStateCommand* desiredStateCommand = new DesiredStateCommand(_tele, &stateEstimate, dt);

    ControlFSMData* _controlData = new ControlFSMData;
    _controlData->_quadruped = &quad;
    _controlData->_stateEstimator = stateEstimator;
    _controlData->_gaitScheduler = gaitScheduler;
    _controlData->_legController = legController;
    _controlData->_desiredStateCommand = desiredStateCommand;

    //_controlData->_testEstimator = stateEstimator;

    FSM_State* fsm = new FSM_State(_controlData);
    motion_init();

    // while(ros::ok()){
    //     legController->updateData();
    //     stateEstimator->run();
    //     double p_act[3];
    //     for(int i = 0; i < 3; i++){
    //        p_act[i] = _controlData->_stateEstimator->getResult().position(i); 
    //     }

    //     std::cout << "x_pos: " << p_act[0] << std::endl;
    //     std::cout << "rpy: " << _controlData->_stateEstimator->getResult().rpy << std::endl;
    //     ros::spinOnce();
    //     loop_rate.sleep();
    // }

    // fsm->SprayDemo();
    fsm->QPstand();
    // fsm->runQPloco();

    delete desiredStateCommand;
    delete gaitScheduler;
    delete legController;
    delete stateEstimator;
    delete _tele;
    delete _controlData;
    delete fsm;

 
    return 0;
} 