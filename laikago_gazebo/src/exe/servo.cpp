/************************************************************************
Copyright (c) 2018-2019, Unitree Robotics.Co.Ltd. All rights reserved.
Use of this source code is governed by the MPL-2.0 license, see LICENSE.
************************************************************************/

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
//#include "../../include/Dynamics/AlienGo.h"


using namespace std;
using namespace laikago_model;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "laikago_gazebo_servo");
    ros::Publisher lowCmd_pub, highCmd_pub; //for rviz visualization
    ros::NodeHandle n;
   // multiThread multi;
  /*!  ros::AsyncSpinner spin(1);
    spin.start();
    usleep(300000);
    
    ros::NodeHandle nm;
   // usleep(300000); // must wait 300ms, to get first state
    lowState_pub = nm.advertise<laikago_msgs::LowState>("/aliengo_gazebo/lowState/state", 1);
    servo_pub[0] = nm.advertise<laikago_msgs::MotorCmd>("/aliengo_gazebo/FR_hip_controller/command", 1);
    servo_pub[1] = nm.advertise<laikago_msgs::MotorCmd>("/aliengo_gazebo/FR_thigh_controller/command", 1);
    servo_pub[2] = nm.advertise<laikago_msgs::MotorCmd>("/aliengo_gazebo/FR_calf_controller/command", 1);
    servo_pub[3] = nm.advertise<laikago_msgs::MotorCmd>("/aliengo_gazebo/FL_hip_controller/command", 1);
    servo_pub[4] = nm.advertise<laikago_msgs::MotorCmd>("/aliengo_gazebo/FL_thigh_controller/command", 1);
    servo_pub[5] = nm.advertise<laikago_msgs::MotorCmd>("/aliengo_gazebo/FL_calf_controller/command", 1);
    servo_pub[6] = nm.advertise<laikago_msgs::MotorCmd>("/aliengo_gazebo/RR_hip_controller/command", 1);
    servo_pub[7] = nm.advertise<laikago_msgs::MotorCmd>("/aliengo_gazebo/RR_thigh_controller/command", 1);
    servo_pub[8] = nm.advertise<laikago_msgs::MotorCmd>("/aliengo_gazebo/RR_calf_controller/command", 1);
    servo_pub[9] = nm.advertise<laikago_msgs::MotorCmd>("/aliengo_gazebo/RL_hip_controller/command", 1);
    servo_pub[10] = nm.advertise<laikago_msgs::MotorCmd>("/aliengo_gazebo/RL_thigh_controller/command", 1);
    servo_pub[11] = nm.advertise<laikago_msgs::MotorCmd>("/aliengo_gazebo/RL_calf_controller/command", 1);

    motion_init();
    */
   
    ros::AsyncSpinner spinner(1); // one threads
    ros::Rate loop_rate(1000);
    spinner.start();

    CurrentState listen_publish_obj;

    //usleep(300000); // must wait 300ms, to get first state
   
    

    // the following nodes have been initialized by "gazebo.launch"
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
  
    
    std::cout << "stand up" << std::endl;
    double dt = 0.001;
    Quadruped quad;
    quad.setQuadruped(1); // 1 for Aliengo, 2 for A1
    // initialize new leg controller and state estimate object
    StateEstimate stateEstimate;
    StateEstimate testEstimate;
    std::cout << "start Leg Controller" << std::endl;
    LegController* legController = new LegController(quad);

    std::cout << "start state estimate" << std::endl;
    StateEstimatorContainer* stateEstimator = new StateEstimatorContainer(
                       &lowState.imu, legController->data,&stateEstimate);
    stateEstimator->addEstimator<ContactEstimator>();
    stateEstimator->addEstimator<CheaterOrientationEstimator>();
    stateEstimator->addEstimator<CheaterPositionVelocityEstimator>();

    // StateEstimatorContainer* testEstimator = new StateEstimatorContainer(
    //                    &lowState.imu, legController->data,&testEstimate);
    // testEstimator->addEstimator<ContactEstimator>();
    // testEstimator->addEstimator<VectorNavOrientationEstimator>();
    // testEstimator->addEstimator<LinearKFPositionVelocityEstimator>();


    // initialize new Gait scheduler object

    std::cout << "start gait" << std::endl;
    GaitScheduler<double>* gaitScheduler = new GaitScheduler<double>(dt);
    gaitScheduler->initialize();
    // initialize desired command

    std::cout << "start cmd" << std::endl;
    teleCmd* _tele = new teleCmd();
    DesiredStateCommand* desiredStateCommand = new DesiredStateCommand(_tele, &stateEstimate, dt);
    

    // initialize FSM

    std::cout << "start controlFSMData" << std::endl;
    ControlFSMData* _controlData = new ControlFSMData;
    _controlData->_quadruped = &quad;
    _controlData->_stateEstimator = stateEstimator;
    _controlData->_gaitScheduler = gaitScheduler;
    _controlData->_legController = legController;
    _controlData->_desiredStateCommand = desiredStateCommand;

    //_controlData->_testEstimator = testEstimator;

    std::cout << "start fsm and init" << std::endl;
    FSM_State* fsm = new FSM_State(_controlData);
    motion_init();
    //enableTorqueMode(true);

    //motion_init();
    std::cout << "FSM_Started" << std::endl;
    
    // legController->updateData();
    // stateEstimator->run();

    //fsm->runLeastSquare();
    fsm->QPstand(); // MPC locomotion implemented in this function
    
    //fsm->PDstand(); // standup with PD control
    // fsm->SprayDemo();
    // fsm->runQPloco();
    
    //std::cout << "finished" << std::endl;
    //delete quad;

    delete desiredStateCommand;
    delete gaitScheduler;
    delete legController;
    delete stateEstimator;
    //delete testEstimator;
    delete _tele;
    delete _controlData;
    delete fsm;

    return 0;
}
