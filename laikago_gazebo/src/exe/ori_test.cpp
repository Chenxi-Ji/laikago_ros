/*!
 * @file ori_test.cpp
 * @brief test orientation estimator based on position control
 * 
 * remeber to set joint kp & kd (position/velocity stiffness) in body.cpp
 * 
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
#include "fstream"

using namespace std;
using namespace laikago_model;

int main(int argc, char ** argv){
    ros::init(argc, argv, "aliengo_gazebo_leg");
    ros::Publisher lowState_pub;
    ros::NodeHandle n;

    ofstream myfile;
    myfile.open("footForce.txt");

    CurrentState listen_publish_obj;
    ros::AsyncSpinner spinner(1);
    spinner.start();
    usleep(300000);
    ros::Rate r(1000);
    lowState_pub = n.advertise<laikago_msgs::LowState>("/laikago_gazebo/lowState/state", 1);
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

    std::cout << "init Joint Parameters" << std::endl;
    motion_init();

    ofstream imu;
    imu.open("imu.txt");

    double dt = 0.001;
    Quadruped quad;
    std::cout << "init Leg Controller" << std::endl;
    quad.setQuadruped(2);
    LegController* legController = new LegController(quad);

    std::cout << "init state estimator" << std::endl;
    StateEstimate stateEstimate;
    StateEstimatorContainer* stateEstimator = new StateEstimatorContainer(
                       &lowState.imu, legController->data,&stateEstimate);
    stateEstimator->addEstimator<ContactEstimator>();
    stateEstimator->addEstimator<VectorNavOrientationEstimator>();
    stateEstimator->addEstimator<LinearKFPositionVelocityEstimator>();
    ros::spinOnce();
    stateEstimator->run();
    legController->updateData();
    std::cout << legController->data[0].p << std::endl;
    while (ros::ok()){
     stateEstimator->run();

     imu << stateEstimator->getResult().vWorld[0] << " " << stateEstimator->getResult().vWorld[1] << std::endl;
    
    r.sleep();
    legController->updateData();
    //std::cout << "q " << legController->data[0].q << std::endl;
    //std::cout << "p_1: " << legController->data[1].p << std::endl;

    }
    imu.close();
    
    //pitch test
    // double pos1[12] = {0.0, 0.5, -1.4, -0.0, 0.5, -1.4, 
    //                    0.0, 1.5, -2.4, -0.0, 1.5, -2.4};
    // moveAllPosition(pos1, 2000);
    // stateEstimator->run();
    // std::cout << "pitch test 1" << std::endl;
    // std::cout << stateEstimator->getResult().rpy << std::endl;
    // usleep(1000000);
    // stand();
    // usleep(1000000);
    // double pos2[12] =  {0.0, 1.0, -2.4, -0.0, 1.0, -2.4, 
    //                     0.0, 0.5, -1.4, -0.0, 0.5, -1.4};

    // moveAllPosition(pos2, 2000);
    // stateEstimator->run();
    // std::cout << "pitch test 1" << std::endl;
    // std::cout << stateEstimator->getResult().rpy << std::endl;
    // usleep(1000000);
    // stand();
    // usleep(1000000);
    // //roll test
    // double pos3[12] = {-1, 1.0, -2.3, 0.0, 0.5, -1.4, 
    //                  -1, 1.0, -2.3, 0.0, 0.5, -1.4};
    // moveAllPosition(pos3, 2000);
    // stateEstimator->run();
    // std::cout << "roll test 1" << std::endl;
    // std::cout << stateEstimator->getResult().rpy << std::endl;
    // usleep(1000000);
   // stand();
   // usleep(1000000);
   // double pos4[12] = {0.0, 0.3, -1.2, 0.8, 1.0, -2.1,
            //           0.0, 0.3, -1.2, 0.8, 1.0, -2.1};
    //moveAllPosition(pos4, 2000);
   // stateEstimator->run();
   // std::cout << "roll test 2" << std::endl;
   // std::cout << stateEstimator->getResult().rpy << std::endl;
   // usleep(1000000);
    
    // yaw test
    //double pos5[12] = {-0.5, 0.8, -1.8, -0.5, 0.8, -1.8, 
     //                  0.5, 0.8, -1.8, 0.5, 0.8, -1.8};
   // moveAllPosition(pos5,2000);
    //stand();
    //stateEstimator->run();
   // std::cout << "yaw test 1" << std::endl;
   // std::cout << stateEstimator->getResult().rpy << std::endl;
    return 0;
    }