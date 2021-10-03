#include "ros/ros.h"
#include <stdio.h>
#include <stdlib.h>
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
#include <sstream>
#include <fstream>

using namespace std;
using namespace laikago_model;

bool start_up = true;

class multiThread
{
public:
    multiThread(){
        imu_sub = nm.subscribe("/trunk_imu", 1, &multiThread::imuCallback, this);
        footForce_sub[0] = nm.subscribe("/visual/FR_foot_contact/the_force", 1, &multiThread::FRfootCallback, this);
        footForce_sub[1] = nm.subscribe("/visual/FL_foot_contact/the_force", 1, &multiThread::FLfootCallback, this);
        footForce_sub[2] = nm.subscribe("/visual/RR_foot_contact/the_force", 1, &multiThread::RRfootCallback, this);
        footForce_sub[3] = nm.subscribe("/visual/RL_foot_contact/the_force", 1, &multiThread::RLfootCallback, this);
        servo_sub[0] = nm.subscribe("/laikago_gazebo/FR_hip_controller/state", 1, &multiThread::FRhipCallback, this);
        servo_sub[1] = nm.subscribe("/laikago_gazebo/FR_thigh_controller/state", 1, &multiThread::FRthighCallback, this);
        servo_sub[2] = nm.subscribe("/laikago_gazebo/FR_calf_controller/state", 1, &multiThread::FRcalfCallback, this);
        servo_sub[3] = nm.subscribe("/laikago_gazebo/FL_hip_controller/state", 1, &multiThread::FLhipCallback, this);
        servo_sub[4] = nm.subscribe("/laikago_gazebo/FL_thigh_controller/state", 1, &multiThread::FLthighCallback, this);
        servo_sub[5] = nm.subscribe("/laikago_gazebo/FL_calf_controller/state", 1, &multiThread::FLcalfCallback, this);
        servo_sub[6] = nm.subscribe("/laikago_gazebo/RR_hip_controller/state", 1, &multiThread::RRhipCallback, this);
        servo_sub[7] = nm.subscribe("/laikago_gazebo/RR_thigh_controller/state", 1, &multiThread::RRthighCallback, this);
        servo_sub[8] = nm.subscribe("/laikago_gazebo/RR_calf_controller/state", 1, &multiThread::RRcalfCallback, this);
        servo_sub[9] = nm.subscribe("/laikago_gazebo/RL_hip_controller/state", 1, &multiThread::RLhipCallback, this);
        servo_sub[10] = nm.subscribe("/laikago_gazebo/RL_thigh_controller/state", 1, &multiThread::RLthighCallback, this);
        servo_sub[11] = nm.subscribe("/laikago_gazebo/RL_calf_controller/state", 1, &multiThread::RLcalfCallback, this);
    }

    void imuCallback(const sensor_msgs::Imu & msg)
    { 
        lowState.imu.quaternion[0] = msg.orientation.w;
        lowState.imu.quaternion[1] = msg.orientation.x;
        lowState.imu.quaternion[2] = msg.orientation.y;
        lowState.imu.quaternion[3] = msg.orientation.z;
    }

    void FRhipCallback(const laikago_msgs::MotorState& msg)
    {
        start_up = false;
        lowState.motorState[0].mode = msg.mode;
        lowState.motorState[0].position = msg.position;
        lowState.motorState[0].velocity = msg.velocity;
        lowState.motorState[0].torque = msg.torque;
    }

    void FRthighCallback(const laikago_msgs::MotorState& msg)
    {
        lowState.motorState[1].mode = msg.mode;
        lowState.motorState[1].position = msg.position;
        lowState.motorState[1].velocity = msg.velocity;
        lowState.motorState[1].torque = msg.torque;
    }

    void FRcalfCallback(const laikago_msgs::MotorState& msg)
    {
        lowState.motorState[2].mode = msg.mode;
        lowState.motorState[2].position = msg.position;
        lowState.motorState[2].velocity = msg.velocity;
        lowState.motorState[2].torque = msg.torque;
    }

    void FLhipCallback(const laikago_msgs::MotorState& msg)
    {
        start_up = false;
        lowState.motorState[3].mode = msg.mode;
        lowState.motorState[3].position = msg.position;
        lowState.motorState[3].velocity = msg.velocity;
        lowState.motorState[3].torque = msg.torque;
    }

    void FLthighCallback(const laikago_msgs::MotorState& msg)
    {
        lowState.motorState[4].mode = msg.mode;
        lowState.motorState[4].position = msg.position;
        lowState.motorState[4].velocity = msg.velocity;
        lowState.motorState[4].torque = msg.torque;
    }

    void FLcalfCallback(const laikago_msgs::MotorState& msg)
    {
        lowState.motorState[5].mode = msg.mode;
        lowState.motorState[5].position = msg.position;
        lowState.motorState[5].velocity = msg.velocity;
        lowState.motorState[5].torque = msg.torque;
    }

    void RRhipCallback(const laikago_msgs::MotorState& msg)
    {
        start_up = false;
        lowState.motorState[6].mode = msg.mode;
        lowState.motorState[6].position = msg.position;
        lowState.motorState[6].velocity = msg.velocity;
        lowState.motorState[6].torque = msg.torque;
    }

    void RRthighCallback(const laikago_msgs::MotorState& msg)
    {
        lowState.motorState[7].mode = msg.mode;
        lowState.motorState[7].position = msg.position;
        lowState.motorState[7].velocity = msg.velocity;
        lowState.motorState[7].torque = msg.torque;
    }

    void RRcalfCallback(const laikago_msgs::MotorState& msg)
    {
        lowState.motorState[8].mode = msg.mode;
        lowState.motorState[8].position = msg.position;
        lowState.motorState[8].velocity = msg.velocity;
        lowState.motorState[8].torque = msg.torque;
    }

    void RLhipCallback(const laikago_msgs::MotorState& msg)
    {
        start_up = false;
        lowState.motorState[9].mode = msg.mode;
        lowState.motorState[9].position = msg.position;
        lowState.motorState[9].velocity = msg.velocity;
        lowState.motorState[9].torque = msg.torque;
    }

    void RLthighCallback(const laikago_msgs::MotorState& msg)
    {
        lowState.motorState[10].mode = msg.mode;
        lowState.motorState[10].position = msg.position;
        lowState.motorState[10].velocity = msg.velocity;
        lowState.motorState[10].torque = msg.torque;
    }

    void RLcalfCallback(const laikago_msgs::MotorState& msg)
    {
        lowState.motorState[11].mode = msg.mode;
        lowState.motorState[11].position = msg.position;
        lowState.motorState[11].velocity = msg.velocity;
        lowState.motorState[11].torque = msg.torque;
    }

    void FRfootCallback(const geometry_msgs::WrenchStamped& msg)
    {
        lowState.eeForce[0].x = msg.wrench.force.x;
        lowState.eeForce[0].y = msg.wrench.force.y;
        lowState.eeForce[0].z = msg.wrench.force.z;
        lowState.footForce[0] = msg.wrench.force.z;
    }

    void FLfootCallback(const geometry_msgs::WrenchStamped& msg)
    {
        lowState.eeForce[1].x = msg.wrench.force.x;
        lowState.eeForce[1].y = msg.wrench.force.y;
        lowState.eeForce[1].z = msg.wrench.force.z;
        lowState.footForce[1] = msg.wrench.force.z;
    }

    void RRfootCallback(const geometry_msgs::WrenchStamped& msg)
    {
        lowState.eeForce[2].x = msg.wrench.force.x;
        lowState.eeForce[2].y = msg.wrench.force.y;
        lowState.eeForce[2].z = msg.wrench.force.z;
        lowState.footForce[2] = msg.wrench.force.z;
    }

    void RLfootCallback(const geometry_msgs::WrenchStamped& msg)
    {
        lowState.eeForce[3].x = msg.wrench.force.x;
        lowState.eeForce[3].y = msg.wrench.force.y;
        lowState.eeForce[3].z = msg.wrench.force.z;
        lowState.footForce[3] = msg.wrench.force.z;
    }

private:
    ros::NodeHandle nm;
    ros::Subscriber servo_sub[8], footForce_sub[4], imu_sub;
};

int main(int argc, char **argv)
{
    // Read the optimization data from imported files
    // Stored the data into vectors

    // Position
    vector<vector<double>> qDes;

    // Velocity
    vector<vector<double>> dqDes;

    // Torque
    vector<vector<double>> tau;

    // Read the joint velocity and position from optimization data
    std::ifstream myFile;
    myFile.open("src/laikago_ros/optimization_data/data_Q.csv");

    // If cannot open the file, report an error
    if(!myFile.is_open())
    {
        std::cout << "Could not open file for position and velocity" << std::endl;
        return 0;
    }
    cout << "Reading Optimization Data for Position and Velocity" << endl;
    string line;
    int index = 0;

    while(getline(myFile, line))
    {
        // Line[3] stores the position for front thigh
        if(index == 3)
        {
            stringstream ss(line);
            double val;
            vector<double> q_d_front_thigh;
            while(ss >> val)
            {
                q_d_front_thigh.push_back(val);
                if(ss.peek() == ',') ss.ignore();
            }
            qDes.push_back(q_d_front_thigh);
        }
        // Line[4] stores the position for front calf
        if(index == 4)
        {
            stringstream ss(line);
            double val;
            vector<double> q_d_front_calf;
            while(ss >> val)
            {
                q_d_front_calf.push_back(val);
                if(ss.peek() == ',') ss.ignore();
            }
            qDes.push_back(q_d_front_calf);
        }

        // Line[5] stores the position for rear thigh
        if(index == 5)
        {
            stringstream ss(line);
            double val;
            vector<double> q_d_rear_thigh;
            while(ss >> val)
            {
                q_d_rear_thigh.push_back(val);
                if(ss.peek() == ',') ss.ignore();
            }
            qDes.push_back(q_d_rear_thigh);
        }

        // Line[6] stores the position for rear calf
        if(index == 6)
        {
            stringstream ss(line);
            double val;
            vector<double> q_d_rear_calf;
            while(ss >> val)
            {
                q_d_rear_calf.push_back(val);
                if(ss.peek() == ',') ss.ignore();
            }
            qDes.push_back(q_d_rear_calf);
        }

        // Line[10] stores the velocity for front thigh
        if(index == 10)
        {
            stringstream ss(line);
            double val;
            vector<double> dq_d_front_thigh;
            while(ss >> val)
            {
                dq_d_front_thigh.push_back(val);
                if(ss.peek() ==',') ss.ignore();
            }
            dqDes.push_back(dq_d_front_thigh);
        }
        // Line[11] stores the velocity for front calf
        if(index == 11)
        {
            stringstream ss(line);
            double val;
            vector<double> dq_d_front_calf;
            while(ss >> val)
            {
                dq_d_front_calf.push_back(val);
                if(ss.peek() ==',') ss.ignore();
            }
            dqDes.push_back(dq_d_front_calf);
        }

        // Line[12] stores the velocity for rear thigh
        if(index == 12)
        {
            stringstream ss(line);
            double val;
            vector<double> dq_d_rear_thigh;
            while(ss >> val)
            {
                dq_d_rear_thigh.push_back(val);
                if(ss.peek() ==',') ss.ignore();
            }
            dqDes.push_back(dq_d_rear_thigh);
        }

        // Line[13] stores the velocity for rear calf
        if(index == 13)
        {
            stringstream ss(line);
            double val;
            vector<double> dq_d_rear_calf;
            while(ss >> val)
            {
                dq_d_rear_calf.push_back(val);
                if(ss.peek() ==',') ss.ignore();
            }
            dqDes.push_back(dq_d_rear_calf);
        }
        index++;
    }
    myFile.close();

    // Read the joint velocity and position from optimization data
    std::ifstream mytauFile("src/laikago_ros/optimization_data/data_tau.csv");

    // If cannot open the file, report an error
    if(!mytauFile.is_open())
    {
        std::cout << "Could not open file for torque" << endl;
        return 0;
    }
    cout << "Reading Optimization Data for Torque" << endl;
    
    // Using for Debug

    // std::ofstream Thighposition("src/USC-AlienGo-Software/laikago_ros/result/Thighposition.txt");
    // std::ofstream Thighvelocity("src/USC-AlienGo-Software/laikago_ros/result/Thighvelocity.txt");
    // std::ofstream Thightorque("src/USC-AlienGo-Software/laikago_ros/result/Thightorque.txt");
    // std::ofstream Calfposition("src/USC-AlienGo-Software/laikago_ros/result/Calfposition.txt");
    // std::ofstream Calfvelocity("src/USC-AlienGo-Software/laikago_ros/result/Calfvelocity.txt");
    // std::ofstream Calftorque("src/USC-AlienGo-Software/laikago_ros/result/Calftorque.txt");
    // std::ofstream Hipposition("src/USC-AlienGo-Software/laikago_ros/result/Hipposition.txt");
    // std::ofstream Hipvelocity("src/USC-AlienGo-Software/laikago_ros/result/Hipvelocity.txt");
    // std::ofstream Hiptorque("src/USC-AlienGo-Software/laikago_ros/result/Hiptorque.txt");
    
    index = 0;
    while(getline(mytauFile, line))
    {
        stringstream ss(line);
        double val;
        vector<double> tau_joint;
        while(ss >> val)
        {
            tau_joint.push_back(val/2.0);
            if(ss.peek() == ',') ss.ignore();
        }
        tau.push_back(tau_joint);
    }
    mytauFile.close();
    
    ros::init(argc, argv, "laikago_gazebo_jump");

    multiThread listen_publish_obj;
    ros::AsyncSpinner spinner(1); // one threads
    spinner.start();
    usleep(2000000); // must wait 300ms, to get first state
    ros::NodeHandle n;
    ros::Publisher lowState_pub; //for rviz visualization

    // Publish the command every 1ms (1000Hz)
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

    std::cout << "init Joint Parameters and Stand Up" << std::endl;

    Jump_Init();

    if(ros::ok()){
        usleep(5000000);
        // Set the Kp and Kd value
        double Kp = 200;
        double Kd = 50;
        double Hip_Kp = 300;
        double Hip_Kd = 100;
        std::cout << "Start jumping" << std::endl;
        for(int i = 0; i < qDes[0].size();i++)
        {
            // Using for Debug
            // Thighposition << lowState.motorState[1].position << " " << lowState.motorState[4].position << " " <<lowState.motorState[7].position <<  " " << lowState.motorState[10].position << endl;           
            // Thighvelocity << lowState.motorState[1].velocity << " " << lowState.motorState[4].velocity << " " <<lowState.motorState[7].velocity <<  " " << lowState.motorState[10].velocity << endl;
            // Thightorque << lowState.motorState[1].torque << " " << lowState.motorState[4].torque << " " <<lowState.motorState[7].torque <<  " " << lowState.motorState[10].torque << endl;
            // Calfposition << lowState.motorState[2].position << " " << lowState.motorState[5].position << " " <<lowState.motorState[8].position <<  " " << lowState.motorState[11].position << endl;
            // Calfvelocity << lowState.motorState[2].velocity << " " << lowState.motorState[5].velocity << " " <<lowState.motorState[8].velocity <<  " " << lowState.motorState[11].velocity << endl;
            // Calftorque << lowState.motorState[2].torque << " " << lowState.motorState[5].torque << " " <<lowState.motorState[8].torque <<  " " << lowState.motorState[11].torque << endl;
            // Hipposition << lowState.motorState[0].position << " " << lowState.motorState[3].position << " " <<lowState.motorState[6].position <<  " " << lowState.motorState[9].position << endl;
            // Hipvelocity << lowState.motorState[0].velocity << " " << lowState.motorState[3].velocity << " " <<lowState.motorState[6].velocity <<  " " << lowState.motorState[9].velocity << endl;
            // Hiptorque << lowState.motorState[0].torque << " " << lowState.motorState[3].torque << " " <<lowState.motorState[6].torque <<  " " << lowState.motorState[9].torque << endl;

            for(int j = 0 ; j < 4; j++)
            {
                // Give command to hip joint
                lowCmd.motorCmd[j*3].position = 0;
                lowCmd.motorCmd[j*3].velocity = 0;
                lowCmd.motorCmd[j*3].positionStiffness = Hip_Kp;
                lowCmd.motorCmd[j*3].velocityStiffness = Hip_Kd;
                // Give command to thigh and calf
                for(int k = 1; k < 3; k++)
                {
                    // Two front legs
                    if(j < 2)
                    {
                        lowCmd.motorCmd[j*3+k].position = -qDes[k-1][i];
                        lowCmd.motorCmd[j*3+k].velocity = -dqDes[k-1][i];
                        lowCmd.motorCmd[j*3+k].torque = -tau[k-1][i];
                        lowCmd.motorCmd[j*3+k].positionStiffness = Kp;
                        lowCmd.motorCmd[j*3+k].velocityStiffness = Kd;
                    }
                    // Two rear legs
                    else
                    {
                        lowCmd.motorCmd[j*3+k].position = -qDes[k+1][i];
                        lowCmd.motorCmd[j*3+k].velocity = -dqDes[k+1][i];
                        lowCmd.motorCmd[j*3+k].torque = -tau[k+1][i];
                        lowCmd.motorCmd[j*3+k].positionStiffness = Kp;
                        lowCmd.motorCmd[j*3+k].velocityStiffness = Kd;
                    }
                }
            }
            sendServoCmd();
            r.sleep();
        } 
        
    }
    Jump_Init();

    // Using for Debug
    // Thighposition.close();
    // Thighvelocity.close();
    // Thightorque.close();
    // Calfposition.close();
    // Calfvelocity.close();
    // Calftorque.close();
    // Hipposition.close();
    // Hipvelocity.close();
    // Hiptorque.close();
    return 0;
}
