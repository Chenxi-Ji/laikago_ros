#include "../include/CurrentState.h"

using namespace laikago_model;

CurrentState::CurrentState() {
    imu_sub = nm.subscribe("/trunk_imu", 1, &CurrentState::imuCallback, this);
    state_sub = nm.subscribe("/gazebo/model_states", 1, &CurrentState::cheaterCallback, this);
    footForce_sub[0] = nm.subscribe("/visual/FR_foot_contact/the_force", 1, &CurrentState::FRfootCallback, this);
    footForce_sub[1] = nm.subscribe("/visual/FL_foot_contact/the_force", 1, &CurrentState::FLfootCallback, this);
    footForce_sub[2] = nm.subscribe("/visual/RR_foot_contact/the_force", 1, &CurrentState::RRfootCallback, this);
    footForce_sub[3] = nm.subscribe("/visual/RL_foot_contact/the_force", 1, &CurrentState::RLfootCallback, this);
    servo_sub[0] = nm.subscribe("/laikago_gazebo/FR_hip_controller/state", 1, &CurrentState::FRhipCallback, this);
    servo_sub[1] = nm.subscribe("/laikago_gazebo/FR_thigh_controller/state", 1, &CurrentState::FRthighCallback, this);
    servo_sub[2] = nm.subscribe("/laikago_gazebo/FR_calf_controller/state", 1, &CurrentState::FRcalfCallback, this);
    servo_sub[3] = nm.subscribe("/laikago_gazebo/FL_hip_controller/state", 1, &CurrentState::FLhipCallback, this);       
    servo_sub[4] = nm.subscribe("/laikago_gazebo/FL_thigh_controller/state", 1, &CurrentState::FLthighCallback, this);
    servo_sub[5] = nm.subscribe("/laikago_gazebo/FL_calf_controller/state", 1, &CurrentState::FLcalfCallback, this);
    servo_sub[6] = nm.subscribe("/laikago_gazebo/RR_hip_controller/state", 1, &CurrentState::RRhipCallback, this);
    servo_sub[7] = nm.subscribe("/laikago_gazebo/RR_thigh_controller/state", 1, &CurrentState::RRthighCallback, this);
    servo_sub[8] = nm.subscribe("/laikago_gazebo/RR_calf_controller/state", 1, &CurrentState::RRcalfCallback, this);
    servo_sub[9] = nm.subscribe("/laikago_gazebo/RL_hip_controller/state", 1, &CurrentState::RLhipCallback, this);
    servo_sub[10] = nm.subscribe("/laikago_gazebo/RL_thigh_controller/state", 1, &CurrentState::RLthighCallback, this);
    servo_sub[11] = nm.subscribe("/laikago_gazebo/RL_calf_controller/state", 1, &CurrentState::RLcalfCallback, this);
   //qp_sub = nm.subscribe("/laikago_gazebo/qp_controller_data_t", 1, &CurrentState::QPSolverCallback, this);
}

void CurrentState::cheaterCallback(const gazebo_msgs::ModelStates& msg)
    {
       lowState.cheat.orientation[0] = msg.pose[2].orientation.w;
       lowState.cheat.orientation[1] = msg.pose[2].orientation.x;
       lowState.cheat.orientation[2] = msg.pose[2].orientation.y;
       lowState.cheat.orientation[3] = msg.pose[2].orientation.z;
        
       lowState.cheat.position[0] = msg.pose[2].position.x;
       lowState.cheat.position[1] = msg.pose[2].position.y;
       lowState.cheat.position[2] = msg.pose[2].position.z;

       lowState.cheat.vWorld[0] = msg.twist[2].linear.x;
       lowState.cheat.vWorld[1] = msg.twist[2].linear.y;
       lowState.cheat.vWorld[2] = msg.twist[2].linear.z;

       lowState.cheat.omegaBody[0] = msg.twist[2].angular.x;
       lowState.cheat.omegaBody[1] = msg.twist[2].angular.y;
       lowState.cheat.omegaBody[2] = msg.twist[2].angular.z;
    }

void CurrentState::imuCallback(const sensor_msgs::Imu & msg)
    { 
        lowState.imu.quaternion[0] = msg.orientation.w;
        lowState.imu.quaternion[1] = msg.orientation.x;
        lowState.imu.quaternion[2] = msg.orientation.y;
        lowState.imu.quaternion[3] = msg.orientation.z;

        lowState.imu.acceleration[0] = msg.linear_acceleration.x;
        lowState.imu.acceleration[1] = msg.linear_acceleration.y;
        lowState.imu.acceleration[2] = msg.linear_acceleration.z;
        //std::cout << "imu_x_accel " << msg.linear_acceleration.x << std::endl;
        //std::cout << "imu_y_accel " << msg.linear_acceleration.y << std::endl;

        lowState.imu.gyroscope[0] = msg.angular_velocity.x;
        lowState.imu.gyroscope[1] = msg.angular_velocity.y;
        lowState.imu.gyroscope[2] = msg.angular_velocity.z;
    }

void CurrentState::FRhipCallback(const laikago_msgs::MotorState& msg)
    {
        //start_up = false;
        lowState.motorState[0].mode = msg.mode;
        lowState.motorState[0].position = msg.position;
        lowState.motorState[0].velocity = msg.velocity;
        lowState.motorState[0].torque = msg.torque;
    }

    void CurrentState::FRthighCallback(const laikago_msgs::MotorState& msg)
    {
        lowState.motorState[1].mode = msg.mode;
        lowState.motorState[1].position = msg.position;
        lowState.motorState[1].velocity = msg.velocity;
        lowState.motorState[1].torque = msg.torque;
    }

    void CurrentState::FRcalfCallback(const laikago_msgs::MotorState& msg)
    {
        lowState.motorState[2].mode = msg.mode;
        lowState.motorState[2].position = msg.position;
        lowState.motorState[2].velocity = msg.velocity;
        lowState.motorState[2].torque = msg.torque;
    }

    void CurrentState::FLhipCallback(const laikago_msgs::MotorState& msg)
    {
        //start_up = false;
        lowState.motorState[3].mode = msg.mode;
        lowState.motorState[3].position = msg.position;
        lowState.motorState[3].velocity = msg.velocity;
        lowState.motorState[3].torque = msg.torque;
    }

    void CurrentState::FLthighCallback(const laikago_msgs::MotorState& msg)
    {
        lowState.motorState[4].mode = msg.mode;
        lowState.motorState[4].position = msg.position;
        lowState.motorState[4].velocity = msg.velocity;
        lowState.motorState[4].torque = msg.torque;
    }

    void CurrentState::FLcalfCallback(const laikago_msgs::MotorState& msg)
    {
        lowState.motorState[5].mode = msg.mode;
        lowState.motorState[5].position = msg.position;
        lowState.motorState[5].velocity = msg.velocity;
        lowState.motorState[5].torque = msg.torque;
    }

    void CurrentState::RRhipCallback(const laikago_msgs::MotorState& msg)
    {
        //start_up = false;
        lowState.motorState[6].mode = msg.mode;
        lowState.motorState[6].position = msg.position;
        lowState.motorState[6].velocity = msg.velocity;
        lowState.motorState[6].torque = msg.torque;
    }

    void CurrentState::RRthighCallback(const laikago_msgs::MotorState& msg)
    {
        lowState.motorState[7].mode = msg.mode;
        lowState.motorState[7].position = msg.position;
        lowState.motorState[7].velocity = msg.velocity;
        lowState.motorState[7].torque = msg.torque;
    }

    void CurrentState::RRcalfCallback(const laikago_msgs::MotorState& msg)
    {
        lowState.motorState[8].mode = msg.mode;
        lowState.motorState[8].position = msg.position;
        lowState.motorState[8].velocity = msg.velocity;
        lowState.motorState[8].torque = msg.torque;
    }

    void CurrentState::RLhipCallback(const laikago_msgs::MotorState& msg)
    {
        //start_up = false;
        lowState.motorState[9].mode = msg.mode;
        lowState.motorState[9].position = msg.position;
        lowState.motorState[9].velocity = msg.velocity;
        lowState.motorState[9].torque = msg.torque;
    }

    void CurrentState::RLthighCallback(const laikago_msgs::MotorState& msg)
    {
        lowState.motorState[10].mode = msg.mode;
        lowState.motorState[10].position = msg.position;
        lowState.motorState[10].velocity = msg.velocity;
        lowState.motorState[10].torque = msg.torque;
    }

    void CurrentState::RLcalfCallback(const laikago_msgs::MotorState& msg)
    {
        lowState.motorState[11].mode = msg.mode;
        lowState.motorState[11].position = msg.position;
        lowState.motorState[11].velocity = msg.velocity;
        lowState.motorState[11].torque = msg.torque;
    }

    void CurrentState::FRfootCallback(const geometry_msgs::WrenchStamped& msg)
    {
        lowState.eeForce[0].x = msg.wrench.force.x;
        lowState.eeForce[0].y = msg.wrench.force.y;
        lowState.eeForce[0].z = msg.wrench.force.z;
        lowState.footForce[0] = msg.wrench.force.z;
        
    }

    void CurrentState::FLfootCallback(const geometry_msgs::WrenchStamped& msg)
    {
        lowState.eeForce[1].x = msg.wrench.force.x;
        lowState.eeForce[1].y = msg.wrench.force.y;
        lowState.eeForce[1].z = msg.wrench.force.z;
        lowState.footForce[1] = msg.wrench.force.z;
    }

    void CurrentState::RRfootCallback(const geometry_msgs::WrenchStamped& msg)
    {
        lowState.eeForce[2].x = msg.wrench.force.x;
        lowState.eeForce[2].y = msg.wrench.force.y;
        lowState.eeForce[2].z = msg.wrench.force.z;
        lowState.footForce[2] = msg.wrench.force.z;
    }

    void CurrentState::RLfootCallback(const geometry_msgs::WrenchStamped& msg)
    {
        lowState.eeForce[3].x = msg.wrench.force.x;
        lowState.eeForce[3].y = msg.wrench.force.y;
        lowState.eeForce[3].z = msg.wrench.force.z;
        lowState.footForce[3] = msg.wrench.force.z;
    }

   

    