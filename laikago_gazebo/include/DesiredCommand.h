/*!
 * @file DesiredCommand.h
 * @brief convert keyborad/gamepad command into desired
 * tracjectory for the robot
 */ 

#ifndef DESIREDCOMMAND_H
#define DESIREDCOMMAND_H

//#include "ros/ros.h"
#include "cppTypes.h"
//#include "body.h"
#include <signal.h>
#include <stdio.h>
#include <termios.h>
#include "StateEstimatorContainer.h"
#include "teleCmd.h"

#define KEYCODE_UP    0x41
#define KEYCODE_DOWN  0x42
#define KEYCODE_LEFT  0x44
#define KEYCODE_RIGHT 0x43

struct DesiredStateData{

    DesiredStateData() { zero(); }

    // Zero all data
    void zero();

    // Instataneous desired state comman
    Vec12<double> stateDes;
    Vec12<double> pre_stateDes;
    
    // Desired future state trajector (up to 10 time step for MPC)
    Eigen::Matrix<double, 12 ,10> stateTrajDes;
};

class DesiredStateCommand {
  public:
    // Initialize
    DesiredStateCommand(teleCmd* cmd, StateEstimate* _estimate, double _dt){
      keyCmd = cmd;
      stateEstimate = _estimate;
      dt = _dt;
    }
    void convertToStateCommands();
    void setStateCommands(double r, double p, Vec3<double> v_des, double yaw_rate);
    void desiredStateTrajectory(int N, Vec10<double> dtVec);
    double deadband(double command, double minVal, double maxVal);
    // These should come from the inferface
    double maxRoll = 0.4;
    double minRoll = -0.4;
    double maxPitch = 0.4;
    double minPitch = -0.4;
    double maxVelX = 2.0;
    double minVelX = -2.0;
    //double maxVelX = 5.0;
    //double minVelX = -5.0;
    double maxVelY = 0.5;
    double minVelY = -0.5;
    double maxTurnRate = 2.0;
    double minTurnRate = -2.0;
    DesiredStateData data;

    //~DesiredStateCommand();
  private:
    teleCmd* keyCmd;
    StateEstimate* stateEstimate;

    Mat12<double> A; // dynamics matrix for discrete time approximation

    double dt; // Control loop time step
};



#endif