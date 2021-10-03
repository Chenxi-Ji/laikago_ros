#include "../include/DesiredCommand.h"

void DesiredStateData::zero(){
    pre_stateDes = Vec12<double>::Zero();
    stateDes = Vec12<double>::Zero();
    stateTrajDes = Eigen::Matrix<double, 12, 10>::Zero();
}

void DesiredStateCommand::convertToStateCommands(){
    data.zero();

    static bool b_firstVisit(true);
    if(b_firstVisit){
    data.pre_stateDes(0) = stateEstimate->position(0);
    data.pre_stateDes(1) = stateEstimate->position(1);
    data.pre_stateDes(5) = stateEstimate->rpy(2);
    b_firstVisit = false;
    }

    double vxCommand = keyCmd->getVxCommand();
    double vyCommand = keyCmd->getYyCommand();

    // forward linear speed
    data.stateDes(6) = deadband(vxCommand, minVelX, maxVelX);

    // lateral linear speed
    data.stateDes(7) = deadband(vyCommand, minVelY, maxVelY);
    // Vertial linear speed
    data.stateDes(8) = 0.0;

    // X position
    data.stateDes(0) = stateEstimate->position(0) + dt * data.stateDes(6);
    //data.stateDes(0) = data.pre_stateDes(0) + dt * data.stateDes(6);

    // Y position
    data.stateDes(1) = stateEstimate->position(1) + dt * data.stateDes(7);
    //data.stateDes(1) = data.pre_stateDes(1) + dt * data.stateDes(7);
  
    // Z position height
    data.stateDes(2) = 0.45;

    // Roll rate
    data.stateDes(9) = 0.0;

    // Pitch rate
    data.stateDes(10) = 0.0;
     
    // Yaw turn rate
    //data.stateDes(11) =
    //  deadband(rightAnalogStick[0], minTurnRate, maxTurnRate);

    // Roll
    data.stateDes(3) = 0.0;

    // Pitch
    data.stateDes(4) = 0.0;
     // deadband(rightAnalogStick[1], minPitch, maxPitch);

    // Yaw
    //data.stateDes(5) = stateEstimate->rpy(2) + dt * data.stateDes(11);
    data.stateDes(5) = data.pre_stateDes(5) + dt * data.stateDes(11);


    data.pre_stateDes = data.stateDes;
}

void DesiredStateCommand::setStateCommands(double r, double p, Vec3<double> v_des, double yaw_rate){
    data.stateDes(6) = v_des[0];
    data.stateDes(7) = v_des[1];
    data.stateDes(8) = 0;

    data.stateDes(9) = 0;
    data.stateDes(10) = 0;
    data.stateDes(11) = yaw_rate;

    data.stateDes(0) = stateEstimate->position(0) + dt * data.stateDes(6);
    data.stateDes(1) = stateEstimate->position(1) + dt * data.stateDes(7);
    //data.stateDes(5) = stateEstimate->rpy(2) + dt * data.stateDes(11);
    data.stateDes(5) = data.pre_stateDes(5) + dt * data.stateDes(11);

    if(data.stateDes(5) > 3.141 && stateEstimate->rpy(2) < 0){
      data.stateDes(5) -= 6.28;//stateEstimate->rpy(2);
    }


     if(data.stateDes(5)  < -3.141 && stateEstimate->rpy(2) > 0){
      data.stateDes(5) += 6.28; //stateEstimate->rpy(2);
    }

    data.pre_stateDes(5) = data.stateDes(5);

     // Roll
    data.stateDes(3) = r;

    // Pitch
    data.stateDes(4) = p;
}

void DesiredStateCommand::desiredStateTrajectory(int N, Vec10<double> dtVec){
    A = Mat12<double>::Zero();
    A(0, 0) = 1;
    A(1, 1) = 1;
    A(2, 2) = 1;
    A(3, 3) = 1;
    A(4, 4) = 1;
    A(5, 5) = 1;
    A(6, 6) = 1;
    A(7, 7) = 1;
    A(8, 8) = 1; 
    A(9, 9) = 1;
    A(10, 10) = 1;
    A(11, 11) = 1;
    data.stateTrajDes.col(0) = data.stateDes;

    for (int k = 1; k < N; k++) {
    A(0, 6) = dtVec(k - 1);
    A(1, 7) = dtVec(k - 1);
    A(2, 8) = dtVec(k - 1);
    A(3, 9) = dtVec(k - 1);
    A(4, 10) = dtVec(k - 1);
    A(5, 11) = dtVec(k - 1);
    data.stateTrajDes.col(k) = A * data.stateTrajDes.col(k - 1);
    for (int i = 0; i < 12; i++) {
      // std::cout << data.stateTrajDes(i, k) << " ";
    }
    // std::cout << std::endl;
  }
}

double DesiredStateCommand::deadband(double command, double minVal, double maxVal){
    return (command/2) *(maxVal-minVal);
}