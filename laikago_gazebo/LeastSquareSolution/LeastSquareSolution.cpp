#include "LeastSquareSolution.hpp"
#include <iostream>

using namespace std;

LeastSquareSolution::LeastSquareSolution(){
    //std::cout << "init least square sol \n";
    mass = 41;
    inertia = 0.01;

    // Model and World parameters
    Ig.resize(3, 3);
    Ig.setIdentity();
    gravity.resize(3,1);

    maxNormalForces_feet.resize(4,1);
    minNormalForces_feet.resize(4,1);

    direction_normal_flatGround.resize(3,1);
    direction_tangential_flatGround.resize(3,1);

    contact_state.resize(4,1);
    contact_state << 1, 1, 1, 1;

    // kinematics actual
    //std::cout << "init actual kinematics \n";
    x_COM_world.resize(3,1);
    xdot_COM_world.resize(3,1);
    omega_b_world.resize(3,1);
    quat_b_world.resize(4,1);
    R_b_world.resize(3,3);
    p_feet.resize(3,4);

    R_yaw_act.resize(3,3);
    R_yaw_act.setIdentity();

    // desired kinematics
    //std::cout << "init desired kinematics \n";
    x_COM_world_desired.resize(3,1);   
    xdot_COM_world_desired.resize(3, 1);
    xddot_COM_world_desired.resize(3, 1);
    omega_b_world_desired.resize(3, 1);
    omegadot_b_world_desired.resize(3, 1);
    R_b_world_desired.resize(3, 3);
    orientation_error.resize(3, 1);

    error_x_rotated.setZero(3);
    error_dx_rotated.setZero(3);
    error_theta_rotated.setZero(3);
    error_dtheta_rotated.setZero(3);

    // temporary, internal matrix
    //std::cout << "init temporary matrix \n";
    tempSkewMatrix3.resize(3, 3);
    tempVector3.resize(3, 1);

    tempSkewMatrix3.setZero();
    tempVector3.setZero();

    A_control.resize(18, 12);
    //A_1_control.resize(18, 12);
    b_control.resize(18, 1);
    //b_1_control.resize(18,1);

    //std::cout << "world data \n";
    set_worldData();

    //std::cout << "setup parameters \n";
    x_COM_world_desired << 0.0, 0.0, 0.0;
    xdot_COM_world_desired << 0, 0, 0;
    omega_b_world_desired << 0, 0, 0;
    R_b_world_desired << 1, 1, 1;

    Kp_COMx = 0;
    Kp_COMy = 0;
    Kp_COMz = 0;

    Kd_COMx = 0;
    Kd_COMy = 0;
    Kd_COMz = 0;

    Kp_Base_roll = 0;
    Kp_Base_pitch = 0;
    Kp_Base_yaw = 0;

    Kd_Base_roll = 0;
    Kd_Base_pitch = 0;
    Kd_Base_yaw = 0;

    yaw_act = 0;
}

LeastSquareSolution::~LeastSquareSolution(){}

/* ------------------------ Set Parameters --------------------------*/

void LeastSquareSolution::set_desiredTrajectoryData(double* rpy_des_in, double* p_des_in,
                                                    double* omegab_des_in, double* v_des_in){

    x_COM_world_desired << p_des_in[0], p_des_in[1], p_des_in[2];
    rpyToR(R_b_world_desired, rpy_des_in);
    omega_b_world_desired << omegab_des_in[0], omegab_des_in[1], omegab_des_in[2];
    xdot_COM_world_desired << v_des_in[0], v_des_in[1], v_des_in[2];
    //std::cout << "rpy_des: " << std::endl;
    //std::cout << rpy_des_in[0] << " " <<  rpy_des_in[1] << " " << rpy_des_in[2] << std::endl;
    //std::cout << "R_b_world_des: " << std::endl;
    //std::cout << R_b_world_desired << std::endl;
    //xddot_COM_world_desired << vdot_des_in[0], vdot_des_in[1], vdot_des_in[2];
}

void LeastSquareSolution::set_PDgains(double* Kp_COM_in, double* Kd_COM_in,
                                      double* Kp_Base_in, double* Kd_Base_in){
    Kp_COMx = Kp_COM_in[0];
    Kp_COMy = Kp_COM_in[1];
    Kp_COMz = Kp_COM_in[2];

    Kd_COMx = Kd_COM_in[0];
    Kd_COMy = Kd_COM_in[1];
    Kd_COMz = Kd_COM_in[2];

    Kp_Base_roll = Kp_Base_in[0];
    Kp_Base_pitch = Kp_Base_in[1];
    Kp_Base_yaw = Kp_Base_in[2];

    Kd_Base_roll = Kd_Base_in[0];
    Kd_Base_pitch = Kd_Base_in[1];
    Kd_Base_yaw = Kd_Base_in[2];
}

void LeastSquareSolution::set_mass(double mass_in) {mass = mass_in;}

void LeastSquareSolution::set_friction(double mu_in) {mu_friction = mu_in;}

void LeastSquareSolution::set_alpha_control(double alpha_control_in) {alpha_control = alpha_control_in;}

/* ------------------------ Primary Interface -----------------------*/

void LeastSquareSolution::updateProblemData(double* xfb_in, double* p_feet_in, double* p_des,
                                            double* p_act, double* v_des, double* v_act,
                                            double* O_err, double yaw_act_in) {
    // Unpack inputs
    copy_Array_to_Eigen(quat_b_world, xfb_in, 4, 0);
    copy_Array_to_Eigen(x_COM_world, xfb_in, 3, 4);
    copy_Array_to_Eigen(omega_b_world, xfb_in, 3, 7);
    copy_Array_to_Eigen(xdot_COM_world, xfb_in, 3, 10);
    copy_Array_to_Eigen(p_feet,p_feet_in, 12, 0);
   // std::cout << quat_b_world << std::endl;
    yaw_act = yaw_act_in;

    R_yaw_act.setZero();

    R_yaw_act(0, 0) = cos(yaw_act);
    R_yaw_act(0, 1) = -sin(yaw_act);
    R_yaw_act(1, 0) = sin(yaw_act);
    R_yaw_act(1, 1) = cos(yaw_act);
    R_yaw_act(2, 2) = 1;
    //std::cout << "output2" << std::endl;
    //std::cout << p_feet << std::endl;

    
    //std::cout << "quat to R: " << std::endl;
    // std::cout << R_b_world << std::endl;

    // Compute controller matrices. Must call these before calculating H_qp and
    // g_qp
    quaternion_to_R(R_b_world, quat_b_world);
    
    calc_PDcontrol();
    update_A_control();
    
    std::cout << "problem data updated \n";
}

void LeastSquareSolution::SetContactData(double* contact_state_in, double* min_force_in,
                                         double*max_force_in){
    copy_Array_to_Eigen(contact_state, contact_state_in, 4, 0);
    copy_Array_to_Eigen(minNormalForces_feet, min_force_in, 4, 0);
    copy_Array_to_Eigen(maxNormalForces_feet, max_force_in, 4, 0);
}

/* ------------------------ Math -----------------------------*/

void LeastSquareSolution::solveLeastSquare(double *xOpt){
    Eigen::VectorXd sol;
    sol.resize(12,1);
    sol.setZero();
    //std::cout << "solve" << std::endl;
    //std::cout << "ATA \n" << A_1_control.transpose() * A_1_control << std::endl;;
    //std::cout << "inverse \n" << (A_1_control.transpose() * A_1_control).inverse() << std::endl;
    //std::cout << "times A tran \n" << (A_1_control.transpose() * A_1_control).inverse() * A_1_control.transpose() << std::endl;
    //std::cout << "result \n" <<(A_1_control.transpose() * A_1_control).inverse() * A_1_control.transpose() * b_1_control << std::endl;
    sol = (A_control.transpose() * A_control).inverse() * A_control.transpose() * b_control;

    std::cout << "det A: " << (A_control.transpose() * A_control).determinant() << std::endl;
    for(int i = 0; i < 12; i++){
        xOpt[i] = -sol(i);
    }
    //std::cout << "firction: " << mu_friction << std::endl; 
    
    for(int i = 0; i < 4; i++){
        if(sol(3*i+2) < minNormalForces_feet(i)){
            xOpt[3*i+2] = -minNormalForces_feet(i);
        }
        if(sol(3*i+2) > maxNormalForces_feet(i)){
            xOpt[3*i+2] = -maxNormalForces_feet(i);
        }
        if(sol(3*i) < xOpt[3*i+2] * mu_friction){
            xOpt[3*i] = xOpt[3*i+2] * mu_friction;
        }
        if(sol(3*i) > -xOpt[3*i+2] * mu_friction){
            xOpt[3*i] = -xOpt[3*i+2] * mu_friction;
        }
        if(sol(3*i+1) < xOpt[3*i+2] * mu_friction){
             xOpt[3*i+1] = xOpt[3*i+2] * mu_friction;
        }
        if(sol(3*i+1) > -xOpt[3*i+2] * mu_friction){
             xOpt[3*i+1] = -xOpt[3*i+2] * mu_friction;
        }
    }
    
    

    std::cout << sol << std::endl;
}

void LeastSquareSolution::calc_PDcontrol(){
    // calcuate error in yaw rotated coordinates
    
    error_x_rotated = R_yaw_act.transpose() * (x_COM_world_desired - x_COM_world);
    error_dx_rotated = 
        R_yaw_act.transpose() * (xdot_COM_world_desired - xdot_COM_world);
    matrixLogRot(R_yaw_act.transpose() * R_b_world_desired * 
                 R_b_world.transpose() * R_yaw_act, orientation_error);
    error_dtheta_rotated =
        R_yaw_act.transpose() * (omega_b_world_desired - omega_b_world);
    //std::cout << "mass: " << mass << std::endl;
    //std::cout << error_x_rotated[2] << std::endl;
    // translational acceleration
    xddot_COM_world_desired(0) = 
        Kp_COMx * error_x_rotated(0) + Kd_COMx * error_dx_rotated(0);
    xddot_COM_world_desired(1) = 
        Kp_COMy * error_x_rotated(1) + Kd_COMy * error_dx_rotated(1);
    xddot_COM_world_desired(2) = 
        Kp_COMz * error_x_rotated(2) + Kd_COMz * error_dx_rotated(2);

    omegadot_b_world_desired(0) = Kp_Base_roll * orientation_error(0) +
                                  Kd_Base_roll * error_dtheta_rotated(0);
    omegadot_b_world_desired(1) = Kp_Base_pitch * orientation_error(1) +
                                  Kd_Base_pitch * error_dtheta_rotated(1);
    omegadot_b_world_desired(2) = Kp_Base_yaw * orientation_error(2) +
                                  Kd_Base_yaw * error_dtheta_rotated(2);
    
    Ig << .033, 0, 0, 0, 0.161, 0, 0, 0, 0.174; // inertia, can have an setup function later
   
    MatrixXd II = R_yaw_act.transpose() * R_b_world * Ig * R_b_world.transpose() *
                R_yaw_act;
    MatrixXd II_1;
    II_1.resize(12,1);
    II_1.setZero();

     // See RHS of Equation (5), [R1]
    b_control << mass * (xddot_COM_world_desired + gravity), 
     II * omegadot_b_world_desired, II_1;

    //std::cout << "p_feet after II" << std::endl;
    //std::cout << p_feet << std::endl;
    //std::cout << "calculate b_1 matrix" << std::endl;
    
    //std::cout << "error_dz =" << error_dx_rotated(2) << "\n";
    //std:: cout << "error_z =" << error_x_rotated(2) << "\n";
    //std::cout << "b_control: " << b_control << std::endl;
    //std::cout << "orientation_error = " << orientation_error << "\n";
}

void LeastSquareSolution::update_A_control(){
    std::cout << "update A_contro \n";
    // Update the A matrix in the controller notation A*f = b
    for (int i = 0; i < 4; i++) {
        A_control.block<3, 3>(0, 3*i) << R_yaw_act.transpose();

        tempVector3 <<  contact_state(i) * p_feet.col(i);
        std::cout << "p_feet \n";
        std::cout << p_feet.col(i) << std::endl;;
        crossMatrix(tempSkewMatrix3, tempVector3);
        A_control.block<3, 3>(3, 3*i) << R_yaw_act.transpose() * tempSkewMatrix3;
    }
    
    MatrixXd identity;
    identity.resize(12,12);
    identity.setIdentity();
    A_control.block<12,12>(6,0)= alpha_control * identity;
    //A_1_control.block<6,6>(6,6).setZero();
    std::cout << "A_control: " << A_control << std::endl;
  
}

/* ------------------------ Utility -----------------------------*/
void LeastSquareSolution::set_worldData(){
    direction_normal_flatGround << 0, 0, 1;
    gravity << 0, 0, 9.81;
    direction_tangential_flatGround << 0.7071, 0.7071, 0;
}

void LeastSquareSolution::copy_Array_to_Eigen(Eigen::VectorXd& target, double* source, 
                                              int len, int startIndex) {
    for (int i = 0; i < len; i++){
        target(i) = source[i + startIndex];
    }                                              
}

void LeastSquareSolution::copy_Array_to_Eigen(Eigen::MatrixXd& target, double* source, 
                                              int len, int startIndex){
    for (int i = 0; i < len; i++){
        target(i) = source[i + startIndex];
    }
}

void LeastSquareSolution::matrixLogRot(const Eigen::MatrixXd& R, 
                                       Eigen::VectorXd& omega) {
    double theta;
    double tmp = (R(0, 0) + R(1, 1) + R(2, 2) - 1) / 2;
    if (tmp >= 1.) {
        theta = 0;
    } else if (tmp <= -1.) {
        theta = M_PI;
    } else {
        theta = acos(tmp);
    }

    // Matrix3F omegaHat = (R-R.transpose())/(2 * sin(theta));
    // crossExtract(omegaHat,omega);
    omega << R(2, 1) - R(1, 2), R(0, 2) - R(2, 0), R(1, 0) - R(0, 1);
    if (theta > 10e-5) {
        omega *= theta / (2 * sin(theta));
    } else {
        omega /= 2;
    }
}
void LeastSquareSolution::crossMatrix(Eigen::MatrixXd& R, const Eigen::VectorXd& omega){
    R(0, 1) = -omega(2);
    R(0, 2) = omega(1);
    R(1, 0) = omega(2);
    R(1, 2) = -omega(0);
    R(2, 0) = -omega(1);
    R(2, 1) = omega(0);
}
 
void LeastSquareSolution::quaternion_to_R(Eigen::MatrixXd& R, 
                                                       Eigen::VectorXd& quat) {
    // wikipedia
    R(0, 0) = 1 - 2 * quat(2) * quat(2) - 2 * quat(3) * quat(3);
    R(0, 1) = 2 * quat(1) * quat(2) - 2 * quat(0) * quat(3);
    R(0, 2) = 2 * quat(1) * quat(3) + 2 * quat(0) * quat(2);
    R(1, 0) = 2 * quat(1) * quat(2) + 2 * quat(0) * quat(3);
    R(1, 1) = 1 - 2 * quat(1) * quat(1) - 2 * quat(3) * quat(3);
    R(1, 2) = 2 * quat(2) * quat(3) - 2 * quat(1) * quat(0);
    R(2, 0) = 2 * quat(1) * quat(3) - 2 * quat(2) * quat(0);
    R(2, 1) = 2 * quat(2) * quat(3) + 2 * quat(1) * quat(0);
    R(2, 2) = 1 - 2 * quat(1) * quat(1) - 2 * quat(2) * quat(2);                             
} 

void LeastSquareSolution::rpyToR(Eigen::MatrixXd& R, double* rpy_in){
  Eigen::Matrix3d Rz, Ry, Rx;

  Rz.setIdentity();
  Ry.setIdentity();
  Rx.setIdentity();

  Rz(0, 0) = cos(rpy_in[2]);
  Rz(0, 1) = -sin(rpy_in[2]);
  Rz(1, 0) = sin(rpy_in[2]);
  Rz(1, 1) = cos(rpy_in[2]);

  Ry(0, 0) = cos(rpy_in[1]);
  Ry(0, 2) = sin(rpy_in[1]);
  Ry(2, 0) = -sin(rpy_in[1]);
  Ry(2, 2) = cos(rpy_in[1]);

  Rx(1, 1) = cos(rpy_in[0]);
  Rx(1, 2) = -sin(rpy_in[0]);
  Rx(2, 1) = sin(rpy_in[0]);
  Rx(2, 2) = cos(rpy_in[0]);

  R = Rz * Ry * Rx;
}