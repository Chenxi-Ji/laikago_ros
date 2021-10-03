#ifndef LEASTSQUARESOLUTION_H
#define LEASTSQUARESOLUTION_H
/*
    optimize AF=b with least square solution method
    A1 = [A, I]^T and b1 = [b, 0]^T
    F = (A1^T * A1)^(-1)*A1^R * b1 
*/

#include "ros/ros.h"
#include <eigen3/Eigen/Dense>
#include "../include/Utilities/pseudoInverse.h"

using namespace Eigen;

class LeastSquareSolution {
  public:
    LeastSquareSolution();
    ~LeastSquareSolution();

    // use new kinematics measurements to update AF = b
    void updateProblemData(double* xfb_in, double* p_feet_in, double* p_des,
                           double* p_act, double* v_des, double* v_act,
                           double* O_err, double yaw_act_in);

    void SetContactData(double* contact_state_in, double* min_force_in,
                        double* max_force_in);

    // set desired COM and orientation
    void set_desiredTrajectoryData(double* rpy_des_in, double* p_des_in,
                                   double* omegab_des_in, double* v_des_in);

    void set_PDgains(double* Kp_COM_in, double* Kd_COM_in, double* Kp_Base_in,
                     double* Kd_Base_in);

    void set_worldData();
     
    void set_mass(double mass_in);
    void set_alpha_control(double alpha_control_in);
    void set_friction(double mu_in);
    // calculate and return solution
    void solveLeastSquare(double* xOpt);

  private:
    Eigen::MatrixXd A_control; // A_1 Matrix
    Eigen::VectorXd b_control; // b_1 Matrix

    double alpha_control;

    // Centroidal control PD gains & variables
    double Kp_COMx, Kp_COMy, Kp_COMz;
    double Kd_COMx, Kd_COMy, Kd_COMz;

    double Kp_Base_roll, Kp_Base_pitch, Kp_Base_yaw;
    double Kd_Base_roll, Kd_Base_pitch, Kd_Base_yaw;

    // Model and Wolrd paramters
    double mass;
    double inertia;
    Eigen::MatrixXd Ig;

    Eigen::VectorXd gravity;

    Eigen::VectorXd minNormalForces_feet;
    Eigen::VectorXd maxNormalForces_feet;

    Eigen::VectorXd direction_normal_flatGround;
    Eigen::VectorXd direction_tangential_flatGround;

    // contact
    Eigen::VectorXd contact_state;
    double mu_friction;

    double yaw_act;

    /* Actual Kinematics*/
    Eigen::VectorXd x_COM_world;
    Eigen::VectorXd xdot_COM_world;
    Eigen::VectorXd omega_b_world;
    Eigen::VectorXd quat_b_world;
    Eigen::MatrixXd R_b_world;
    Eigen::MatrixXd p_feet;

    Eigen::MatrixXd R_yaw_act;

    /* Desired Kinematics */
    Eigen::VectorXd x_COM_world_desired;
    Eigen::VectorXd xdot_COM_world_desired;
    Eigen::VectorXd xddot_COM_world_desired;
    Eigen::VectorXd omega_b_world_desired;
    Eigen::VectorXd omegadot_b_world_desired;
    Eigen::MatrixXd R_b_world_desired;

    // Error coordinates
    Eigen::VectorXd error_x_rotated;
    Eigen::VectorXd error_dx_rotated;
    Eigen::VectorXd error_theta_rotated;
    Eigen::VectorXd error_dtheta_rotated;

    Eigen::VectorXd orientation_error;

    /* Temporary, Internal Matrices */
    Eigen::MatrixXd tempSkewMatrix3;
    Eigen::VectorXd tempVector3;

    // configure A_control, A_1_control, b_control, B_1_control
    void update_A_control(); // calculate A & A_1
    void calc_PDcontrol(); // calcualte b & b_1

    
    // utility
    void copy_Array_to_Eigen(Eigen::VectorXd& target, double* source, int len,
                             int startIndex);
    void copy_Array_to_Eigen(Eigen::MatrixXd& target, double* source, int len,
                             int startIndex);
    //void copy_Eigen_to_double(double* target, Eigen::VectorXd& source,
      //                      int length);

    void matrixLogRot(const Eigen::MatrixXd& R, Eigen::VectorXd& omega);
    void crossMatrix(Eigen::MatrixXd& R, const Eigen::VectorXd& omega);
  
    void quaternion_to_R(Eigen::MatrixXd& R, Eigen::VectorXd& quat);
    void rpyToR(Eigen::MatrixXd& R, double* rpy_in);
};

#endif
