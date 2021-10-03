#include <stdio.h>
#include <string.h>
#include <fstream> 
#include <iostream> 

#include "../include/FSM.h"
#include "ros/ros.h"
#include "../include/Utilities/Timer.h"


FSM_State::FSM_State(ControlFSMData* _controlFSMData):
_data(_controlFSMData),
Cmpc(0.001, 30) 
{
    std::cout<< "zero transition data" << std::endl;
    transitionData.zero();
    std::cout << "Initialize FSM" << std::endl;
}

void FSM_State::mpcBoundingTest(){
  ros::Rate rate(1000);
  // reset forces and steps to 0
  footFeedForwardForces = Mat34<double>::Zero();
  footstepLocations = Mat34<double>::Zero();
  double minForce = 5;
  double maxForce = 500;
  double contactStateScheduled[4] = {1, 1, 1, 1};

  double minForces[4] = {minForce, minForce, minForce, minForce};
  double maxForces[4] = {maxForce, maxForce, maxForce, maxForce};

  //Timer time;
  //time.start();
  //double COM_weights_stance[3] = {50, 50, 50};
  //double Base_weights_stance[3] = {500, 150, 30};// for QP locomotion
  double COM_weights_stance[3] = {5, 5, 10};
  double Base_weights_stance[3] = {10, 10, 20};
  double pFeet[12], p_act[3], v_act[3], O_err[3], rpy[3],v_des[3], p_des[3],
      omegaDes[3];
  double se_xfb[13];
  double kpCOM[3], kdCOM[3], kpBase[3], kdBase[3];
  double b_control[6];
  int counter = 0;
   ros::spinOnce();
  _data->_legController->updateData();
  _data->_stateEstimator->run();
  // se_xfb[3] = 1.0; 
  //  int during = 10;
  // position & rpy desired
  for(int i = 0; i < 3; i++){
    p_des[i] = _data->_stateEstimator->getResult().position(i); 
    v_des[i] = 0;
    omegaDes[i] = 0;
    rpy[i] = 0;
  }
    rpy[2] = _data->_stateEstimator->getResult().rpy(2);
    //p_des[2] = 0.4; // standup height for AlienGo
    p_des[2] = 0.3; // standup height for A1

  // while(ros::ok()) {
  //   //std::cout << ros::Time::now() << std::endl;
  //   // rate.reset();
  //   _data->_legController->updateData();
  //   _data->_stateEstimator->run();

  //   if(counter > 1500){
  //   //std::cout << "start motion" << std::endl;
  //     _data->_legController->updateData();
  //     _data->_stateEstimator->run();
  //     Vec3<double> v_des(0, 0, 0);
  //     double yaw_rate = 0;
     

  //     _data->_desiredStateCommand->setStateCommands(0,0,v_des, yaw_rate);
  //     // mpcBounding.run(*_data);
      
    
  //     runQP = false;
  //   }
  //   if(runQP){
      for (int i = 0; i < 4; i++) {
        se_xfb[i] = _data->_stateEstimator->getResult().orientation(i);
      }
     
      for (int i = 0; i < 3; i++) {
        //rpy[i] = 0;
        //rpy[i] = _data->_stateEstimator->getResult().rpy(i);
        //p_des[i] = _data->_stateEstimator->getResult().position(i);
        p_act[i] = _data->_stateEstimator->getResult().position(i);
        
        v_act[i] = _data->_stateEstimator->getResult().vWorld(i);
        // v_des[i] = 0;
        //v_des[2] = 0.005;

        se_xfb[4 + i] = _data->_stateEstimator->getResult().position(i);
        se_xfb[7 + i] = _data->_stateEstimator->getResult().omegaWorld(i);
        se_xfb[10 + i] = _data->_stateEstimator->getResult().vWorld(i);
        
    // Set the translational and orientation gains
        kpCOM[i] = 30;   //_data->controlParameters->kpCOM(i);
        kdCOM[i] = 10;     //_data->controlParameters->kdCOM(i);
        kpBase[i] = 80;     //_data->controlParameters->kpBase(i);
        kdBase[i] = 50; //  _data->controlParameters->kdBase(i);
        }

      kpCOM[2] = 50;
      //kdCOM[2] = 10;

    //Vec3<double> pFeetVec;
    Vec3<double> pFeetVecCOM;

    // Get the foot locations relative to COM
      for (int leg = 0; leg < 4; leg++) {
        // computeLegJacobianAndPosition(&_data->_quadruped, _data->_legController->data[leg].q,
        //                           (Mat3<double>*)nullptr, &pFeetVec, leg);
        //pFeetVecCOM = _data->_stateEstimator->getResult().rBody.transpose() *
                  //(_data->_quadruped->getHipLocation(leg) + pFeetVec);

        pFeetVecCOM =  _data->_stateEstimator->getResult().rBody.transpose() *
        (_data->_quadruped->getHipLocation(leg) + _data->_legController->data[leg].p);


        pFeet[leg * 3] = pFeetVecCOM[0];
        pFeet[leg * 3 + 1] = pFeetVecCOM[1];
        pFeet[leg * 3 + 2] = pFeetVecCOM[2];
        //std::cout << "pFeet" << leg << std::endl;
      }
    //  myfile << "\n";
     // std::cout << j << std::endl;
     //std::cout << "run QP" << std::endl;
    balanceController.set_alpha_control(0.01);
    balanceController.set_friction(0.6);
    balanceController.set_mass(_data->_quadruped->mass);
    balanceController.set_wrench_weights(COM_weights_stance, Base_weights_stance);
    balanceController.set_PDgains(kpCOM, kdCOM, kpBase, kdBase);
    balanceController.set_desiredTrajectoryData(rpy, p_des, omegaDes, v_des);
    balanceController.SetContactData(contactStateScheduled, minForces, maxForces);
    balanceController.updateProblemData(se_xfb, pFeet, p_des, p_act, v_des, v_act,
                                      O_err, _data->_stateEstimator->getResult().rpy(2));
   // balanceController.print_QPData();
    double fOpt[12];
    balanceController.solveQP_nonThreaded(fOpt);

  //   for (int leg = 0; leg < 4; leg++) {
  //       footFeedForwardForces.col(leg) << fOpt[leg * 3], fOpt[leg * 3 + 1],
  //       fOpt[leg * 3 + 2]; // force in world frame, need to convert to body frame

  //       _data->_legController->commands[leg].feedforwardForce = footFeedForwardForces.col(leg);
       
  //     }
  
  //   }

  //   _data->_legController->updateCommand();
  //   rate.sleep();
  
  //   counter++;
  // }

}

void FSM_State::SprayDemo()
{

  ros::Rate rate(1000);
  // reset forces and steps to 0
  footFeedForwardForces = Mat34<double>::Zero();
  footstepLocations = Mat34<double>::Zero();
  double minForce = 5;
  double maxForce = 500;
  double contactStateScheduled[4] = {1, 1, 1, 1};

  double minForces[4] = {minForce, minForce, minForce, minForce};
  double maxForces[4] = {maxForce, maxForce, maxForce, maxForce};

  //Timer time;
  //time.start();
  //double COM_weights_stance[3] = {50, 50, 50};
  //double Base_weights_stance[3] = {500, 150, 30};// for QP locomotion
  double COM_weights_stance[3] = {10, 10, 10};
  double Base_weights_stance[3] = {100, 50, 20};
  double pFeet[12], p_act[3], v_act[3], O_err[3], rpy[3],v_des[3], p_des[3],
      omegaDes[3];
  double se_xfb[13];
  double kpCOM[3], kdCOM[3], kpBase[3], kdBase[3];
  double b_control[6];
  int counter = 0;
  // ros::spinOnce();
  _data->_legController->updateData();
  _data->_stateEstimator->run();
  // se_xfb[3] = 1.0; 
  //  int during = 10;
  // position & rpy desired
  for(int i = 0; i < 3; i++){
    p_des[i] = _data->_stateEstimator->getResult().position(i); 
    v_des[i] = 0;
    omegaDes[i] = 0;
    rpy[i] = 0;
  }
    rpy[2] = _data->_stateEstimator->getResult().rpy(2);
    p_des[2] = 0.4; // standup height for AlienGo
    // p_des[2] = 0.28; // standup height for A1

  while(ros::ok()) {
    //std::cout << ros::Time::now() << std::endl;
    // rate.reset();
    _data->_legController->updateData();
    _data->_stateEstimator->run();

    if(counter > 1500){
      // rpy[1] = -0.5;
      // p_des[1] = 0.15;
    }

    if(counter > 1800){
      // p_des[2] = 0.3;
      // rpy[2] = -1;
    }

    if(counter > 2000){
    //  rpy[2] = -0.5;
    //  p_des[1] = 0;
    }



    //  if(counter > 10000){
    //  p_des[1] = -0.15;
    // }


    if(counter > 8000){
     rpy[1] = 0.8;
      rpy[2] = 0;
    }

    if(counter > 10000){
      rpy[1] = 1;
    }

    
    if(counter > 12000){
      rpy[1] = -1;
      //rpy[2] = -0.2;
    }


    if(counter > 10000){
      rpy[1] = 0;
      
    }
    if(runQP){
      for (int i = 0; i < 4; i++) {
        se_xfb[i] = _data->_stateEstimator->getResult().orientation(i);
      }
     
      for (int i = 0; i < 3; i++) {
        //rpy[i] = 0;
        //rpy[i] = _data->_stateEstimator->getResult().rpy(i);
        //p_des[i] = _data->_stateEstimator->getResult().position(i);
        p_act[i] = _data->_stateEstimator->getResult().position(i);
        
        v_act[i] = _data->_stateEstimator->getResult().vWorld(i);
        // v_des[i] = 0;
        //v_des[2] = 0.005;

        se_xfb[4 + i] = _data->_stateEstimator->getResult().position(i);
        se_xfb[7 + i] = _data->_stateEstimator->getResult().omegaWorld(i);
        se_xfb[10 + i] = _data->_stateEstimator->getResult().vBody(i);
        
    // Set the translational and orientation gains
        kpCOM[i] = 30;   //_data->controlParameters->kpCOM(i);
        kdCOM[i] = 10;     //_data->controlParameters->kdCOM(i);
        kpBase[i] = 100;     //_data->controlParameters->kpBase(i);
        kdBase[i] = 30; //  _data->controlParameters->kdBase(i);
        }

      kdCOM[0] = 10;
      kpCOM[0] = 20;
      kpCOM[2] = 50;
      kpBase[0] = 400;
      kpBase[1] = 200;

      // std::cout << "x_pos " << p_act[0] << std::endl;
      // std::cout << "y_pos " << p_act[1] << std::endl;
      // std::cout << "z_pos " << p_act[2] << std::endl;

    //Vec3<double> pFeetVec;
    Vec3<double> pFeetVecCOM;

    // Get the foot locations relative to COM
      for (int leg = 0; leg < 4; leg++) {
        // computeLegJacobianAndPosition(&_data->_quadruped, _data->_legController->data[leg].q,
        //                           (Mat3<double>*)nullptr, &pFeetVec, leg);
        //pFeetVecCOM = _data->_stateEstimator->getResult().rBody.transpose() *
                  //(_data->_quadruped->getHipLocation(leg) + pFeetVec);

        pFeetVecCOM =  _data->_stateEstimator->getResult().rBody.transpose() *
        (_data->_quadruped->getHipLocation(leg) + _data->_legController->data[leg].p);


        pFeet[leg * 3] = pFeetVecCOM[0];
        pFeet[leg * 3 + 1] = pFeetVecCOM[1];
        pFeet[leg * 3 + 2] = pFeetVecCOM[2];
        //std::cout << "pFeet" << leg << std::endl;
      }
    //  myfile << "\n";
     // std::cout << j << std::endl;
     //std::cout << "run QP" << std::endl;
    balanceController.set_alpha_control(0.01);
    balanceController.set_friction(0.6);
    balanceController.set_mass(_data->_quadruped->mass);
    balanceController.set_wrench_weights(COM_weights_stance, Base_weights_stance);
    balanceController.set_PDgains(kpCOM, kdCOM, kpBase, kdBase);
    balanceController.set_desiredTrajectoryData(rpy, p_des, omegaDes, v_des);
    balanceController.SetContactData(contactStateScheduled, minForces, maxForces);
    balanceController.updateProblemData(se_xfb, pFeet, p_des, p_act, v_des, v_act,
                                      O_err, _data->_stateEstimator->getResult().rpy(2));
   // balanceController.print_QPData();
    double fOpt[12];
    balanceController.solveQP_nonThreaded(fOpt);

    for (int leg = 0; leg < 4; leg++) {
        footFeedForwardForces.col(leg) << fOpt[leg * 3], fOpt[leg * 3 + 1],
        fOpt[leg * 3 + 2]; // force in world frame, need to convert to body frame

        _data->_legController->commands[leg].feedforwardForce = footFeedForwardForces.col(leg);
       
      }
  
    }

    _data->_legController->updateCommand();
    rate.sleep();
  
    counter++;
  }

}

class Instruction
{
    public:
	double v_set[3];
	double w_set[3];
	void set(double v_des[3],double w_des[3]);
	void command(int option1, double option2);	
	void compute(double input[27]);
        Vec3<double> output_v();

	int m=6;
	int n=27;
	const double matrix_A[6][27] ={
	{-8.51243896e-01, -1.42241375e+00, -3.66831654e-01,
         9.79771819e-15,  2.06501483e-14, -1.09912079e-14,
         1.90344728e+00,  5.92264978e-01,  7.43575422e-01,
        -4.73847838e-01,  3.87169122e-01, -3.99990995e-02,
        -2.18239058e-01,  4.22413292e-02, -1.65953877e-01,
        -2.88394286e+00,  1.53004204e+00,  6.89431329e-01,
         4.27193312e+00, -1.92855725e+00, -5.88501319e-01,
        -4.30082236e-01, -5.08661662e-01, -2.10269181e-01,
        -3.71867542e-01,  4.39421244e-01,  6.92019360e-02},

        {0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
         0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
         0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
         0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
         0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
         0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
         0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
         0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
         0.00000000e+00,  0.00000000e+00,  0.00000000e+00},

        {0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
         0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
         0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
         0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
         0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
         0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
         0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
         0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
         0.00000000e+00,  0.00000000e+00,  0.00000000e+00},

       {-8.89352269e-01,  1.97237046e-01, -1.48758452e-01,
         3.71924713e-15,  1.06858966e-15, -1.94289029e-15,
        -1.24720116e+00,  3.48552008e-01,  2.60761886e-02,
        -5.25651874e-01,  1.30977074e-01, -4.18818629e-02,
        -3.68108685e-01, -1.55700477e-01, -7.44669518e-02,
        -1.42111343e-01,  1.00057887e+00, -2.21330193e-02,
         1.02210478e+00, -1.09578825e+00,  1.57488458e-01,
        -7.27172945e-01,  1.04905371e+00,  4.19241339e-02,
         2.51651727e-01, -9.50651921e-01, -1.64734022e-01},

       { 0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
         0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
         0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
         0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
         0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
         0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
         0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
         0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
         0.00000000e+00,  0.00000000e+00,  0.00000000e+00},

       {-1.13798306e+00, -1.37767682e+00, -2.10890785e-02,
        -9.43689571e-16,  0.00000000e+00,  1.88737914e-15,
        -3.33990730e-01,  8.77357770e-02,  2.46144689e-01,
         1.51984525e-01, -1.86572628e-01,  1.12335339e-02,
         2.57689671e-01,  1.52098365e-01, -5.09755913e-02,
         3.14134107e+00, -2.40411396e-01,  3.92969981e-01,
        -3.23885698e-01, -1.98132152e-01, -2.80759341e-01,
        -9.66015529e-01,  7.46752628e-01, -1.23725561e-01,
        -7.53548081e-01, -4.07612267e-01, -7.26849081e-03}};

    	const double vector_b[6] ={2.27156191,  0.0, 0.0, -1.14913446, 0.0, -1.76167327};
};

void Instruction::set(double v_des[3],double w_des[3])
{
    for(int i=0;i<3;i++)
    {
    	v_set[i]=v_des[i];
    	w_set[i]=w_des[i];
    }

}

void Instruction::command(int option1, double option2) //1:walk x, 2:turn, 3:stance 
{
    switch(option1)
    {
	case 1:
	    v_set[0]=option2;
	    break;
     	case 2:
	    w_set[0]=option2;
	    break;
        case 3 :
	    v_set[0]=0;
	    break;
    }
}

void Instruction::compute(double input[27])
{
    double output[6]={0,0,0,0,0,0};
    double output1[3]={0,0,0};
    double output2[3]={0,0,0};

    for(int i=0;i<m;i++)
    {
	double temp=vector_b[i];
	for (int j=0;j<n;j++)
	{
	    temp+=matrix_A[i][j]*input[j];
	}
	output[i]=temp;
    }

    for(int i=0;i<3;i++)
    	output1[i]=output[i];
 
    for(int i=0;i<3;i++)
    	output2[i]=output[i+3];

	
}

Vec3<double> Instruction::output_v()
{
    Vec3<double> output_v{v_set[0],v_set[1],v_set[2]};
    return output_v;
}


void FSM_State::QPstand(){
    ofstream myfile;
    ofstream QP;
    ofstream z_pos;
    ofstream b_des;
    ofstream outfile;

    myfile.open ("ori.txt");
    QP.open("QPsolution.txt");
    z_pos.open("zPos.txt");
    b_des.open("b_des_z.txt");
    outfile.open("outfile.txt");

    ros::Rate rate(1000);
    // reset forces and steps to 0
    footFeedForwardForces = Mat34<double>::Zero();
    footstepLocations = Mat34<double>::Zero();
    double minForce = 5;
    double maxForce = 500;
    double contactStateScheduled[4] = {1, 1, 1, 1};

    double minForces[4] = {minForce, minForce, minForce, minForce};
    double maxForces[4] = {maxForce, maxForce, maxForce, maxForce};

    Vec4<double> contactphase(0.5,0.5,0.5,0.5); 
    _data->_stateEstimator->setContactPhase(contactphase);
    //Timer time;
    //time.start();
    //double COM_weights_stance[3] = {50, 50, 50};
    //double Base_weights_stance[3] = {500, 150, 30};// for QP locomotion
    double COM_weights_stance[3] = {5, 5, 10};
    double Base_weights_stance[3] = {10, 10, 20};
    double pFeet[12], p_act[3], v_act[3], O_err[3], rpy[3],v_des[3], p_des[3],
      omegaDes[3];
    double se_xfb[13];
    double kpCOM[3], kdCOM[3], kpBase[3], kdBase[3];
    double b_control[6];
    int counter = 0;
       // ros::spinOnce();
       _data->_legController->updateData();
       _data->_stateEstimator->run();
    // se_xfb[3] = 1.0; 
    //  int during = 10;
    // position & rpy desired
    for(int i = 0; i < 3; i++){
      //p_des[i] = _data->_stateEstimator->getResult().position(i); 
      v_des[i] = 0;
      omegaDes[i] = 0;
      rpy[i] = 0;
    }
       //rpy[2] = _data->_stateEstimator->getResult().rpy(2);
        p_des[2] = 0.4; // standup height for AlienGo
      // p_des[2] = 0.3; // standup height for A1
     //bool firstLocoRun = true;
     bool runMPC = true;
    //double v_des[3] = {0, 0, 0};


    while(ros::ok()) {
     //std::cout << ros::Time::now() << std::endl;
      // rate.reset();
       _data->_legController->updateData();
       _data->_stateEstimator->run();

    Instruction instruction;
    /*======================== MPC locomotion ====================*/
   if(counter>100){
     z_pos << _data->_stateEstimator->getResult().position[2] << " " <<  _data->_stateEstimator->getResult().position[1] << std::endl;
     //std::cout << "contact " << _data->_stateEstimator->getResult().contactEstimate << std::endl;
     //ROS_INFO("a");
     //std::cout << "start motion" << std::endl;
      //_data->_legController->updateData();
      //_data->_stateEstimator->run();
      Vec3<double> v_des(0, 0, 0); // v_body
      double yaw_rate = 0;
      double roll = 0;
      double pitch = 0;
      Cmpc.setGaitNum(2); 

      double v_input[3]={v_des[0],v_des[1],v_des[2]} ;
      double w_des[3]= {yaw_rate,roll,pitch};
      instruction.set(v_input,w_des);
      int gaitNum=3;
      // Cmpc.climb = 1;
    //   std::cout << "state estimate rpy ";
    //   for(int i = 0; i < 3; i++){
    //   std::cout << _data->_stateEstimator->getResult().rpy(i) << " " ; 
    //  }
      
      //if(counter > 500){ 
	//yaw_rate = 1;
	//roll=1;
	//pitch=1;
	//v_des[1]=0.3;
      //}
      
      
      if(counter > 300){
	//v_input[0]=1;
	//instruction.set(v_input,w_des);

      
	//if(counter%50==0)
	//{	

	//double input_1,input_2,input_3,input_4,input_5,input_6;
	
	ifstream infile("infile.txt", ios::in);
    	if (!infile.fail())
    	{
           if(!infile.eof())
           {
		for(int i=0;i<3;i++)
		{
		    string str5;
            	    infile >> str5;
            	    outfile << str5 <<" ";
	            v_input[i]=atof(str5.c_str());
		    //v_input[i]=0;
		}
		//v_input[0]=1;

		for(int i=0;i<3;i++)
		{
		    string str5;
            	    infile >> str5;
            	    outfile << str5 <<" ";
	            w_des[i]=atof(str5.c_str());
		    //w_des[i]=0;
		}
		infile >> str5;
            	outfile << str5 <<" ";
		gaitNum=atof(str5.c_str());
		
           }
	
	   
	   outfile<<v_input[0]<<" "<<v_input[1]<<" "<<v_input[2]<<" "<<w_des[0]<<" "<<w_des[1]<<" "<<w_des[2]<<" "<<gaitnum<<" "<<endl;
    	}
	infile.close();
	
	//}

	
	//int echo=counter/1000;
	//gaitnum=echo%6+1;

	printf("command: %f %f %f %f %f %f \n",v_input[0],v_input[1],v_input[2],w_des[0],w_des[1],w_des[2]);
	printf("GaitNum: %d \n",gaitnum);

	instruction.set(v_input,w_des);
	Cmpc.setGaitNum(gaitnum);

      }
      //if(counter > 1500){
	//v_input[0]=1
	//w_des[0]=-0.3;
	//instruction.set(v_input,w_des);
      //	instruction.command(2,-0.3);
      //}
      //if(counter > 5000){
	//instruction.command(2,-0.2);
	//instruction.command(1,0.7);
	//Cmpc.setGaitNum(6);
      //}

      if(counter > 15000){
        // Cmpc.setGaitNum(4);
      v_des[0] = 0;
      //yaw_rate = 0;
      // pitch = 0.3;
      }


	//  yaw_rate = 0.39;
  //     }

  //     if (counter > 23250 && counter < 24000) {
	//  std::cout<< "Rest4" << std::endl;
	//  yaw_rate = 0;
  //     }
  //     if (counter > 24000 && counter < 26000) {
	//  std::cout<< "Fifth leg" << std::endl;
  //        //pitch = -0.5;
  //        v_des[0] = sqrt(3);
  //        v_des[1] = -0.08;
	//  yaw_rate = 0;
  //     }

  //     if(counter > 26000 && counter < 27250){
  //       std::cout<< "Rotating" << std::endl;
  //       //pitch = -0.6;
  //       v_des[0] = 0;
	// v_des[1] = 0;
  //       yaw_rate = 0.52;
  //     }
      
  //     if (counter > 27250 && counter < 27750) {
	//  std::cout<< "Dec5" << std::endl;
  //        //pitch = -0.5;
    
	//  yaw_rate = 0.39;
  //     }

  //     if (counter > 27750 && counter < 30500) {
	//  std::cout<< "Rest5" << std::endl;
	//  yaw_rate = 0;
  //     }
  //     if (counter > 30500 && counter < 32500) {
	//  std::cout<< "sixth leg" << std::endl;
  //        //pitch = -0.5;
  //        v_des[0] = sqrt(3);
  //        v_des[1] = -0.08;
	//  yaw_rate = 0;
  //     }

      //_data->_desiredStateCommand->setStateCommands(roll, pitch, v_des, yaw_rate);
      _data->_desiredStateCommand->setStateCommands(instruction.w_set[1], instruction.w_set[2], instruction.output_v(), instruction.w_set[0]);
      Cmpc.run(*_data);
    
    //  for(int i = 0; i < 4; i++){
    //     std::cout << " leg " << i << ":  " << _data->_legController->commands[i].feedforwardForce[2] << std::endl;
    //   }
      for(int i = 0; i < 4; i++){
        QP << _data->_legController->commands[i].feedforwardForce[2] << " ";
      }
      QP << "\n";
      //runQP = false;
	runQP=false;
    }

  
        
    /* =================================================================*/
    if(runQP){
      for (int i = 0; i < 4; i++) {
        se_xfb[i] = _data->_stateEstimator->getResult().orientation(i);
      }
       // v_des[2] = 0.1;
      for (int i = 0; i < 3; i++) {
        //rpy[i] = 0;
        //rpy[i] = _data->_stateEstimator->getResult().rpy(i);
        //p_des[i] = _data->_stateEstimator->getResult().position(i);
        p_act[i] = _data->_stateEstimator->getResult().position(i);
        
        v_act[i] = _data->_stateEstimator->getResult().vWorld(i);
        // v_des[i] = 0;
        //v_des[2] = 0.005;

        se_xfb[4 + i] = _data->_stateEstimator->getResult().position(i);
        se_xfb[7 + i] = _data->_stateEstimator->getResult().omegaWorld(i);
        se_xfb[10 + i] = _data->_stateEstimator->getResult().vWorld(i);
        
    // Set the translational and orientation gains
        kpCOM[i] = 30;   //_data->controlParameters->kpCOM(i);
        kdCOM[i] = 10;     //_data->controlParameters->kdCOM(i);
        kpBase[i] = 80;     //_data->controlParameters->kpBase(i);
        kdBase[i] = 20; //  _data->controlParameters->kdBase(i);
        }

      kpCOM[2] = 50;
      kpBase[0] = 300;
      kpBase[1] = 200;

    //Vec3<double> pFeetVec;
    Vec3<double> pFeetVecCOM;

    // Get the foot locations relative to COM
      for (int leg = 0; leg < 4; leg++) {
        // computeLegJacobianAndPosition(&_data->_quadruped, _data->_legController->data[leg].q,
        //                           (Mat3<double>*)nullptr, &pFeetVec, leg);
        //pFeetVecCOM = _data->_stateEstimator->getResult().rBody.transpose() *
                  //(_data->_quadruped->getHipLocation(leg) + pFeetVec);

        pFeetVecCOM =  _data->_stateEstimator->getResult().rBody.transpose() *
        (_data->_quadruped->getHipLocation(leg) + _data->_legController->data[leg].p);

        pFeet[leg * 3] = pFeetVecCOM[0];
        pFeet[leg * 3 + 1] = pFeetVecCOM[1];
        pFeet[leg * 3 + 2] = pFeetVecCOM[2];
        //std::cout << "pFeet" << leg << std::endl;
      }
     _data->_stateEstimator->setContactPhase(contactphase);

    p_des[0] = p_act[0] + (pFeet[0] + pFeet[3] + pFeet[6] + pFeet[9]) / 4.0;
    p_des[1] = p_act[1] + (pFeet[1] + pFeet[4] + pFeet[7] + pFeet[10]) / 4.0;
    //  myfile << "\n";
     // std::cout << j << std::endl;
     //std::cout << "run QP" << std::endl;
    balanceController.set_alpha_control(0.01);
    balanceController.set_friction(0.2);
    balanceController.set_mass(_data->_quadruped->mass);
    balanceController.set_wrench_weights(COM_weights_stance, Base_weights_stance);
    balanceController.set_PDgains(kpCOM, kdCOM, kpBase, kdBase);
    balanceController.set_desiredTrajectoryData(rpy, p_des, omegaDes, v_des);
    balanceController.SetContactData(contactStateScheduled, minForces, maxForces);
    balanceController.updateProblemData(se_xfb, pFeet, p_des, p_act, v_des, v_act,
                                      O_err, _data->_stateEstimator->getResult().rpy(2));
   // balanceController.print_QPData();
    double fOpt[12];
    balanceController.solveQP_nonThreaded(fOpt);

    //std::cout << "zForce leg 0: " << fOpt[2] << std::endl;  //source code
    //balanceController.get_b_matrix(b_control);
    //b_des << b_control[2] << "\n";

  // Publish the results over ROS
  // balanceController.publish_data_lcm();

  // Copy the results to the feed forward forces
    
     //_data->_stateEstimator->run();


    for (int leg = 0; leg < 4; leg++) {
        footFeedForwardForces.col(leg) << fOpt[leg * 3], fOpt[leg * 3 + 1],
        fOpt[leg * 3 + 2]; // force in world frame, need to convert to body frame
        
        _data->_legController->commands[leg].feedforwardForce = footFeedForwardForces.col(leg);
        //_data->_stateEstimator->getResult().rBody.transpose() * footFeedForwardForces.col(leg); 
        // std::cout << _data->_legController->commands[leg].feedforwardForce[2]<< "\n";
    }
  
    //std::cout << j << std::endl;
    //QP << "\n";
    
    
    //_data->_legController->commands[3].feedforwardForce = _data->_legController->commands[2].feedforwardForce;
    //if(p_act[2] < 0.25){
     //  std::cout << "force" << std::endl;
    //std::cout << footFeedForwardForces << std::endl;
    //_data->_legController->updateCommand();

    // plots
    // if(counter>500){
    //  for(int i = 0; i < 4; i++){
    //    QP << _data->_legController->commands[i].feedforwardForce[2] << " ";
    //    //b_des << _data->_legController->commands[i].pDes[2] << " "; // foot pos
       
    //   }
    //   b_des << _data->_legController->commands[1].pDes[2] << " ";
    //   b_des << _data->_legController->data[1].p[2] << "\n ";
      
    //   for(int i =0; i<3; i++){
    //     //myfile << _data->_stateEstimator->getResult().rpy(i);
    //     //myfile << " " ;
    //      myfile << _data->_legController->commands[1].pDes[i] - _data->_legController->data[1].p[i] << " ";
    //   }
    //  myfile << "\n";
    //   z_pos << _data->_stateEstimator->getResult().position(2) << "\n";
    //  QP << "\n";
    //  //b_des << "\n";
    // }
    }

    _data->_legController->updateCommand();
    rate.sleep();
  
    counter++;
 
}
 
 b_des.close();
 z_pos.close();
 myfile.close();
 QP.close();
 outfile.close();
 std::cout << "stand up finished" << std::endl;
}



void FSM_State::runQPloco()
{
  ofstream foot_force;
  foot_force.open("foot_force.txt");

  ofstream pos;
  pos.open("COM_pos.txt");

  ofstream rpy_log;
  rpy_log.open("rpy.txt");

  ros::Rate rate(1000);
  // reset forces and steps to 0
  footFeedForwardForces = Mat34<double>::Zero();
  footstepLocations = Mat34<double>::Zero();
  double minForce = 5;
  double maxForce = 500;
  double contactStateScheduled[4] = {1, 1, 1, 1};

  double minForces[4] = {minForce, minForce, minForce, minForce};
  double maxForces[4] = {maxForce, maxForce, maxForce, maxForce};

     Vec4<double> contactphase(0.5,0.5,0.5,0.5); 
    _data->_stateEstimator->setContactPhase(contactphase);

  //Timer time;
  //time.start();
  //double COM_weights_stance[3] = {50, 50, 50};
  //double Base_weights_stance[3] = {500, 150, 30};// for QP locomotion
  double COM_weights_stance[3] = {10, 10, 20};
  double Base_weights_stance[3] = {50, 25, 20};
  double pFeet[12], p_act[3], v_act[3], O_err[3], rpy[3],v_des[3], p_des[3],
      omegaDes[3];
  double se_xfb[13];
  double kpCOM[3], kdCOM[3], kpBase[3], kdBase[3];
  double b_control[6];
  int counter = 0;
  // ros::spinOnce();
  _data->_legController->updateData();
  _data->_stateEstimator->run();
  // se_xfb[3] = 1.0; 
  //  int during = 10;
  // position & rpy desired
  for(int i = 0; i < 3; i++){
    p_des[i] = _data->_stateEstimator->getResult().position(i); 
    v_des[i] = 0;
    omegaDes[i] = 0;
    rpy[i] = 0;
  }
    rpy[2] = _data->_stateEstimator->getResult().rpy(2);
    p_des[2] = 0.3; // standup height for AlienGo
    // p_des[2] = 0.3; // standup height for A1

  while(ros::ok()) {
    //std::cout << ros::Time::now() << std::endl;
    // rate.reset();
    _data->_legController->updateData();
    _data->_stateEstimator->run();
   
    if(counter > 1500){
      Vec3<double> v_des(0, 0, 0);
      double yaw_rate = 0;
      if(counter > 3000){
       // yaw_rate = 0.5;
       v_des[0] = 0.1;
      }
      _data->_desiredStateCommand->setStateCommands(0, 0, v_des, yaw_rate);
      QPloco.climb = true;
      QPloco.run(*_data);
      runQP = false;
      for(int i = 0; i < 4; i++){
        foot_force << _data->_legController->commands[i].feedforwardForce[2] << " ";
      }

      for(int i = 0; i < 3; i++){
        rpy_log << _data->_stateEstimator->getResult().rpy(i) << " ";
      }
      rpy_log << "\n";

      pos << _data->_desiredStateCommand->data.stateDes(5) << " ";
      pos << _data->_stateEstimator->getResult().rpy(2) << std::endl;

      foot_force << "\n";

    }
    // if(counter > 15000 && counter < 19000){
    //   runQP = true;
    // }

    
  
    if(runQP){
      for (int i = 0; i < 4; i++) {
        se_xfb[i] = _data->_stateEstimator->getResult().orientation(i);
      }
     
      for (int i = 0; i < 3; i++) {
        //rpy[i] = 0;
        //rpy[i] = _data->_stateEstimator->getResult().rpy(i);
        //p_des[i] = _data->_stateEstimator->getResult().position(i);
        p_act[i] = _data->_stateEstimator->getResult().position(i);
        
        v_act[i] = _data->_stateEstimator->getResult().vWorld(i);
        // v_des[i] = 0;
        //v_des[2] = 0.005;

        se_xfb[4 + i] = _data->_stateEstimator->getResult().position(i);
        se_xfb[7 + i] = _data->_stateEstimator->getResult().omegaWorld(i);
        se_xfb[10 + i] = _data->_stateEstimator->getResult().vWorld(i);
        
    // Set the translational and orientation gains
        kpCOM[i] = 50;   //_data->controlParameters->kpCOM(i);
        kdCOM[i] = 10;     //_data->controlParameters->kdCOM(i);
        kpBase[i] = 100;     //_data->controlParameters->kpBase(i);
        kdBase[i] = 50; //  _data->controlParameters->kdBase(i);
        }
      
      kpCOM[2] = 50;
      kpBase[0] = 300;
      kpBase[1] = 200;
   

    //Vec3<double> pFeetVec;
    Vec3<double> pFeetVecCOM;

    // Get the foot locations relative to COM
      for (int leg = 0; leg < 4; leg++) {
        // computeLegJacobianAndPosition(&_data->_quadruped, _data->_legController->data[leg].q,
        //                           (Mat3<double>*)nullptr, &pFeetVec, leg);
        //pFeetVecCOM = _data->_stateEstimator->getResult().rBody.transpose() *
                  //(_data->_quadruped->getHipLocation(leg) + pFeetVec);

        pFeetVecCOM =  _data->_stateEstimator->getResult().rBody.transpose() *
        (_data->_quadruped->getHipLocation(leg) + _data->_legController->data[leg].p);


        pFeet[leg * 3] = pFeetVecCOM[0];
        pFeet[leg * 3 + 1] = pFeetVecCOM[1];
        pFeet[leg * 3 + 2] = pFeetVecCOM[2];
        //std::cout << "pFeet" << leg << std::endl;
      }
    //  myfile << "\n";
     // std::cout << j << std::endl;
     //std::cout << "run QP" << std::endl;
    balanceController.set_alpha_control(0.01);
    balanceController.set_friction(0.6);
    balanceController.set_mass(_data->_quadruped->mass);
    balanceController.set_wrench_weights(COM_weights_stance, Base_weights_stance);
    balanceController.set_PDgains(kpCOM, kdCOM, kpBase, kdBase);
    balanceController.set_desiredTrajectoryData(rpy, p_des, omegaDes, v_des);
    balanceController.SetContactData(contactStateScheduled, minForces, maxForces);
    balanceController.updateProblemData(se_xfb, pFeet, p_des, p_act, v_des, v_act,
                                      O_err, _data->_stateEstimator->getResult().rpy(2));
   // balanceController.print_QPData();
    double fOpt[12];
    balanceController.solveQP_nonThreaded(fOpt);

    for (int leg = 0; leg < 4; leg++) {
        footFeedForwardForces.col(leg) << fOpt[leg * 3], fOpt[leg * 3 + 1],
        fOpt[leg * 3 + 2]; // force in world frame, need to convert to body frame

        _data->_legController->commands[leg].feedforwardForce = footFeedForwardForces.col(leg);
        
      }
     

    }

    _data->_legController->updateCommand();
    rate.sleep();
  
    counter++;
  }

  foot_force.close();
  pos.close();
  rpy_log.close();
}

// least square solution test
/*
void FSM_State::runLeastSquare(){

    ofstream myfile;
    ofstream Sol;
    ofstream f_y;
   // ofstream z_pos;
  //  ofstream b_des;
    myfile.open ("ori_leastSqure.txt");
    Sol.open("leastSquareSol.txt");
    f_y.open("fy.txt");
   // b_des.open("b_des_z.txt");
     ros::Rate rate(1000);
    footFeedForwardForces = Mat34<double>::Zero();
    footstepLocations = Mat34<double>::Zero();

    double minForce = 15;
    double maxForce = 500;
    double contactStateScheduled[4] = {1, 1, 1, 1};

    double minForces[4] = {minForce, minForce, minForce, minForce};
    double maxForces[4] = {maxForce, maxForce, maxForce, maxForce};

    double pFeet[12], p_des[3], p_act[3], v_des[3], v_act[3], O_err[3], rpy[3],
      omegaDes[3];
    double se_xfb[13];
    double kpCOM[3], kdCOM[3], kpBase[3], kdBase[3];

     ros::spinOnce();
       _data->_legController->updateData();
       _data->_stateEstimator->run();

    for(int i = 0; i < 3; i++){
      p_des[i] = _data->_stateEstimator->getResult().position(i); 
      v_des[i] = 0;
      omegaDes[i] = 0;
      rpy[i] = 0;
      rpy[2] = _data->_stateEstimator->getResult().rpy(2);
    } 
    p_des[2] = 0.4;

   for(int j =0; j < 500; j++) {
      
      _data->_legController->updateData();
      _data->_stateEstimator->run();

      for (int i = 0; i < 4; i++) {
        se_xfb[i] = _data->_stateEstimator->getResult().orientation(i);
      }
    // se_xfb[3] = 1.0;
      for (int i = 0; i < 3; i++) {
        //rpy[i] = _data->_stateEstimator->getResult().rpy(i);
        //p_des[i] = _data->_stateEstimator->getResult().position(i);
        p_act[i] = _data->_stateEstimator->getResult().position(i);
        //omegaDes[i] = _data->_stateEstimator->getResult().omegaBody(i);
        v_act[i] = _data->_stateEstimator->getResult().vBody(i);
        //v_des[i] = _data->_stateEstimator->getResult().vBody(i);

        se_xfb[4 + i] = _data->_stateEstimator->getResult().position(i);
        se_xfb[7 + i] = _data->_stateEstimator->getResult().omegaBody(i);
        se_xfb[10 + i] = _data->_stateEstimator->getResult().vBody(i);

    // Set the translational and orientation gains
        kpCOM[i] = 30;    //_data->controlParameters->kpCOM(i);
        kdCOM[i] =  10; //_data->controlParameters->kdCOM(i);
        kpBase[i] = 30;   //_data->controlParameters->kpBase(i);
        kdBase[i] = 10; //  _data->controlParameters->kdBase(i);
      }
      kpCOM[2] = 30;
      kdCOM[2] = 10;
   
      Vec3<double> pFeetVec;
      Vec3<double> pFeetVecCOM;

      for(int i =0; i < 3; i++){
        myfile << _data->_stateEstimator->getResult().rpy(i);
        myfile << " " ;
      }
      myfile << "\n";
    // Get the foot locations relative to COM
       for (int leg = 0; leg < 4; leg++) {
        computeLegJacobianAndPosition(_data->_legController->data[leg].q,
                                  (Mat3<double>*)nullptr, &pFeetVec, leg);
        //pFeetVecCOM = _data->_stateEstimator->getResult().rBody.transpose() *
                  //(_data->_quadruped->getHipLocation(leg) + pFeetVec);

        pFeetVecCOM =  _data->_stateEstimator->getResult().rBody.transpose() *
        (_data->_quadruped->getHipLocation(leg) + _data->_legController->data[leg].p);


        pFeet[leg * 3] = pFeetVecCOM[0];
        pFeet[leg * 3 + 1] = pFeetVecCOM[1];
        pFeet[leg * 3 + 2] = pFeetVecCOM[2];
       // std::cout << "pFeet" << leg << std::endl;
       // std::cout << pFeetVecCOM << std::endl;
      }

    leastSquareController.set_alpha_control(0.01);
    leastSquareController.set_mass(19.0);
    leastSquareController.set_friction(0.4);
    //std::cout << "mass set \n";
    leastSquareController.set_PDgains(kpCOM, kdCOM, kpBase, kdBase);
    //std::cout << "PD gain set \n";
    leastSquareController.set_desiredTrajectoryData(rpy, p_des, omegaDes, v_des);
    //std::cout << "desired traj set \n";
    leastSquareController.SetContactData(contactStateScheduled, minForces, maxForces);
    //std::cout << "cotanct set \n"; 
    leastSquareController.updateProblemData(se_xfb, pFeet, p_des, p_act, v_des, v_act,
                                      O_err, _data->_stateEstimator->getResult().rpy(2));

    double fOpt[12];
  
    leastSquareController.solveLeastSquare(fOpt);

    std::cout << "finished" << std::endl;
    // balanceController.solveQP_nonThreaded(fOpt);

    // Publish the results over ROS
    // balanceController.publish_data_lcm();

    // Copy the results to the feed forward forces
    for (int leg = 0; leg < 4; leg++) {
      footFeedForwardForces.col(leg) << fOpt[leg * 3], fOpt[leg * 3 + 1],
      fOpt[leg * 3 + 2];

     _data->_legController->commands[leg].feedforwardForce = _data->_stateEstimator->getResult().rBody.transpose() *
                                                             footFeedForwardForces.col(leg);
      //Sol << _data->_legController->commands[leg].feedforwardForce[2] << " ";
      }
      //Sol << "\n";
    std::cout << footFeedForwardForces << std::endl;
    _data->_legController->updateCommand();
    rate.sleep();
  }

  Sol.close();
  myfile.close();
}
*/

void FSM_State::PDstand(){
  ros::Rate rate(1000);
  int counter = 0;
  
  // initial foot possition
  Mat34<double> init_foot_pos;
  _data->_legController->updateData();
  for(int i = 0; i < 4; i++){
    init_foot_pos.col(i) = _data->_legController->data[i].p;
  }
  double h = 0.4; // standup height
  double side_sign[4] = {-1, 1, -1, 1};
  Vec3<double> ff_force_world(0.5, 0, 0);

  while(ros::ok()){
     double progress = counter * 0.001;
     if(progress > 1.)  {progress = 1.;}

    _data->_legController->updateData();  
    _data->_stateEstimator->run();    

      for(int i = 0; i < 4; i++){
        _data->_legController->commands[i].kpCartesian = Vec3<double>(400,400,900).asDiagonal();
        _data->_legController->commands[i].kdCartesian = Vec3<double>(20,20,20).asDiagonal();

        _data->_legController->commands[i].pDes << 0, side_sign[i] * 0.083, 0;
        
        _data->_legController->commands[i].pDes[2] = -h*progress + (1. - progress) * init_foot_pos(2, i);
       // _data->_legController->commands[i].feedforwardForce = _data->_stateEstimator->getResult().rBody.transpose() * ff_force_world;
       // std::cout << "leg  " << i << "  " << _data->_legController->data[i].p << std::endl;
      }

      _data->_legController->updateCommand();
      counter++;

      rate.sleep();
  }
}
