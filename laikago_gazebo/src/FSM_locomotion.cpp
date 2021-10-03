/*============================ Locomotion =============================*/
/**
 * FSM State for robot locomotion. Manages the contact specific logic
 * and handles calling the interfaces to the controllers. This state
 * should be independent of controller, gait, and desired trajectory.
 */

#include "../include/FSM_locomotion.h"
#include "../include/QPLocomotion.h"
#include "../include/Utilities/Timer.h"

/**
 * Constructor
 * 
 * @param _controlFSMData store all relevant control data 
 */ 
FSM_locomotion::FSM_locomotion(ControlFSMData<double>* _controlFSMData)
: FSM_State(_controlFSMData),
  qp_locomotion(0.001, 30),
  {
    this->footFeedForwardForces = Mat34<double>::Zero();
    this->footstepLocations = Mat34<double>::Zero();
    balanceController.set_alpha_contro(0.01);
    balanceController.set_friction(0.6);
    balanceController.set_mass(19);
  }

void FSM_locomotion::run() {
    std::cout << "start locomotion" << std::endl;
    LocomotionControlStep();
  }

void FSM_locomotion::LocomotionControlStep() {
    qp_locomotion.run(*this->_data);

    
  }