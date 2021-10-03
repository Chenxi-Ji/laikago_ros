#ifndef CONTROLFSMDATA_H
#define CONTROLFSMDATA_H

#include "DesiredCommand.h"
#include "GaitScheduler.h"
#include "LegController.h"
#include "Quadruped.h"
//#include "StateEstimatorContainer.h"
// #include "Dynamics/Quadruped.h"

/**
 *
 */
//template <typename T>
struct ControlFSMData {
  // EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Quadruped* _quadruped;
  StateEstimatorContainer* _stateEstimator;
 // StateEstimatorContainer* _testEstimator;
  LegController* _legController;
  GaitScheduler<double>* _gaitScheduler;
  DesiredStateCommand* _desiredStateCommand;
};


#endif  // CONTROLFSM_H