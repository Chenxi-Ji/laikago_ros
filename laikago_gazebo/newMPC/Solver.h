#ifndef _MPC_solver
#define _MPC_solver

#include <eigen3/Eigen/Dense>
#include "../ConvexMPC/common_types.h"
#include <iostream>
#include <stdio.h>
#include "MPC_wrapper.h"

using Eigen::Matrix;
using Eigen::Quaternionf;
using Eigen::Quaterniond;

template <class T>
T t_min(T a, T b)
{
    if(a < b) return a;
    return b;
}

template <class T>
T sq(T a)
{
    return a*a;
}

void solve(update_data* update, MPC_setup* setup);

void quat2rpy(Quaternionf q, Matrix<fpt,3,1>& rpy);
void StateSpaceMat(Matrix<fpt,3,3> I_world, fpt m, Matrix<fpt,3,4> r_feet, Matrix<fpt,3,3> R_yaw, Matrix<fpt,12,12>& A, Matrix<fpt,12,12>& B);
void resizeMatix(s16 horizon);
void convertToqp(Matrix<fpt,12,12> Ac, Matrix<fpt,12,12> Bc, fpt dt, s16 horizon);

#endif