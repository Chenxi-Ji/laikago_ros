#include "Solver.h"
#include "../ConvexMPC/common_types.h"
#include "../ConvexMPC/RobotState.h"
#include <eigen3/Eigen/Dense>
#include <cmath>
#include <eigen3/unsupported/Eigen/MatrixFunctions>
#include "../qpOASES/include/qpOASES.hpp"
#include <stdio.h>
#include <sys/time.h>
#include "../include/Utilities/Timer.h"

#define BIG_NUMBER 5e10
//big enough to act like infinity, small enough to avoid numerical weirdness.

using Eigen::Dynamic;

Matrix<fpt, Dynamic, 12> A_qp;
Matrix<fpt, Dynamic, Dynamic> B_qp;
Matrix<fpt, 12, 12> Adt;    // A_continuous time
Matrix<fpt, 12, 12> Bdt;    // B_continuous time
Matrix<fpt, 24, 24> Abc, expmm;
Matrix<fpt, Dynamic, Dynamic> S;
Matrix<fpt, Dynamic, 1> x_d; // x_desired
Matirx<fpt, Dynamic, 1> U_b; // upper bound
Matrix<fpt, Dynamic, Dynamic> fmat;

Matrix<fpt, Dynamic, Dynamic> qH;
Matrix<fpt, Dynamic, 1> qg;

Matrix<fpt, Dynamic, Dynamic> eye_12;

qpOASES::real_t* H_qpoases;
qpOASES::real_t* g_qpoases;
qpOASES::real_t* A_qpoases;
qpOASES::real_t* lb_qpoases;
qpOASES::real_t* ub_qpoases;
qpOASES::real_t* q_soln;

qpOASES::real_t* H_red;
qpOASES::real_t* g_red;
qpOASES::real_t* A_red;
qpOASES::real_t* lb_red;
qpOASES::real_t* ub_red;
qpOASES::real_t* q_red;
u8 real_allocated = 0;


char var_elim[2000];
char con_elim[2000];

s8 near_zero(fpt a)
{
  return (a < 0.01 && a > -.01) ;
}

s8 near_one(fpt a)
{
  return near_zero(a-1);
}
void matrix_to_real(qpOASES::real_t* dst, Matrix<fpt,Dynamic,Dynamic> src, s16 rows, s16 cols)
{
  s32 a = 0;
  for(s16 r = 0; r < rows; r++)
  {
    for(s16 c = 0; c < cols; c++)
    {
      dst[a] = src(r,c);
      a++;
    }
  }
}

void resizeMatrices(s16 horizon)
{
  int mcount = 0;
  int h2 = horizon * horizon;

  A_qp.resize(12*horizon, Eigen::NoChange);
  mcount += 12*horizon*1;

  B_qp.resize(12*horizon, 12*horizon);
  mcount += 12*h2*12;

  S.resize(12*horizon, 12*horizon);
  mcount += 12*12*h2;

  X_d.resize(12*horizon, Eigen::NoChange);
  mcount += 12*horizon;

  U_b.resize(12*horizon, Eigen::NoChange);
  mcount += 12*h2;

  fmat.resize(20*horizon, 12*horizon);
  mcount += 20*12*h2;

  qH.resize(12*horizon, 12*horizon);
  mcount +=12*12*h2;

  qg.resize(12*horizon, Eigen::NoChange);
  mcount += 12*horizon;

  eye_12h.resize(12*horizon, 12*horizon);
  mcount += 12*12*horizon;

  //printf("realloc'd %d floating point numbers.\n",mcount);
  mcount = 0;

  A_qp.setZero();
  B_qp.setZero();
  S.setZero();
  X_d.setZero();
  U_b.setZero();
  fmat.setZero();
  qH.setZero();
  eye_12h.setIdentity();

  //TODO: use realloc instead of free/malloc on size changes

  if(real_allocated)
  {

    free(H_qpoases);
    free(g_qpoases);
    free(A_qpoases);
    free(lb_qpoases);
    free(ub_qpoases);
    free(q_soln);
    free(H_red);
    free(g_red);
    free(A_red);
    free(lb_red);
    free(ub_red);
    free(q_red);
  }

  H_qpoases = (qpOASES::real_t*)malloc(12*12*horizon*horizon*sizeof(qpOASES::real_t));
  mcount += 12*12*h2;
  g_qpoases = (qpOASES::real_t*)malloc(12*1*horizon*sizeof(qpOASES::real_t));
  mcount += 12*horizon;
  A_qpoases = (qpOASES::real_t*)malloc(12*20*horizon*horizon*sizeof(qpOASES::real_t));
  mcount += 12*20*h2;
  lb_qpoases = (qpOASES::real_t*)malloc(20*1*horizon*sizeof(qpOASES::real_t));
  mcount += 20*horizon;
  ub_qpoases = (qpOASES::real_t*)malloc(20*1*horizon*sizeof(qpOASES::real_t));
  mcount += 20*horizon;
  q_soln = (qpOASES::real_t*)malloc(12*horizon*sizeof(qpOASES::real_t));
  mcount += 12*horizon;

  H_red = (qpOASES::real_t*)malloc(12*12*horizon*horizon*sizeof(qpOASES::real_t));
  mcount += 12*12*h2;
  g_red = (qpOASES::real_t*)malloc(12*1*horizon*sizeof(qpOASES::real_t));
  mcount += 12*horizon;
  A_red = (qpOASES::real_t*)malloc(12*20*horizon*horizon*sizeof(qpOASES::real_t));
  mcount += 12*20*h2;
  lb_red = (qpOASES::real_t*)malloc(20*1*horizon*sizeof(qpOASES::real_t));
  mcount += 20*horizon;
  ub_red = (qpOASES::real_t*)malloc(20*1*horizon*sizeof(qpOASES::real_t));
  mcount += 20*horizon;
  q_red = (qpOASES::real_t*)malloc(12*horizon*sizeof(qpOASES::real_t));
  mcount += 12*horizon;
  real_allocated = 1;
}

void convertToqp(Matrix<fpt,12,12> Ac, Matrix<fpt,12,12> Bc, fpt dt, s16 horizon)
{

}