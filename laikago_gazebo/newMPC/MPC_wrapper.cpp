#include "MPC_wrapper.h"
#include "../ConvexMPC/common_types.h"
#include "Solver.h"
#include <eigen3/Eigen/Dense>
#include <pthread.h>
#include <stdio.h>
#include <string.h>

#define K_NUM_LEGS 4

MPC_setup problem_config;
u8 gait_data[K_MAX_GAIT_SEGEMNTS];
pthread_mutex_t problem_cfg_mt;
pthread_mutex_t update_mt;
update_data update;
pthread_t solve_thread;

u8 first_run = 1;

void setup_MPC(double dt, double horizon, double mu, double f_max)
{
    if(first_run)
    {
        fisrt_run = false;
        initialize_mpc();
    }

    resize_qp_mats(horizon);
}