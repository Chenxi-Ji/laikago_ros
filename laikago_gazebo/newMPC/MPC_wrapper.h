#ifndef MPC_Wrapper
#define MPC_Wrapper
#define K_MAX_GAIT_SEGEMNTS 36

#ifdef __cplusplus
#define EXTERNC extern "C"
#else
#define EXTERNC
#endif

struct MPC_setup
{
    float dt;
    float mu;
    float f_max;
    int horizon;
};

struct update_data
{
    float p[3];
    float v[3];
    float q[4];
    float w[3];
    float r[12];
    float yaw;
    float weights[12];
    float traj[12*K_MAX_GAIT_SEGEMNTS];
    float alpha;
    unsigned char gait[K_MAX_GAIT_SEGEMNTS];

};

EXTERNC void setup_MPC(double dt, double horizon, double mu, double f_max);
EXTERNC void update_MPC_data(double* p, double* v, double* q, double* r, double yaw, double* weights, double* state_traj, double alpha, int* gait);
EXTERNC double get_solution(int index);

#endif