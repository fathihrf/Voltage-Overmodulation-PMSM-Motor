#define S_FUNCTION_LEVEL 2 
#define S_FUNCTION_NAME plant_motor
#include "simstruc.h" 
#include <math.h> 

// Definisikan 6 parameter fisik motor SPMSM
#define RS_IDX 0
#define LS_IDX 1
#define P_IDX  2
#define PHIF_IDX 3
#define B_IDX  4
#define J_IDX  5
#define N_PARAMS 6

#define U(element) (*uPtrs[element])

static void mdlInitializeSizes(SimStruct *S){ 
    ssSetNumSFcnParams(S, N_PARAMS);
    if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) return;

    // Set 3 state kontinu: i_d, i_q, omega_m
    ssSetNumContStates(S, 3); 

    // 1 input port dengan 3 sinyal: [u_d, u_q, T_L]
    if (!ssSetNumInputPorts(S, 1)) return; 
    ssSetInputPortWidth(S, 0, 3); 
    ssSetInputPortDirectFeedThrough(S, 0, 0); 

    // 1 output port dengan 3 sinyal: [i_d, i_q, omega_m]
    if (!ssSetNumOutputPorts(S, 1)) return; 
    ssSetOutputPortWidth(S, 0, 3); 
    
    ssSetNumSampleTimes(S, 1); 
    ssSetOptions(S, SS_OPTION_EXCEPTION_FREE_CODE); 
} 

static void mdlInitializeSampleTimes(SimStruct *S) { 
    ssSetSampleTime(S, 0, CONTINUOUS_SAMPLE_TIME); 
    ssSetOffsetTime(S, 0, 0.0); 
} 

#define MDL_INITIALIZE_CONDITIONS 
static void mdlInitializeConditions(SimStruct *S) { 
    real_T *X0 = ssGetContStates(S); 
    // Inisialisasi semua state ke 0.0
    X0[0] = 0.0; // i_d(0)
    X0[1] = 0.0; // i_q(0)
    X0[2] = 0.0; // omega_m(0)
} 

static void mdlOutputs(SimStruct *S, int_T tid) { 
    real_T *Y = ssGetOutputPortRealSignal(S,0); 
    real_T *X = ssGetContStates(S); 
    // Output y = x
    Y[0] = X[0]; // i_d
    Y[1] = X[1]; // i_q
    Y[2] = X[2]; // omega_m
} 

#define MDL_DERIVATIVES 
static void mdlDerivatives(SimStruct *S) { 
    const real_T Rs   = mxGetScalar(ssGetSFcnParam(S, RS_IDX));
    const real_T Ls   = mxGetScalar(ssGetSFcnParam(S, LS_IDX));
    const real_T P    = mxGetScalar(ssGetSFcnParam(S, P_IDX));
    const real_T Phif = mxGetScalar(ssGetSFcnParam(S, PHIF_IDX));
    const real_T B    = mxGetScalar(ssGetSFcnParam(S, B_IDX));
    const real_T J    = mxGetScalar(ssGetSFcnParam(S, J_IDX));

    real_T *dX = ssGetdX(S); 
    real_T *X = ssGetContStates(S); 
    InputRealPtrsType uPtrs = ssGetInputPortRealSignalPtrs(S,0); 

    // State saat ini: [i_d, i_q, omega_m]
    real_T id = X[0];
    real_T iq = X[1];
    real_T wm = X[2];
    
    // Input saat ini: [u_d, u_q, T_L]
    real_T ud = U(0);
    real_T uq = U(1);
    real_T TL = U(2);

    // Sistem Persamaan Non-linear State Space
    // di_d/dt = -(Rs/Ls)*i_d + P*wm*i_q + (1/Ls)*u_d
    dX[0] = -(Rs/Ls)*id + P*wm*iq + (1.0/Ls)*ud;      
    // di_q/dt = -P*wm*i_d - (Rs/Ls)*i_q - (P*Phif/Ls)*wm + (1/Ls)*u_q
    dX[1] = -P*wm*id - (Rs/Ls)*iq - (P*Phif/Ls)*wm + (1.0/Ls)*uq; 
    // d(wm)/dt = (3P*Phif/(2J))*i_q - (B/J)*wm - (1/J)*T_L
    dX[2] = (3.0*P*Phif/(2.0*J))*iq - (B/J)*wm - (1.0/J)*TL;
} 

static void mdlTerminate(SimStruct *S) {} 
#ifdef MATLAB_MEX_FILE 
#include "simulink.c" 
#else 
#include "cg_sfun.h" 
#endif