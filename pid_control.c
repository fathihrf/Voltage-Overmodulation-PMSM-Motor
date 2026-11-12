#define S_FUNCTION_LEVEL 2
#define S_FUNCTION_NAME pid_control
#include "simstruc.h"
#include <math.h>

// Batasan keluaran (Anti-Windup dan Saturation)
#define MAX_IQ_REF 100.0   // Batasan referensi arus q-axis [A]
#define MAX_V      400.0   // Batasan tegangan d-q [V]

// Definisikan 9 parameter gain
#define KP_W_IDX  0
#define KI_W_IDX  1
#define KD_W_IDX  2

#define KP_ID_IDX 3
#define KI_ID_IDX 4
#define KD_ID_IDX 5

#define KP_IQ_IDX 6
#define KI_IQ_IDX 7
#define KD_IQ_IDX 8

#define N_PARAMS 9

// Definisikan 6 state diskrit
#define INT_ERR_W_IDX  0
#define INT_ERR_ID_IDX 1
#define INT_ERR_IQ_IDX 2

#define PREV_ERR_W_IDX  3
#define PREV_ERR_ID_IDX 4
#define PREV_ERR_IQ_IDX 5

#define U(element) (*uPtrs[element])

static void mdlInitializeSizes(SimStruct *S) {
    ssSetNumSFcnParams(S, N_PARAMS);
    if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) return;

    ssSetNumDiscStates(S, 6);

    // Input: [omega_ref, id_ref, id_fb, iq_fb, omega_fb]
    if (!ssSetNumInputPorts(S, 1)) return;
    ssSetInputPortWidth(S, 0, 5); 

    // Output: [u_d, u_q]
    if (!ssSetNumOutputPorts(S, 1)) return;
    ssSetOutputPortWidth(S, 0, 2); 

    ssSetInputPortDirectFeedThrough(S, 0, 1);
    ssSetNumSampleTimes(S, 1);
    ssSetOptions(S, SS_OPTION_EXCEPTION_FREE_CODE);
}

static void mdlInitializeSampleTimes(SimStruct *S) {
    ssSetSampleTime(S, 0, 1e-4);
    ssSetOffsetTime(S, 0, 0.0);
}

#define MDL_INITIALIZE_CONDITIONS
static void mdlInitializeConditions(SimStruct *S) {
    real_T *X0 = ssGetRealDiscStates(S);
    int i;
    for (i = 0; i < 6; i++) {
        X0[i] = 0.0;
    }
}

static void mdlOutputs(SimStruct *S, int_T tid) {
    const real_T Kp_w = mxGetScalar(ssGetSFcnParam(S, KP_W_IDX));
    const real_T Ki_w = mxGetScalar(ssGetSFcnParam(S, KI_W_IDX));
    const real_T Kd_w = mxGetScalar(ssGetSFcnParam(S, KD_W_IDX));

    const real_T Kp_id = mxGetScalar(ssGetSFcnParam(S, KP_ID_IDX));
    const real_T Ki_id = mxGetScalar(ssGetSFcnParam(S, KI_ID_IDX));
    const real_T Kd_id = mxGetScalar(ssGetSFcnParam(S, KD_ID_IDX));

    const real_T Kp_iq = mxGetScalar(ssGetSFcnParam(S, KP_IQ_IDX));
    const real_T Ki_iq = mxGetScalar(ssGetSFcnParam(S, KI_IQ_IDX));
    const real_T Kd_iq = mxGetScalar(ssGetSFcnParam(S, KD_IQ_IDX));

    real_T *Y = ssGetOutputPortRealSignal(S, 0);
    real_T *X = ssGetRealDiscStates(S);
    InputRealPtrsType uPtrs = ssGetInputPortRealSignalPtrs(S, 0);
    real_T dt = ssGetSampleTime(S, 0);

    real_T omega_ref = U(0);
    real_T id_ref    = U(1);
    real_T id_fb     = U(2);
    real_T iq_fb     = U(3);
    real_T omega_fb  = U(4);

    // --- 1. Loop Kecepatan ---
    real_T error_w = omega_ref - omega_fb;
    real_T deriv_err_w = (dt > 0) ? (error_w - X[PREV_ERR_W_IDX]) / dt : 0.0;
    
    real_T iq_ref = (Kp_w * error_w) + (Ki_w * X[INT_ERR_W_IDX]) + (Kd_w * deriv_err_w);
    // Saturasi
    if (iq_ref > MAX_IQ_REF) iq_ref = MAX_IQ_REF;
    if (iq_ref < -MAX_IQ_REF) iq_ref = -MAX_IQ_REF;

    // --- 2. Loop Arus D ---
    real_T error_id = id_ref - id_fb;
    real_T deriv_err_id = (dt > 0) ? (error_id - X[PREV_ERR_ID_IDX]) / dt : 0.0;
    
    real_T ud = (Kp_id * error_id) + (Ki_id * X[INT_ERR_ID_IDX]) + (Kd_id * deriv_err_id);
    // Saturasi Tegangan
    if (ud > MAX_V) ud = MAX_V;
    if (ud < -MAX_V) ud = -MAX_V;

    // --- 3. Loop Arus Q ---
    real_T error_iq = iq_ref - iq_fb;
    real_T deriv_err_iq = (dt > 0) ? (error_iq - X[PREV_ERR_IQ_IDX]) / dt : 0.0;
    
    real_T uq = (Kp_iq * error_iq) + (Ki_iq * X[INT_ERR_IQ_IDX]) + (Kd_iq * deriv_err_iq);
    // Saturasi Tegangan
    if (uq > MAX_V) uq = MAX_V;
    if (uq < -MAX_V) uq = -MAX_V;

    Y[0] = ud;
    Y[1] = uq;
}

#define MDL_UPDATE
static void mdlUpdate(SimStruct *S, int_T tid) {
    real_T *X = ssGetRealDiscStates(S);
    InputRealPtrsType uPtrs = ssGetInputPortRealSignalPtrs(S, 0);
    real_T dt = ssGetSampleTime(S, 0);

    const real_T Kp_w = mxGetScalar(ssGetSFcnParam(S, KP_W_IDX));
    const real_T Ki_w = mxGetScalar(ssGetSFcnParam(S, KI_W_IDX));
    
    const real_T Kp_id = mxGetScalar(ssGetSFcnParam(S, KP_ID_IDX));
    const real_T Ki_id = mxGetScalar(ssGetSFcnParam(S, KI_ID_IDX));
    
    const real_T Kp_iq = mxGetScalar(ssGetSFcnParam(S, KP_IQ_IDX));
    const real_T Ki_iq = mxGetScalar(ssGetSFcnParam(S, KI_IQ_IDX));

    real_T omega_ref = U(0);
    real_T id_ref    = U(1);
    real_T id_fb     = U(2);
    real_T iq_fb     = U(3);
    real_T omega_fb  = U(4);

    real_T error_w = omega_ref - omega_fb;
    
    // Cek batas iq_ref sementara untuk anti-windup (Conditional Integration)
    real_T iq_ref_temp = (Kp_w * error_w) + (Ki_w * X[INT_ERR_W_IDX]);
    
    // Hanya akumulasi jika tidak melewati batas (Saturasi) 
    // atau jika error bertentangan arah sehingga akan mengurangi integrasi
    if ((iq_ref_temp < MAX_IQ_REF && iq_ref_temp > -MAX_IQ_REF) || 
        (iq_ref_temp >= MAX_IQ_REF && error_w < 0) || 
        (iq_ref_temp <= -MAX_IQ_REF && error_w > 0)) {
        X[INT_ERR_W_IDX] += error_w * dt;
    }

    real_T error_id = id_ref - id_fb;
    real_T ud_temp = (Kp_id * error_id) + (Ki_id * X[INT_ERR_ID_IDX]);
    if ((ud_temp < MAX_V && ud_temp > -MAX_V) || 
        (ud_temp >= MAX_V && error_id < 0) || 
        (ud_temp <= -MAX_V && error_id > 0)) {
        X[INT_ERR_ID_IDX] += error_id * dt;
    }

    // Kalkulasi error_iq mengharuskan kita tahu iq_ref yang saturated
    real_T deriv_err_w = (dt > 0) ? (error_w - X[PREV_ERR_W_IDX]) / dt : 0.0;
    real_T iq_ref = (Kp_w * error_w) + (Ki_w * X[INT_ERR_W_IDX]) + (mxGetScalar(ssGetSFcnParam(S, KD_W_IDX)) * deriv_err_w);
    if (iq_ref > MAX_IQ_REF) iq_ref = MAX_IQ_REF;
    if (iq_ref < -MAX_IQ_REF) iq_ref = -MAX_IQ_REF;

    real_T error_iq = iq_ref - iq_fb;
    real_T uq_temp = (Kp_iq * error_iq) + (Ki_iq * X[INT_ERR_IQ_IDX]);
    
    if ((uq_temp < MAX_V && uq_temp > -MAX_V) || 
        (uq_temp >= MAX_V && error_iq < 0) || 
        (uq_temp <= -MAX_V && error_iq > 0)) {
        X[INT_ERR_IQ_IDX] += error_iq * dt;
    }

    // Update state turunan (previous error)
    X[PREV_ERR_W_IDX] = error_w;
    X[PREV_ERR_ID_IDX] = error_id;
    X[PREV_ERR_IQ_IDX] = error_iq;
}

static void mdlTerminate(SimStruct *S) {}

#ifdef MATLAB_MEX_FILE
#include "simulink.c"
#else
#include "cg_sfun.h"
#endif
