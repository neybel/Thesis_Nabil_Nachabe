/*
 * Copyright (c) The acados authors.
 *
 * This file is part of acados.
 *
 * The 2-Clause BSD License
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.;
 */

#define S_FUNCTION_NAME acados_solver_sfunction_pendulum
#define S_FUNCTION_LEVEL 2

#define MDL_START

// acados
// #include "acados/utils/print.h"
#include "acados_c/sim_interface.h"
#include "acados_c/external_function_interface.h"

// example specific
#include "pendulum_model/pendulum_model.h"
#include "acados_solver_pendulum.h"



#include "simstruc.h"

#define SAMPLINGTIME -1

static void mdlInitializeSizes (SimStruct *S)
{
    // specify the number of continuous and discrete states
    ssSetNumContStates(S, 0);
    ssSetNumDiscStates(S, 0);

    int N = PENDULUM_N;      // specify the number of input ports
    if ( !ssSetNumInputPorts(S, 14) )
        return;

    // specify the number of output ports
    if ( !ssSetNumOutputPorts(S, 9) )
        return;

    // specify dimension information for the input ports
    // lbx_0
    ssSetInputPortVectorDimension(S, 0, 4);
    // ubx_0
    ssSetInputPortVectorDimension(S, 1, 4);
    // y_ref_0
    ssSetInputPortVectorDimension(S, 2, 5);
    // y_ref
    ssSetInputPortVectorDimension(S, 3, 95);
    // y_ref_e
    ssSetInputPortVectorDimension(S, 4, 4);
    // lbu
    ssSetInputPortVectorDimension(S, 5, 20);
    // ubu
    ssSetInputPortVectorDimension(S, 6, 20);
    // lh_0
    ssSetInputPortVectorDimension(S, 7, 1);
    // uh_0
    ssSetInputPortVectorDimension(S, 8, 1);  
    // cost_W_0
    ssSetInputPortVectorDimension(S, 9, 25);  
    // cost_W
    ssSetInputPortVectorDimension(S, 10, 25);  
    // cost_W_e
    ssSetInputPortVectorDimension(S, 11, 16);
    // reset_solver
    ssSetInputPortVectorDimension(S, 12, 1);
    // x_init
    ssSetInputPortVectorDimension(S, 13, 84);/* specify dimension information for the OUTPUT ports */
    ssSetOutputPortVectorDimension(S, 0, 1 );
    ssSetOutputPortVectorDimension(S, 1, 20 );
    ssSetOutputPortVectorDimension(S, 2, 84 );
    ssSetOutputPortVectorDimension(S, 3, 1 );
    ssSetOutputPortVectorDimension(S, 4, 1 );
    ssSetOutputPortVectorDimension(S, 5, 4 );
    ssSetOutputPortVectorDimension(S, 6, 4 ); // state at shooting node 1
    ssSetOutputPortVectorDimension(S, 7, 1);
    ssSetOutputPortVectorDimension(S, 8, 1 );

    // specify the direct feedthrough status
    // should be set to 1 for all inputs used in mdlOutputs
    ssSetInputPortDirectFeedThrough(S, 0, 1);
    ssSetInputPortDirectFeedThrough(S, 1, 1);
    ssSetInputPortDirectFeedThrough(S, 2, 1);
    ssSetInputPortDirectFeedThrough(S, 3, 1);
    ssSetInputPortDirectFeedThrough(S, 4, 1);
    ssSetInputPortDirectFeedThrough(S, 5, 1);
    ssSetInputPortDirectFeedThrough(S, 6, 1);
    ssSetInputPortDirectFeedThrough(S, 7, 1);
    ssSetInputPortDirectFeedThrough(S, 8, 1);
    ssSetInputPortDirectFeedThrough(S, 9, 1);
    ssSetInputPortDirectFeedThrough(S, 10, 1);
    ssSetInputPortDirectFeedThrough(S, 11, 1);
    ssSetInputPortDirectFeedThrough(S, 12, 1);
    ssSetInputPortDirectFeedThrough(S, 13, 1);

    // one sample time
    ssSetNumSampleTimes(S, 1);
}


#if defined(MATLAB_MEX_FILE)

#define MDL_SET_INPUT_PORT_DIMENSION_INFO
#define MDL_SET_OUTPUT_PORT_DIMENSION_INFO

static void mdlSetInputPortDimensionInfo(SimStruct *S, int_T port, const DimsInfo_T *dimsInfo)
{
    if ( !ssSetInputPortDimensionInfo(S, port, dimsInfo) )
         return;
}

static void mdlSetOutputPortDimensionInfo(SimStruct *S, int_T port, const DimsInfo_T *dimsInfo)
{
    if ( !ssSetOutputPortDimensionInfo(S, port, dimsInfo) )
         return;
}

#endif /* MATLAB_MEX_FILE */


static void mdlInitializeSampleTimes(SimStruct *S)
{
    ssSetSampleTime(S, 0, SAMPLINGTIME);
    ssSetOffsetTime(S, 0, 0.0);
}


static void mdlStart(SimStruct *S)
{
    pendulum_solver_capsule *capsule = pendulum_acados_create_capsule();
    pendulum_acados_create(capsule);

    ssSetUserData(S, (void*)capsule);
}


static void mdlOutputs(SimStruct *S, int_T tid)
{
    pendulum_solver_capsule *capsule = ssGetUserData(S);
    ocp_nlp_config *nlp_config = pendulum_acados_get_nlp_config(capsule);
    ocp_nlp_dims *nlp_dims = pendulum_acados_get_nlp_dims(capsule);
    ocp_nlp_in *nlp_in = pendulum_acados_get_nlp_in(capsule);
    ocp_nlp_out *nlp_out = pendulum_acados_get_nlp_out(capsule);

    InputRealPtrsType in_sign;

    int N = PENDULUM_N;            

    // local buffer
    real_t buffer[25];
    double tmp_cpu_time;

    /* go through inputs */
    // lbx_0
    in_sign = ssGetInputPortRealSignalPtrs(S, 0);
    for (int i = 0; i < 4; i++)
        buffer[i] = (double)(*in_sign[i]);

    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "lbx", buffer);
    // ubx_0
    in_sign = ssGetInputPortRealSignalPtrs(S, 1);
    for (int i = 0; i < 4; i++)
        buffer[i] = (double)(*in_sign[i]);
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "ubx", buffer);

  
    // y_ref_0
    in_sign = ssGetInputPortRealSignalPtrs(S, 2);

    for (int i = 0; i < 5; i++)
        buffer[i] = (double)(*in_sign[i]);

    ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, 0, "yref", (void *) buffer);

  
    // y_ref - for stages 1 to N-1
    in_sign = ssGetInputPortRealSignalPtrs(S, 3);

    for (int ii = 1; ii < N; ii++)
    {
        for (int jj = 0; jj < 5; jj++)
            buffer[jj] = (double)(*in_sign[(ii-1)*5+jj]);
        ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, ii, "yref", (void *) buffer);
    }

  
    // y_ref_e
    in_sign = ssGetInputPortRealSignalPtrs(S, 4);

    for (int i = 0; i < 4; i++)
        buffer[i] = (double)(*in_sign[i]);

    ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, N, "yref", (void *) buffer);
    // lbu
    in_sign = ssGetInputPortRealSignalPtrs(S, 5);
    for (int ii = 0; ii < N; ii++)
    {
        for (int jj = 0; jj < 1; jj++)
            buffer[jj] = (double)(*in_sign[ii*1+jj]);
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, ii, "lbu", (void *) buffer);
    }
    // ubu
    in_sign = ssGetInputPortRealSignalPtrs(S, 6);
    for (int ii = 0; ii < N; ii++)
    {
        for (int jj = 0; jj < 1; jj++)
            buffer[jj] = (double)(*in_sign[ii*1+jj]);
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, ii, "ubu", (void *) buffer);
    }
    // lh_0
    in_sign = ssGetInputPortRealSignalPtrs(S, 7);
    for (int i = 0; i < 1; i++)
        buffer[i] = (double)(*in_sign[i]);
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "lh", buffer);
    // uh_0
    in_sign = ssGetInputPortRealSignalPtrs(S, 8);
    for (int i = 0; i < 1; i++)
        buffer[i] = (double)(*in_sign[i]);
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "uh", buffer);  
    // cost_W_0
    in_sign = ssGetInputPortRealSignalPtrs(S, 9);
    for (int i = 0; i < 25; i++)
        buffer[i] = (double)(*in_sign[i]);

    ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, 0, "W", buffer);  
    // cost_W
    in_sign = ssGetInputPortRealSignalPtrs(S, 10);
    for (int i = 0; i < 25; i++)
        buffer[i] = (double)(*in_sign[i]);

    for (int ii = 1; ii < N; ii++)
        ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, ii, "W", buffer);  
    // cost_W_e
    in_sign = ssGetInputPortRealSignalPtrs(S, 11);
    for (int i = 0; i < 16; i++)
        buffer[i] = (double)(*in_sign[i]);

    ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, N, "W", buffer);  
    // reset_solver
    in_sign = ssGetInputPortRealSignalPtrs(S, 12);
    double reset = (double)(*in_sign[0]);
    if (reset)
    {
        pendulum_acados_reset(capsule, 1);
    }  
    // x_init
    in_sign = ssGetInputPortRealSignalPtrs(S, 13);
    for (int ii = 0; ii < 21; ii++)
    {
        for (int jj = 0; jj < 4; jj++)
            buffer[jj] = (double)(*in_sign[(ii)*4+jj]);
        ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, ii, "x", (void *) buffer);
    }


    /* call solver */
    int rti_phase = 0;
    ocp_nlp_solver_opts_set(nlp_config, capsule->nlp_opts, "rti_phase", &rti_phase);
    int acados_status = pendulum_acados_solve(capsule);
    // get time
    ocp_nlp_get(nlp_config, capsule->nlp_solver, "time_tot", (void *) buffer);
    tmp_cpu_time = buffer[0];

    /* set outputs */
    // assign pointers to output signals
    real_t *out_u0, *out_utraj, *out_xtraj, *out_ztraj, *out_status, *out_sqp_iter, *out_KKT_res, *out_KKT_residuals, *out_x1, *out_cpu_time, *out_cpu_time_sim, *out_cpu_time_qp, *out_cpu_time_lin, *out_cost_value;
    int tmp_int;
    out_u0 = ssGetOutputPortRealSignal(S, 0);
    ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, 0, "u", (void *) out_u0);
    out_utraj = ssGetOutputPortRealSignal(S, 1);
    for (int ii = 0; ii < N; ii++)
        ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, ii,
                        "u", (void *) (out_utraj + ii * 1));

  

    out_xtraj = ssGetOutputPortRealSignal(S, 2);
    for (int ii = 0; ii < 21; ii++)
        ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, ii,
                        "x", (void *) (out_xtraj + ii * 4));

  
    out_status = ssGetOutputPortRealSignal(S, 3);
    *out_status = (real_t) acados_status;
    out_cost_value = ssGetOutputPortRealSignal(S, 4);
    ocp_nlp_eval_cost(capsule->nlp_solver, nlp_in, nlp_out);
    ocp_nlp_get(nlp_config, capsule->nlp_solver, "cost_value", (void *) out_cost_value);
    out_KKT_residuals = ssGetOutputPortRealSignal(S, 5);
    ocp_nlp_get(nlp_config, capsule->nlp_solver, "res_stat", (void *) &out_KKT_residuals[0]);
    ocp_nlp_get(nlp_config, capsule->nlp_solver, "res_eq", (void *) &out_KKT_residuals[1]);
    ocp_nlp_get(nlp_config, capsule->nlp_solver, "res_ineq", (void *) &out_KKT_residuals[2]);
    ocp_nlp_get(nlp_config, capsule->nlp_solver, "res_comp", (void *) &out_KKT_residuals[3]);
    out_x1 = ssGetOutputPortRealSignal(S, 6);
    ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, 1, "x", (void *) out_x1);
    out_cpu_time = ssGetOutputPortRealSignal(S, 7);
    out_cpu_time[0] = tmp_cpu_time;
    out_sqp_iter = ssGetOutputPortRealSignal(S, 8);
    // get sqp iter
    ocp_nlp_get(nlp_config, capsule->nlp_solver, "sqp_iter", (void *) &tmp_int);
    *out_sqp_iter = (real_t) tmp_int;

}

static void mdlTerminate(SimStruct *S)
{
    pendulum_solver_capsule *capsule = ssGetUserData(S);

    pendulum_acados_free(capsule);
    pendulum_acados_free_capsule(capsule);
}


#ifdef  MATLAB_MEX_FILE
#include "simulink.c"
#else
#include "cg_sfun.h"
#endif
