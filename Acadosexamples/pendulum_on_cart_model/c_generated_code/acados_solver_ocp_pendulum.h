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

#ifndef ACADOS_SOLVER_ocp_pendulum_H_
#define ACADOS_SOLVER_ocp_pendulum_H_

#include "acados/utils/types.h"

#include "acados_c/ocp_nlp_interface.h"
#include "acados_c/external_function_interface.h"

#define OCP_PENDULUM_NX     4
#define OCP_PENDULUM_NZ     0
#define OCP_PENDULUM_NU     1
#define OCP_PENDULUM_NP     0
#define OCP_PENDULUM_NBX    0
#define OCP_PENDULUM_NBX0   4
#define OCP_PENDULUM_NBU    1
#define OCP_PENDULUM_NSBX   0
#define OCP_PENDULUM_NSBU   0
#define OCP_PENDULUM_NSH    0
#define OCP_PENDULUM_NSH0   0
#define OCP_PENDULUM_NSG    0
#define OCP_PENDULUM_NSPHI  0
#define OCP_PENDULUM_NSHN   0
#define OCP_PENDULUM_NSGN   0
#define OCP_PENDULUM_NSPHIN 0
#define OCP_PENDULUM_NSPHI0 0
#define OCP_PENDULUM_NSBXN  0
#define OCP_PENDULUM_NS     0
#define OCP_PENDULUM_NS0    0
#define OCP_PENDULUM_NSN    0
#define OCP_PENDULUM_NG     0
#define OCP_PENDULUM_NBXN   0
#define OCP_PENDULUM_NGN    0
#define OCP_PENDULUM_NY0    5
#define OCP_PENDULUM_NY     5
#define OCP_PENDULUM_NYN    4
#define OCP_PENDULUM_N      100
#define OCP_PENDULUM_NH     0
#define OCP_PENDULUM_NHN    0
#define OCP_PENDULUM_NH0    0
#define OCP_PENDULUM_NPHI0  0
#define OCP_PENDULUM_NPHI   0
#define OCP_PENDULUM_NPHIN  0
#define OCP_PENDULUM_NR     0

#ifdef __cplusplus
extern "C" {
#endif


// ** capsule for solver data **
typedef struct ocp_pendulum_solver_capsule
{
    // acados objects
    ocp_nlp_in *nlp_in;
    ocp_nlp_out *nlp_out;
    ocp_nlp_out *sens_out;
    ocp_nlp_solver *nlp_solver;
    void *nlp_opts;
    ocp_nlp_plan_t *nlp_solver_plan;
    ocp_nlp_config *nlp_config;
    ocp_nlp_dims *nlp_dims;

    // number of expected runtime parameters
    unsigned int nlp_np;

    /* external functions */
    // dynamics

    external_function_param_casadi *forw_vde_casadi;
    external_function_param_casadi *expl_ode_fun;




    // cost






    // constraints







} ocp_pendulum_solver_capsule;

ACADOS_SYMBOL_EXPORT ocp_pendulum_solver_capsule * ocp_pendulum_acados_create_capsule(void);
ACADOS_SYMBOL_EXPORT int ocp_pendulum_acados_free_capsule(ocp_pendulum_solver_capsule *capsule);

ACADOS_SYMBOL_EXPORT int ocp_pendulum_acados_create(ocp_pendulum_solver_capsule * capsule);

ACADOS_SYMBOL_EXPORT int ocp_pendulum_acados_reset(ocp_pendulum_solver_capsule* capsule, int reset_qp_solver_mem);

/**
 * Generic version of ocp_pendulum_acados_create which allows to use a different number of shooting intervals than
 * the number used for code generation. If new_time_steps=NULL and n_time_steps matches the number used for code
 * generation, the time-steps from code generation is used.
 */
ACADOS_SYMBOL_EXPORT int ocp_pendulum_acados_create_with_discretization(ocp_pendulum_solver_capsule * capsule, int n_time_steps, double* new_time_steps);
/**
 * Update the time step vector. Number N must be identical to the currently set number of shooting nodes in the
 * nlp_solver_plan. Returns 0 if no error occurred and a otherwise a value other than 0.
 */
ACADOS_SYMBOL_EXPORT int ocp_pendulum_acados_update_time_steps(ocp_pendulum_solver_capsule * capsule, int N, double* new_time_steps);
/**
 * This function is used for updating an already initialized solver with a different number of qp_cond_N.
 */
ACADOS_SYMBOL_EXPORT int ocp_pendulum_acados_update_qp_solver_cond_N(ocp_pendulum_solver_capsule * capsule, int qp_solver_cond_N);
ACADOS_SYMBOL_EXPORT int ocp_pendulum_acados_update_params(ocp_pendulum_solver_capsule * capsule, int stage, double *value, int np);
ACADOS_SYMBOL_EXPORT int ocp_pendulum_acados_update_params_sparse(ocp_pendulum_solver_capsule * capsule, int stage, int *idx, double *p, int n_update);

ACADOS_SYMBOL_EXPORT int ocp_pendulum_acados_solve(ocp_pendulum_solver_capsule * capsule);
ACADOS_SYMBOL_EXPORT int ocp_pendulum_acados_free(ocp_pendulum_solver_capsule * capsule);
ACADOS_SYMBOL_EXPORT void ocp_pendulum_acados_print_stats(ocp_pendulum_solver_capsule * capsule);
ACADOS_SYMBOL_EXPORT int ocp_pendulum_acados_custom_update(ocp_pendulum_solver_capsule* capsule, double* data, int data_len);


ACADOS_SYMBOL_EXPORT ocp_nlp_in *ocp_pendulum_acados_get_nlp_in(ocp_pendulum_solver_capsule * capsule);
ACADOS_SYMBOL_EXPORT ocp_nlp_out *ocp_pendulum_acados_get_nlp_out(ocp_pendulum_solver_capsule * capsule);
ACADOS_SYMBOL_EXPORT ocp_nlp_out *ocp_pendulum_acados_get_sens_out(ocp_pendulum_solver_capsule * capsule);
ACADOS_SYMBOL_EXPORT ocp_nlp_solver *ocp_pendulum_acados_get_nlp_solver(ocp_pendulum_solver_capsule * capsule);
ACADOS_SYMBOL_EXPORT ocp_nlp_config *ocp_pendulum_acados_get_nlp_config(ocp_pendulum_solver_capsule * capsule);
ACADOS_SYMBOL_EXPORT void *ocp_pendulum_acados_get_nlp_opts(ocp_pendulum_solver_capsule * capsule);
ACADOS_SYMBOL_EXPORT ocp_nlp_dims *ocp_pendulum_acados_get_nlp_dims(ocp_pendulum_solver_capsule * capsule);
ACADOS_SYMBOL_EXPORT ocp_nlp_plan_t *ocp_pendulum_acados_get_nlp_plan(ocp_pendulum_solver_capsule * capsule);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif  // ACADOS_SOLVER_ocp_pendulum_H_
