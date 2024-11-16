%% Testing for the Inverse KKT ->
clear all
close all
clc
GENERATE_C_CODE = 0;model_name = 'BILOOOO';
% check that env.sh has been run
env_run = getenv('ENV_RUN');
if (~strcmp(env_run, 'true'))
error('env.sh has not been sourced! Before executing this example, run: source env.sh');
end
%% Options
compile_interface = 'auto'; % true, false
codgen_model = 'true'; % true, false
% simulation
sim_method = 'erk';sim_sens_forw = 'false';sim_num_stages = 4; sim_num_steps = 4;
ocp_N = 100;nlp_solver = 'sqp_rti';nlp_solver_exact_hessian = 'false';regularize_method = 'project_reduc_hess';nlp_solver_max_iter = 500; %sqp max iteration probably
qp_solver = 'partial_condensing_hpipm';qp_solver_iter_max = 100;qp_solver_cond_N = 5;qp_solver_warm_start = 0;qp_solver_cond_ric_alg = 0;qp_solver_ric_alg = 0;
ocp_sim_method = 'erk';ocp_sim_method_num_stages = 4;ocp_sim_method_num_steps = 4;
cost_type = 'ext_cost';
%% 
model=Lagrangian();h = 0.02;T = ocp_N*h;
nx = model.nx;
x0 = ones(1,11);
%% acados ocp model
ocp_model = acados_ocp_model();ocp_model.set('name', model_name);ocp_model.set('T', T);ocp_model.set('sym_x', model.sym_x);ocp_model.set('sym_p', model.sym_p);

ocp_model.set('sym_u',model.sym_u);ocp_model.set('sym_xdot',model.sym_xdot);

ocp_model.set('cost_type', cost_type);ocp_model.set('cost_expr_ext_cost', model.expr_ext_cost);
ocp_model.set('cost_type_e', cost_type);ocp_model.set('cost_expr_ext_cost_e', model.expr_ext_cost_e);

ocp_model.set('dyn_type', 'explicit');ocp_model.set('dyn_expr_f', model.expr_f_expl);
ocp_model.set('constr_x0', x0);

%% acados ocp opts
ocp_opts = acados_ocp_opts();ocp_opts.set('compile_interface', compile_interface);ocp_opts.set('codgen_model', codgen_model);ocp_opts.set('param_scheme_N', ocp_N);ocp_opts.set('nlp_solver', nlp_solver);ocp_opts.set('nlp_solver_exact_hessian', nlp_solver_exact_hessian);ocp_opts.set('regularize_method', regularize_method);
if (strcmp(nlp_solver, 'sqp'))
	ocp_opts.set('nlp_solver_max_iter', nlp_solver_max_iter);
end
ocp_opts.set('qp_solver', qp_solver);
if (strcmp(qp_solver, 'partial_condensing_hpipm'))
	ocp_opts.set('qp_solver_cond_N', qp_solver_cond_N);ocp_opts.set('qp_solver_cond_ric_alg', qp_solver_cond_ric_alg);ocp_opts.set('qp_solver_ric_alg', qp_solver_ric_alg);ocp_opts.set('qp_solver_warm_start', qp_solver_warm_start);
end
ocp_opts.set('qp_solver_iter_max', qp_solver_iter_max);
ocp_opts.set('sim_method', ocp_sim_method);ocp_opts.set('sim_method_num_stages', ocp_sim_method_num_stages);ocp_opts.set('sim_method_num_steps', ocp_sim_method_num_steps);
ocp_opts.opts_struct
%% acados ocp
ocp = acados_ocp(ocp_model, ocp_opts);
if GENERATE_C_CODE == 1
    ocp.generate_c_code()
end