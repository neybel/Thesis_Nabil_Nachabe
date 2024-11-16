%% Writing an external] cost function using Acados
%% 
clear all
clc
close all
model_name = 'BILOOOO';
GENERATE_C_CODE = 0;


% Check that env.sh has been run
env_run = getenv('ENV_RUN');
if (~strcmp(env_run, 'true'))
	disp('ERROR: env.sh has not been sourced! Before executing this example, run:');
	disp('source env.sh');
	return;
end
%% Options
compile_interface = 'auto'; % true, false
codgen_model = 'true'; % true, false
% simulation
sim_method = 'erk'; % erk, irk, irk_gnsf
sim_sens_forw = 'false'; % true, false
sim_num_stages = 4; 
sim_num_steps = 4;
% ocp
ocp_N = 100;
% nlp_solver = 'sqp_rti';
%nlp_solver_exact_hessian = 'false';
nlp_solver = 'sqp'; % sqp, sqp_rti
nlp_solver_exact_hessian = 'false';
regularize_method = 'project_reduc_hess'; % no_regularize, project,...
	% project_reduc_hess, mirror, convexify
%regularize_method = 'mirror';
%regularize_method = 'convexify';
nlp_solver_max_iter = 500; %sqp max iteration probably
qp_solver = 'partial_condensing_hpipm';
% full_condensing_hpipm, partial_condensing_hpipm, full_condensing_qpoases
qp_solver_iter_max = 100;
qp_solver_cond_N = 5;
qp_solver_warm_start = 0;
qp_solver_cond_ric_alg = 0; % 0: dont factorize hessian in the condensing; 1: factorize
qp_solver_ric_alg = 0; % HPIPM specific
ocp_sim_method = 'erk'; % erk, irk, irk_gnsf
% ocp_sim_method = 'irk';
ocp_sim_method_num_stages = 4;
ocp_sim_method_num_steps = 4;
cost_type = 'ext_cost'; % linear_ls, ext_cost
%%

model = kinematic_single_track_model();

h = 0.1;
T = ocp_N*h; % horizon length time

% dims
nx = model.nx;
nu = model.nu;

ny = nu+nx; % number of outputs in lagrange term
ny_e = nx; % number of outputs in mayer term

ng = 0; % number of general linear constraints intermediate stages
ng_e = 0; % number of general linear constraints final stage
nbx = 0; % number of bounds on state x
%%
x_ref = [5; 0; pi/4]; % example reference state

%%
linear_constraints = 1; % 1: encode control bounds as bounds (efficient)
    % 0: encode control bounds as external CasADi functions
if linear_constraints
	nbu = nu;
	nh = 0;
	nh_e = 0;
else
% 	nbu = 0;
    nbu = nu;
	nh = nu;
	nh_e = 0;
end

%%
yr = zeros(ny, 1); % output reference in lagrange term
yr_e = [10;0;0];
x0 = [0; 0; 0];
Jbu = zeros(nbu, nu); for ii=1:nbu Jbu(ii,ii)=1.0; end
lbu = [0;-pi];
ubu = [10;+pi];
%% acados ocp model
ocp_model = acados_ocp_model();
ocp_model.set('name', model_name);
ocp_model.set('T', T);

% symbolics
ocp_model.set('sym_x', model.sym_x);
if isfield(model, 'sym_u')
	ocp_model.set('sym_u', model.sym_u);
end
if isfield(model, 'sym_xdot')
	ocp_model.set('sym_xdot', model.sym_xdot);
end

% cost
ocp_model.set('cost_type', cost_type);
ocp_model.set('cost_type_e', cost_type);
if (strcmp(cost_type, 'linear_ls'))
	ocp_model.set('cost_Vu', Vu);
	ocp_model.set('cost_Vx', Vx);
	ocp_model.set('cost_Vx_e', Vx_e);
	ocp_model.set('cost_W', W);
	ocp_model.set('cost_W_e', W_e);
	ocp_model.set('cost_y_ref', yr);
	ocp_model.set('cost_y_ref_e', yr_e);
elseif (strcmp(cost_type, 'ext_cost'))
	ocp_model.set('cost_expr_ext_cost', model.expr_ext_cost);
	ocp_model.set('cost_expr_ext_cost_e', model.expr_ext_cost_e);
end

% dynamics
if (strcmp(ocp_sim_method, 'erk'))
	ocp_model.set('dyn_type', 'explicit');
	ocp_model.set('dyn_expr_f', model.expr_f_expl);
else % irk
	ocp_model.set('dyn_type', 'implicit');
	ocp_model.set('dyn_expr_f', model.expr_f_impl);
end
ocp_model.set('constr_x0', x0);
%%
ocp_model.set('constr_Jbu', Jbu);
ocp_model.set('constr_lbu', lbu);
ocp_model.set('constr_ubu', ubu);
ocp_model.model_struct
%% acados ocp opts
ocp_opts = acados_ocp_opts();
ocp_opts.set('compile_interface', compile_interface);
ocp_opts.set('codgen_model', codgen_model);
ocp_opts.set('param_scheme_N', ocp_N);
ocp_opts.set('nlp_solver', nlp_solver);
ocp_opts.set('nlp_solver_exact_hessian', nlp_solver_exact_hessian);
ocp_opts.set('regularize_method', regularize_method);
if (strcmp(nlp_solver, 'sqp'))
	ocp_opts.set('nlp_solver_max_iter', nlp_solver_max_iter);
end
ocp_opts.set('qp_solver', qp_solver);
if (strcmp(qp_solver, 'partial_condensing_hpipm'))
	ocp_opts.set('qp_solver_cond_N', qp_solver_cond_N);
	ocp_opts.set('qp_solver_cond_ric_alg', qp_solver_cond_ric_alg);
	ocp_opts.set('qp_solver_ric_alg', qp_solver_ric_alg);
	ocp_opts.set('qp_solver_warm_start', qp_solver_warm_start);
end
ocp_opts.set('qp_solver_iter_max', qp_solver_iter_max);
ocp_opts.set('sim_method', ocp_sim_method);
ocp_opts.set('sim_method_num_stages', ocp_sim_method_num_stages);
ocp_opts.set('sim_method_num_steps', ocp_sim_method_num_steps);

ocp_opts.opts_struct
%% acados sim model
%% create ocp
ocp = acados_ocp(ocp_model, ocp_opts);

if GENERATE_C_CODE == 1
    ocp.generate_c_code()
end
%%
sim_model = acados_sim_model();
% symbolics
sim_model.set('sym_x', model.sym_x);
if isfield(model, 'sym_u')
	sim_model.set('sym_u', model.sym_u);
end
if isfield(model, 'sym_xdot')
	sim_model.set('sym_xdot', model.sym_xdot);
end
% model
sim_model.set('T', T/ocp_N);
if (strcmp(sim_method, 'erk'))
	sim_model.set('dyn_type', 'explicit');
	sim_model.set('dyn_expr_f', model.expr_f_expl);
else % irk
	sim_model.set('dyn_type', 'implicit');
	sim_model.set('dyn_expr_f', model.expr_f_impl);
end

%sim_model.model_struct
%% acados sim opts
sim_opts = acados_sim_opts();
sim_opts.set('compile_interface', compile_interface);
sim_opts.set('codgen_model', codgen_model);
sim_opts.set('num_stages', sim_num_stages);
sim_opts.set('num_steps', sim_num_steps);
sim_opts.set('method', sim_method);
sim_opts.set('sens_forw', sim_sens_forw);
%% acados sim

sim = acados_sim(sim_model,sim_opts);


% N_sim = ocp_N;
N_sim = 1;
x_sim = zeros(nx, N_sim+1);
x_sim(:,1) = x0; % initial state
u_sim = zeros(nu, N_sim);


x_traj_init=[linspace(0,10,ocp_N+1);linspace(0,0,ocp_N+1);(pi/4)*zeros(1,ocp_N+1)];
u_traj_init = zeros(nu, ocp_N);

    tic;

for ii= 1 : N_sim

	% set x0
	ocp.set('constr_x0', x_sim(:,ii));
	ocp.set('init_x', x_traj_init);
	ocp.set('init_u', u_traj_init);
	ocp.solve();
	if 1
		status = ocp.get('status');
		sqp_iter = ocp.get('sqp_iter');
		time_tot = ocp.get('time_tot');
		time_lin = ocp.get('time_lin');
		time_qp_sol = ocp.get('time_qp_sol');

		fprintf('\nstatus = %d, sqp_iter = %d, time_int = %f [ms] (time_lin = %f [ms], time_qp_sol = %f [ms])\n',...
            status, sqp_iter, time_tot*1e3, time_lin*1e3, time_qp_sol*1e3);
        if status~=0
            disp('acados ocp solver failed');
        end
	end
x_traj = ocp.get('x');
u_traj = ocp.get('u');
x_traj_init = [x_traj_init(:,2:end), x_traj_init(:,end)];
u_traj_init = [u_traj_init(:,2:end), u_traj_init(:,end)];
u_sim(:,ii) = ocp.get('u', 0);
sim.set('x',x_sim(:,ii));
sim.set('u', u_sim(:,ii));
sim.solve();
x_sim(:,ii+1) = sim.get('xn');
end
avg_time_solve = toc/N_sim
