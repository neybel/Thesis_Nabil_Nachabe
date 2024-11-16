function [ocp,sim] = SettingUp()
GENERATE_C_CODE = 0;
model_name = 'BILOOOO';
% check that env.sh has been run
env_run = getenv('ENV_RUN');
if (~strcmp(env_run, 'true'))
	error('env.sh has not been sourced! Before executing this example, run: source env.sh');
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
ocp_N = 20;
% nlp_solver = 'sqp_rti';
%nlp_solver_exact_hessian = 'false';
nlp_solver = 'sqp_rti'; % sqp, sqp_rti
nlp_solver_exact_hessian = 'false';
regularize_method = 'project_reduc_hess'; % no_regularize, project,...
	% project_reduc_hess, mirror, convexify
%regularize_method = 'mirror';
%regularize_method = 'convexify';
nlp_solver_max_iter = 50; %sqp max iteration probably 500
qp_solver = 'partial_condensing_hpipm';
% full_condensing_hpipm, partial_condensing_hpipm, full_condensing_qpoases
qp_solver_iter_max = 100;
qp_solver_cond_N = 5;
qp_solver_warm_start = 1; % 0
qp_solver_cond_ric_alg = 0; % 0: dont factorize hessian in the condensing; 1: factorize
qp_solver_ric_alg = 0; % HPIPM specific
ocp_sim_method = 'erk'; % erk, irk, irk_gnsf
% ocp_sim_method = 'irk';
ocp_sim_method_num_stages = 4;
ocp_sim_method_num_steps = 4;
cost_type = 'ext_cost'; % linear_ls, ext_cost
% cost_type = 'ext_cost'; % linear_ls, ext_cost
%% create model entries
model = DYNtwocarExternalCoupling_variableWeights();
h = 0.05;
T = ocp_N*h; % horizon length time
% dims
nx = model.nx;
nu = model.nu;
ny = nu+nx; % number of outputs in lagrange term
ny_e = nx; % number of outputs in mayer term
ng = 0; % number of general linear constraints intermediate stages
ng_e = 0; % number of general linear constraints final stage
nbx = 1; % number of bounds on state x
linear_constraints = 0; % 1: encode control bounds as bounds (efficient)
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
% cost
% constraints
x0 = [110; -5; pi/2;10;0;115;0;pi;10;0];
% Jbx = zeros(1, nx); for ii=1:nbx Jbx(ii,ii)=1.0; end
% lbx = 0*ones(nbx, 1);
% ubx =  120*ones(nbx, 1);
Jbu = zeros(nbu, nu); for ii=1:nbu Jbu(ii,ii)=1.0; end
lbu = [-11.5;-1;-11.5;-1];
ubu = [+11.5;+1;+11.5;+1];
%% Bound on State (Velocity)
% Define the index for the velocity state
idx_v1 = 4;
% idx_v2 = 9;
% Define Jbx to constrain the velocity (4th state variable)
Jbx = zeros(1, nx); 
Jbx(idx_v1) = 1; % only constrain the velocity state
% Jbx(idx_v2) = 1; % only constrain the velocity state
% Set lower and upper bounds for velocity
lbx = 0; % minimum velocity
ubx = 20; % no upper bound constraint on velocity
%% New CBF 
x1 = model.sym_x(1:3);
v1 = model.sym_x(4);
x2 = model.sym_x(6:8);
v2 = model.sym_x(9);
global alpha 
global dsafe
alpha = 10.0; dsafe= 1.0;
dist_squared = ((x1(1) - x2(1))^2) + ((x1(2) - x2(2))^2) - dsafe;
h =dist_squared;
a1 = x1(1) - x2(1); a2 = v1*cos(x1(3))-v2*cos(x2(3)); b1 = x1(2) - x2(2); b2 = v1*sin(x1(3))-v2*sin(x2(3));
dh_dt = 2*((a1*a2) + (b1*b2));
C = + dh_dt + alpha * h; % choice between + or -
constr.expr_h = C;

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
if isfield(model, 'sym_p')
    ocp_model.set('sym_p', model.sym_p);
end

% cost
ocp_model.set('cost_type', cost_type);
ocp_model.set('cost_type_e', cost_type);
ocp_model.set('cost_expr_ext_cost', model.expr_ext_cost);
ocp_model.set('cost_expr_ext_cost_e', model.expr_ext_cost_e);
% dynamics
if (strcmp(ocp_sim_method, 'erk'))
	ocp_model.set('dyn_type', 'explicit');
	ocp_model.set('dyn_expr_f', model.expr_f_expl);
else % irk
	ocp_model.set('dyn_type', 'implicit');
	ocp_model.set('dyn_expr_f', model.expr_f_impl);
end
% constraints
ocp_model.set('constr_x0', x0);
if 1
if (nh>0)
ocp_model.set('constr_expr_h',constr.expr_h);
ocp_model.set('constr_lh', 0); % Ensure dh_dt + alpha * h is always positive
ocp_model.set('constr_uh', 1e8);
nsh = 1;
Jsh = 1;
end
end
ocp_model.set('constr_Jbu', Jbu);
ocp_model.set('constr_lbu', lbu);
ocp_model.set('constr_ubu', ubu);
ocp_model.set('constr_Jbx', Jbx);
ocp_model.set('constr_lbx', lbx);
ocp_model.set('constr_ubx', ubx);
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
%% acados ocp
% create ocp
ocp = acados_ocp(ocp_model, ocp_opts);

if GENERATE_C_CODE == 1
    ocp.generate_c_code()
end
%% acados sim model
sim_model = acados_sim_model();
% symbolics
sim_model.set('sym_x', model.sym_x);
if isfield(model, 'sym_u')
	sim_model.set('sym_u', model.sym_u);
end
if isfield(model, 'sym_xdot')
	sim_model.set('sym_xdot', model.sym_xdot);
end
if isfield(model, 'sym_p')
    sim_model.set('sym_p', model.sym_p);
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

sim = acados_sim(sim_model,sim_opts);
end