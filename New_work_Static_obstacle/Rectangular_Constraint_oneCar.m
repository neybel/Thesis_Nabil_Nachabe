%% Rectangular Constraint for 1 car encoutering an obstacle on its way
%%
clear all
close all
clc
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
ocp_N = 50;
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
cost_type = 'linear_ls'; % linear_ls, ext_cost
% cost_type = 'ext_cost'; % linear_ls, ext_cost
%% create model entries
model = kinematic_single_track_model_with_acceleration_and_steering_rat()
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
% cost
Vu = zeros(ny, nu); for ii=1:nu  Vu(ii,ii)=1.0; end % input-to-output matrix in lagrange term
Vx = zeros(ny, nx); for ii=1:nx Vx(nu+ii,ii)=1.0; end % state-to-output matrix in lagrange term
Vx_e = zeros(ny_e, nx); for ii=1:nx Vx_e(ii,ii)=1.0; end % state-to-output matrix in mayer term
W = eye(ny); % weight matrix in lagrange term
W(1,1) = 1e0; W(2,2) = 1e0; W(3,3) = 1e3; W(4,4) = 1e2; W(5,5)= 1e1; W(6,6)= 1e-1; W(7,7)= 1e-1;
W_e = eye(5);
W_e(1,1) = 1e3; W_e(2,2)= 1e2; W_e(3,3)=1e1; W_e(4,4)=1e1; W_e(5,5)=1e1;
yr = zeros(ny, 1);
yr_e = [100;0;0;0;0];
% constraints
x0 = [0; 0; 0;10;0];
%Jbx = zeros(nbx, nx); for ii=1:nbx Jbx(ii,ii)=1.0; end
%lbx = -4*ones(nbx, 1);
%ubx =  4*ones(nbx, 1);
Jbu = zeros(nbu, nu); for ii=1:nbu Jbu(ii,ii)=1.0; end
lbu = [-11.5;-0.4];
ubu = [+11.5;+0.4];
%% Rectangular Constraint
obstacle_center = [50;0];  % Position of the obstacle center
dsafe = 0.50;
w = 1.5; L=2.73;
x = model.sym_x(1:3);
a1 = sin(x(3))*obstacle_center(1) - cos(x(3))*obstacle_center(2) -(w+dsafe)/2 -sin(x(3))*x(1) + cos(x(3))*x(2);
a2 = -sin(x(3))*obstacle_center(1) + cos(x(3))*obstacle_center(2) -(w+dsafe)/2 +sin(x(3))*x(1) - cos(x(3))*x(2);
a3 = cos(x(3))*obstacle_center(1) + sin(x(3))*obstacle_center(2) -(L+dsafe)/2 +cos(x(3))*x(1) + sin(x(3))*x(2);
a4 = -cos(x(3))*obstacle_center(1) - sin(x(3))*obstacle_center(2) -(L+dsafe)/2 + cos(x(3))*x(1) + sin(x(3))*x(2);
C = [a1;a2;a3;a4];
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
% constraints
ocp_model.set('constr_x0', x0);
if 1
if (nh>0)
ocp_model.set('constr_expr_h',constr.expr_h);
ocp_model.set('constr_lh', zeros(4,1)); % Ensure dh_dt + alpha * h is always positive
ocp_model.set('constr_uh', inf(4,1));

% nsh = 1;
% Jsh = 1;
% ocp_model.set('constr_Jsh', Jsh);
% % Set cost on slack
% % L1 slack (linear term)
% ocp_model.set('cost_zl', 1e0 * ones(nsh,1));
% ocp_model.set('cost_zu', 0* ones(nsh,1));
% % L2 slack (squared term)
% ocp_model.set('cost_Zl', 1e0 * ones(nsh,nsh));
% ocp_model.set('cost_Zu', 0 * ones(nsh,nsh));
end
end
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
N_sim = 120;
x_sim = zeros(nx, N_sim+1);
x_sim(:,1) = x0; % initial state
u_sim = zeros(nu, N_sim);


x_traj_init=[linspace(0,100,ocp_N+1);linspace(0,0,ocp_N+1);(pi/4)*zeros(1,ocp_N+1);zeros(1,ocp_N+1);zeros(1,ocp_N+1)];
u_traj_init = zeros(nu, ocp_N);

    tic;

for ii= 1 : N_sim
    for jj = 0 : ocp_N - 1
        if jj ~=ocp_N -1 
        kk = jj + 1;
        else
            kk = ocp_N -1; 
        end
%     yref = [x_traj_init(1,jj + 1);x_traj_init(2,jj+1);x_traj_init(3,jj+1);x_traj_init(4,jj+1);x_traj_init(5,jj+1);u_traj_init(1,kk);u_traj_init(2,kk)];
    yref = [u_traj_init(1,kk);u_traj_init(2,kk);x_traj_init(1,jj + 1);x_traj_init(2,jj+1);x_traj_init(3,jj+1);x_traj_init(4,jj+1);x_traj_init(5,jj+1)];
    ocp.set('cost_y_ref', yref, jj);
    end
    ocp.set('cost_y_ref_e', yr_e, ocp_N);
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
%%%%%% Montitor the value of the constraint %%%%%%%

%     x_curr = x_sim(:, ii);
%     u_curr = u_sim(:, ii);
%     dist_test = (x_curr(1) - obstacle_center(1))^2 + (x_curr(2) - obstacle_center(2))^2;
%     h_test = dist_test - (radius_car + radius_obs + safety_distance)^2;
%     a1_test = x_curr(1) - obstacle_center(1); a2_test = u_curr(1)*cos(x_curr(3)); b1_test = x_curr(2) - obstacle_center(2); b2_test = u_curr(1)*sin(x_curr(3));
%     dh_dt_test = 2*((a1_test*a2_test) + (b1_test*b2_test));
% 
%     constr_value = -dh_dt_test + alpha * h_test;
%     disp(['Iteration ', num2str(ii), ': Constraint value = ', num2str(constr_value)]);
%%%%%
end

avg_time_solve = toc/N_sim
%%
if 1
    figure(100)
plot(x_sim(1,:),x_sim(2,:),'r*-'); title('Trajectory');
% hold on; plot(obstacle_center(1),obstacle_center(2),'ko',LineWidth=5); legend('car 1','Obstacle'); title('Trajectory XY'); xlabel('x'); ylabel('y');

figure (200)
subplot(2,1,1)
plot(1:N_sim,u_sim(1,:)); title('Acceleration');
subplot(2,1,2)
plot(1:N_sim,u_sim(2,:)); title ('Steering Rate');


%Distance evaluation
% Extract positions of each car from x_sim
xa = x_sim(1,:);
ya = x_sim(2,:);

% Calculate distance between the two cars
% distance = sqrt((obstacle_center(1) - xa).^2 + (obstacle_center(2) - ya).^2);
% [mindistance,index] = min(distance)
end
plot(1:N_sim+1,x_sim(4,:))
