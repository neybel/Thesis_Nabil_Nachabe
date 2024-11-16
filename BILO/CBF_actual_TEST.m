%% Scnario of 2 cars with adopting penalizing collision inside the cost
% function
% Fixed some problems but it has to be checked again, try to play with the
% weights 
% Eventually I need to move to constrained optimization eitherway. 
%%
clear all; close all; clc
GENERATE_C_CODE = 0; model_name = 'BILOOOO';env_run = getenv('ENV_RUN');
if (~strcmp(env_run, 'true'))
	error('env.sh has not been sourced! Before executing this example, run: source env.sh');
end
%% Options
compile_interface = 'auto'; % true, false
codgen_model = 'true'; % true, false
% simulation
sim_method = 'erk'; % erk, irk, irk_gnsf
sim_sens_forw = 'false'; % true, false
sim_num_stages = 4;sim_num_steps = 4;
% ocp
ocp_N = 25;
nlp_solver = 'sqp'; % sqp, sqp_rti
nlp_solver_exact_hessian = 'false';
regularize_method = 'project_reduc_hess'; % no_regularize, project,...
	% project_reduc_hess, mirror, convexify
%regularize_method = 'mirror';
%regularize_method = 'convexify';
nlp_solver_max_iter = 500; %sqp max iteration probably
qp_solver = 'partial_condensing_hpipm';
        % full_condensing_hpipm, partial_condensing_hpipm, full_condensing_qpoases
qp_solver_iter_max = 100;qp_solver_cond_N = 5;qp_solver_warm_start = 0;qp_solver_cond_ric_alg = 0; % 0: dont factorize hessian in the condensing; 1: factorize
qp_solver_ric_alg = 0; % HPIPM specific
ocp_sim_method = 'erk'; % erk, irk, irk_gnsf
% ocp_sim_method = 'irk';
ocp_sim_method_num_stages = 4;ocp_sim_method_num_steps = 4;
cost_type = 'linear_ls'; % linear_ls, ext_cost
% cost_type = 'ext_cost'; % linear_ls, ext_cost
%% create model entries
model = kinematic_double_track_model_twocars()                               
h = 0.1;T = ocp_N*h; % horizon length time
nx = model.nx;nu = model.nu;ny = nu+nx; % number of outputs in lagrange term
ny_e = nx; % number of outputs in mayer term
ng = 0; % number of general linear constraints intermediate stages
ng_e = 0; % number of general linear constraints final stage
nbx = 0; % number of bounds on state x
linear_constraints = 0; 
if linear_constraints
	nbu = nu;nh = 0;nh_e = 0;
else
	nbu = nu;nh = 1;nh_e = 0;
end
Vu = zeros(ny, nu); for ii=1:nu  Vu(ii,ii)=1.0; end % input-to-output matrix in lagrange term
Vx = zeros(ny, nx); for ii=1:nx Vx(nu+ii,ii)=1.0; end % state-to-output matrix in lagrange term
Vx_e = zeros(ny_e, nx); for ii=1:nx Vx_e(ii,ii)=1.0; end % state-to-output matrix in mayer term
W = eye(ny); % weight matrix in lagrange term
% W(1,1) = 5e1; W(2,2) = 1e1; W(3,3) = 1e-1; W(4,4) = 1e-2; W(5,5)=W(4,4);
% W(6,6) = W(1,1); W(7,7) = W(2,2); W(8,8) = W(3,3); W(9,9) = W(4,4) ; W(10,10)=W(9,9);
% 
% W(1,1) = 1700; W(2,2) = 800; W(3,3) = 1e-1; W(4,4) = 1e-2; W(5,5)=W(4,4);
% W(6,6) = 900; W(7,7) = 600; W(8,8) = W(3,3); W(9,9) = 1e0 ; W(10,10)=W(9,9);

% W(1,1) = 1700; W(2,2) = 800; W(3,3) = 1e-1; W(4,4) = 1e-2; W(5,5)=W(4,4);
% W(6,6) = 1700; W(7,7) = 800; W(8,8) = W(3,3); W(9,9) = 1e0 ; W(10,10)=W(9,9);

W(1,1) = 1e-1; W(2,2) = 1e1; W(3,3) = 1e3; W(4,4) = 1e3; W(5,5)= 1e1;
W(6,6) = 1e-1; W(7,7) = 1e1; W(8,8) = W(3,3); W(9,9) = 1e3 ; W(10,10)=W(9,9);
W_e = eye(6)*1e-3;
W_e(1,1) = 9e4; W_e(2,2)= 1e4; W_e(4,4) = W_e(1,1); W_e(5,5)=W_e(2,2);
% W_e(1,1) = 850; W_e(2,2)= 850; W_e(4,4) = 850; W_e(5,5)=850;
yr = zeros(ny, 1); % output reference in lagrange term
% yr_e = zeros(ny_e, 1); % output reference in mayer term
yr_e = [1;1;pi/4;1;0;-pi/4];
% constraints
x0 = [0; 0; pi/4;0;1;-pi/4];
%Jbx = zeros(nbx, nx); for ii=1:nbx Jbx(ii,ii)=1.0; end
%lbx = -4*ones(nbx, 1);
%ubx =  4*ones(nbx, 1);
Jbu = zeros(nbu, nu); for ii=1:nbu Jbu(ii,ii)=1.0; end
lbu = [0;-pi/12;0;-pi/12];
ubu = [10;+pi/12;10;+pi/12];
%% CBF
radius = 0.05;
% CBF OLD x1 = model.sym_x(1:2);
% x2 = model.sym_x(4:5);
% 
% % v1 = model.sym_xdot(1:2);
% % v2 = model.sym_xdot(4:5);
% v1 = model.sym_u(1);v2=model.sym_u(3)
% 
% dist_squared = (x1(1) - x2(1))^2 + (x1(2) - x2(2))^2;
% 
% % the barrier function
% h = dist_squared;
% v_rel_longitudinal = v1 - v2;
% 
% % time derivative of the barrier function
% dh_dt = 2 * v_rel_longitudinal * (x1(1) - x2(1));
% 
% % negative alpha parameter
% alpha = 1e-1;  % smaller alfa - > stricter; larger alfa - > more flexible in appraoching the unsafe region  (Might be right Now that I was double checking!!)
% %check again this reasoning of alfa (might be wrong)
% C = -dh_dt + alpha*h;
% constr=struct();
% constr.expr_h= C;

x = model.sym_x(1:6); v1 = model.sym_u(1); v2 = model.sym_u(3); alpha = 1e-1;
dist_squared = ((x(1) - x(4))^2) + ((x(2) - x(5))^2);
h =dist_squared;
a1 = x(1) - x(4); a2 = v1*cos(x(3)) - v2*cos(x(6)); b1 = x(2) - x(5); b2 = v1*sin(x(3)) - v2*sin(x(6));
dh_dt = 2*((a1*a2) + (b1*b2));
C = -dh_dt + alpha * h; constr.expr_h = C;

%% acados ocp model
ocp_model = acados_ocp_model();ocp_model.set('name', model_name);ocp_model.set('T', T);
ocp_model.set('sym_x', model.sym_x);
if isfield(model, 'sym_u')
	ocp_model.set('sym_u', model.sym_u);
end
if isfield(model, 'sym_xdot')
	ocp_model.set('sym_xdot', model.sym_xdot);
end
ocp_model.set('cost_type', cost_type);ocp_model.set('cost_type_e', cost_type);
if (strcmp(cost_type, 'linear_ls'))
	ocp_model.set('cost_Vu', Vu);ocp_model.set('cost_Vx', Vx);ocp_model.set('cost_Vx_e', Vx_e);ocp_model.set('cost_W', W);ocp_model.set('cost_W_e', W_e);ocp_model.set('cost_y_ref', yr);ocp_model.set('cost_y_ref_e', yr_e);
elseif (strcmp(cost_type, 'ext_cost'))
	ocp_model.set('cost_expr_ext_cost', model.expr_ext_cost);ocp_model.set('cost_expr_ext_cost_e', model.expr_ext_cost_e);
end
if (strcmp(ocp_sim_method, 'erk'))
	ocp_model.set('dyn_type', 'explicit');ocp_model.set('dyn_expr_f', model.expr_f_expl);
else % irk
	ocp_model.set('dyn_type', 'implicit');ocp_model.set('dyn_expr_f', model.expr_f_impl);
end
% constraints
ocp_model.set('constr_x0', x0);
if (ng>0)
	ocp_model.set('constr_C', C);ocp_model.set('constr_D', D);ocp_model.set('constr_lg', lg);ocp_model.set('constr_ug', ug);ocp_model.set('constr_C_e', C_e);ocp_model.set('constr_lg_e', lg_e);ocp_model.set('constr_ug_e', ug_e);
elseif (nh>0)
	ocp_model.set('constr_expr_h', constr.expr_h);ocp_model.set('constr_lh', zeros(1,1));ocp_model.set('constr_uh', inf(1,1));   
%     Without relaxation I am having failed solution always.  
%     nsh = 1;
%     Jsh = 1;
%     ocp_model.set('constr_Jsh', Jsh);
%     ocp_model.set('cost_zl', 1e-2 * ones(nsh,1));
%     ocp_model.set('cost_zu', 0 * ones(nsh,1));
%     ocp_model.set('cost_Zl', 1e-4 * ones(nsh,nsh));
%     ocp_model.set('cost_Zu', 0 * ones(nsh,nsh));
else
	ocp_model.set('constr_Jbu', Jbu);ocp_model.set('constr_lbu', lbu);ocp_model.set('constr_ubu', ubu);
end
	ocp_model.set('constr_Jbu', Jbu);ocp_model.set('constr_lbu', lbu);ocp_model.set('constr_ubu', ubu);
ocp_model.model_struct
%% acados ocp opts
ocp_opts = acados_ocp_opts();ocp_opts.set('compile_interface', compile_interface);ocp_opts.set('codgen_model', codgen_model);ocp_opts.set('param_scheme_N', ocp_N);ocp_opts.set('nlp_solver', nlp_solver);ocp_opts.set('nlp_solver_exact_hessian', nlp_solver_exact_hessian);ocp_opts.set('regularize_method', regularize_method);
if (strcmp(nlp_solver, 'sqp'))
	ocp_opts.set('nlp_solver_max_iter', nlp_solver_max_iter);
end
ocp_opts.set('qp_solver', qp_solver);
if (strcmp(qp_solver, 'partial_condensing_hpipm'))
	ocp_opts.set('qp_solver_cond_N', qp_solver_cond_N);ocp_opts.set('qp_solver_cond_ric_alg', qp_solver_cond_ric_alg);ocp_opts.set('qp_solver_ric_alg', qp_solver_ric_alg);ocp_opts.set('qp_solver_warm_start', qp_solver_warm_start);
end
ocp_opts.set('qp_solver_iter_max', qp_solver_iter_max);ocp_opts.set('sim_method', ocp_sim_method);ocp_opts.set('sim_method_num_stages', ocp_sim_method_num_stages);ocp_opts.set('sim_method_num_steps', ocp_sim_method_num_steps);ocp_opts.opts_struct
%% acados ocp
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
	sim_model.set('dyn_type', 'explicit');sim_model.set('dyn_expr_f', model.expr_f_expl);
else % irk
	sim_model.set('dyn_type', 'implicit');sim_model.set('dyn_expr_f', model.expr_f_impl);
end
%% acados sim opts
sim_opts = acados_sim_opts();sim_opts.set('compile_interface', compile_interface);sim_opts.set('codgen_model', codgen_model);sim_opts.set('num_stages', sim_num_stages);sim_opts.set('num_steps', sim_num_steps);sim_opts.set('method', sim_method);sim_opts.set('sens_forw', sim_sens_forw);
%% acados sim
sim = acados_sim(sim_model,sim_opts);
N_sim = 50;
x_sim = zeros(nx, N_sim+1);x_sim(:,1) = x0;u_sim = zeros(nu, N_sim);
x_traj_init1=[linspace(0,1,ocp_N+1);linspace(0,1,ocp_N+1);(pi/4)*ones(1,ocp_N+1)];
x_traj_init2=[linspace(0,1,ocp_N+1);linspace(1,0,ocp_N+1);-(pi/4)*ones(1,ocp_N+1)];
x_traj_init=[x_traj_init1;x_traj_init2];
u_traj_init = zeros(nu, ocp_N);
    tic;
for ii= 1 : N_sim
    %%% Correcting the referencing for the shooting nodes
    for jj = 0 : ocp_N - 1
    yref = [u_traj_init(1,jj+1);u_traj_init(2,jj+1);x_traj_init(1,jj + 1);x_traj_init(2,jj+1);x_traj_init(3,jj+1);u_traj_init(3,jj+1);u_traj_init(4,jj+1);x_traj_init(4,jj + 1);x_traj_init(5,jj+1);x_traj_init(6,jj+1)];
    ocp.set('cost_y_ref', yref, jj);
    end
    ocp.set('cost_y_ref_e', yr_e, ocp_N);
	ocp.set('constr_x0', x_sim(:,ii));ocp.set('init_x', x_traj_init);ocp.set('init_u', u_traj_init);ocp.solve();
	if 1
		status = ocp.get('status');sqp_iter = ocp.get('sqp_iter');time_tot = ocp.get('time_tot');time_lin = ocp.get('time_lin');time_qp_sol = ocp.get('time_qp_sol');
		fprintf('\nstatus = %d, sqp_iter = %d, time_int = %f [ms] (time_lin = %f [ms], time_qp_sol = %f [ms])\n',...
            status, sqp_iter, time_tot*1e3, time_lin*1e3, time_qp_sol*1e3);
        if status~=0
            disp('acados ocp solver failed');
        end
	end
x_traj = ocp.get('x');
u_traj = ocp.get('u');
x_traj_init = [x_traj(:,2:end), x_traj(:,end)];
u_traj_init = [u_traj(:,2:end), u_traj(:,end)];
u_sim(:,ii) = ocp.get('u', 0);
sim.set('x',x_sim(:,ii));
sim.set('u', u_sim(:,ii));
sim.solve();
x_sim(:,ii+1) = sim.get('xn');
    %%%%%%% Testing to monitor the value of the constraint
    x_curr = x_sim(:, ii);
    u_curr = u_sim(:, ii);
    dist_test = (x_curr(1) - x_curr(4))^2 + (x_curr(2) - x_curr(5))^2;
    a1_test = x_curr(1) - x_curr(4); a2_test = u_curr(1)*cos(x_curr(3)) - u_curr(3)*cos(x_curr(6)); b1_test = x_curr(2) - x_curr(5); b2_test = u_curr(1)*sin(x_curr(3)) - u_curr(3)*sin(x_curr(6));
    dh_dt_test = 2*((a1_test*a2_test) + (b1_test*b2_test));
    h_test = dist_test;
    constr_value = -dh_dt_test + alpha * h_test;
    disp(['Iteration ', num2str(ii), ': Constraint value = ', num2str(constr_value)]);
    %%%%%%%%
end
avg_time_solve = toc/N_sim
%% Plotting and monitoring
%Plot of trajectory
figure(1)
plot(x_sim(1,:),x_sim(2,:),'r*-'); hold on; plot(x_sim(4,:),x_sim(5,:),'k*-'); legend('car 1','car 2');
grid on
figure(2)
plot(1:50,u_sim(1,:),1:50,u_sim(3,:));
legend('Vel car1','Vel car2');
%% New simulation Plot / Animation
if 0
% Number of points to approximate the circle
% Initialize the plot
num_points_circle = 100;
figure(1);
h_car1 = plot(x_sim(1,1), x_sim(2,1), 'r*-'); 
hold on; 
h_car2 = plot(x_sim(4,1), x_sim(5,1), 'k*-'); 
legend('car 1', 'car 2');
grid on
% Plot initial and goal positions for each car
plot(x0(1), x0(2), 'go', 'MarkerSize', 10); % Initial position of car 1
plot(x0(4), x0(5), 'go', 'MarkerSize', 10); % Initial position of car 2
plot(yr_e(1), yr_e(2), 'ro', 'MarkerSize', 10); % Goal position of car 1
plot(yr_e(4), yr_e(5), 'ro', 'MarkerSize', 10); % Goal position of car 2
% Plot circles around each car's initial position
% Circle around car 1
center_car1 = [x_sim(1,1); x_sim(2,1)];
theta = linspace(0, 2*pi, num_points_circle);
x_circle_car1 = center_car1(1) + radius * cos(theta);
y_circle_car1 = center_car1(2) + radius * sin(theta);
h_circle_car1 = plot(x_circle_car1, y_circle_car1, 'r--');
% Circle around car 2
center_car2 = [x_sim(4,1); x_sim(5,1)];
x_circle_car2 = center_car2(1) + radius * cos(theta);
y_circle_car2 = center_car2(2) + radius * sin(theta);
h_circle_car2 = plot(x_circle_car2, y_circle_car2, 'k--');
% Set axis limits
xlim([-1 1.5]);ylim([-0.25 1.5])
pause (2)
% Loop to animate the plot
for i = 2:N_sim
    % Update car positions
    set(h_car1, 'XData', x_sim(1,i), 'YData', x_sim(2,i));
    set(h_car2, 'XData', x_sim(4,i), 'YData', x_sim(5,i));
    
    % Update circles around cars' positions
    center_car1 = [x_sim(1,i); x_sim(2,i)];
    x_circle_car1 = center_car1(1) + radius * cos(theta);
    y_circle_car1 = center_car1(2) + radius * sin(theta);
    set(h_circle_car1, 'XData', x_circle_car1, 'YData', y_circle_car1);
    
    center_car2 = [x_sim(4,i); x_sim(5,i)];
    x_circle_car2 = center_car2(1) + radius * cos(theta);
    y_circle_car2 = center_car2(2) + radius * sin(theta);
    set(h_circle_car2, 'XData', x_circle_car2, 'YData', y_circle_car2);
    
    % Pause for animation
    if i==2 
    pause(2); % Adjust as needed
    else
        pause (0.5)
    end
end
%% Distance evaluation
% Extract positions of each car from x_sim
xa = x_sim(1,:);
ya = x_sim(2,:);
xb = x_sim(4,:);
yb = x_sim(5,:);
% Calculate distance between the two cars
distance = sqrt((xb - xa).^2 + (yb - ya).^2);
mindistance = min(distance)
end



