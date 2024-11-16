%% Scenario 1 car with an obstacle in the middle
% GOOD Try to fix the extend of the reference trajectory though!
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
ocp_N = 100;
nlp_solver = 'sqp_rti'; % sqp, sqp_rti
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
model = newdyn()
h = 0.02;
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
    nbu = nu;
	nh = nu;
	nh_e = 0;
end
% cost
Vu = zeros(ny, nu); for ii=1:nu  Vu(ii,ii)=1.0; end % input-to-output matrix in lagrange term
Vx = zeros(ny, nx); for ii=1:nx Vx(nu+ii,ii)=1.0; end % state-to-output matrix in lagrange term
Vx_e = zeros(ny_e, nx); for ii=1:nx Vx_e(ii,ii)=1.0; end % state-to-output matrix in mayer term
W = eye(ny); % weight matrix in lagrange term
% W(1,1) = 1e1; W(2,2) = 1e-1; W(3,3) = 1e3; W(4,4) = 1e-1; W(5,5)= 1e-1; W(6,6)= 1e-1; W(7,7)= 1e-1;
W(1,1) = 1e3; W(2,2) = 1e-1; W(3,3) = 1e5 ;W(4,4) = 1e5; W(5,5)= 1e-2; W(6,6)= 1e2; W(7,7)= 1e-2;
W_e = eye(5);
W_e(1,1) = 1e5; W_e(2,2)= 1e5; W_e(3,3)=1e-2; W_e(4,4)=1e2; W_e(5,5)=1e-2;
yr = zeros(ny, 1);
yr_e = [110;10*T;0;10;0];
% constraints
x0 = [110; -30; pi/2;5;0];
Jbx = zeros(1, nx); for ii=1:nbx Jbx(ii,ii)=1.0; end
lbx = 0*ones(nbx, 1);
ubx =  120*ones(nbx, 1);
Jbu = zeros(nbu, nu); for ii=1:nbu Jbu(ii,ii)=1.0; end
lbu = [-11.5;-1];
ubu = [+11.5;+1];
%% Bound on State X and Y
% % Define the index for the velocity state
idx_x=1; idx_y=2;

% Define Jbx to constrain the velocity (4th state variable)
Jbx = zeros(1, nx); 
Jbx(idx_x) = 1; % only constrain the velocity state
Jbx(idx_y) = 1; % only constrain the velocity state

% Set lower and upper bounds for velocity
lbx = 0; % minimum velocity
ubx = inf; % no upper bound constraint on velocity
%% CBF Updated for moving obtacle (only in x) 
% radius_car = 0.25; 
% radius_obs = 1.0;
% safety_distance = 0.50;
% obstacle_center = [50; 0];  % Position of the obstacle center (Assumed to be a point mass for now)
v_obs=-1;
x = model.sym_x(1:3);
v = model.sym_x(4);
x_obs = model.sym_p(1);
y_obs = -5;
global alpha 
global dsafe
alpha = 5.0; dsafe= 5.0;
dist_squared = ((x(1) - x_obs)^2) + ((x(2) - y_obs)^2) - dsafe;
h =dist_squared;
a1 = x(1) - x_obs(1); a2 = v*cos(x(3)) - v_obs ; b1 = x(2) - y_obs; b2 = v*sin(x(3));
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
ocp_model.set('sym_p', model.sym_p);
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
ocp_model.set('constr_lh', 0); % Ensure dh_dt + alpha * h is always positive
ocp_model.set('constr_uh', 1e8);
% nsh = 1;
% Jsh = 1;
% 
% ocp_model.set('constr_Jsh', Jsh);
% % Set cost on slack
% % L1 slack (linear term)
% ocp_model.set('cost_zl', 1e-2 * ones(nsh,1));
% ocp_model.set('cost_zu', 0* ones(nsh,1));
% % L2 slack (squared term)
% ocp_model.set('cost_Zl', 1e-4 * ones(nsh,nsh));
% ocp_model.set('cost_Zu', 0 * ones(nsh,nsh));
end
end
ocp_model.set('constr_Jbu', Jbu);
ocp_model.set('constr_lbu', lbu);
ocp_model.set('constr_ubu', ubu);
% ocp_model.set('constr_Jbx', Jbx);
% ocp_model.set('constr_lbx', lbx);
% ocp_model.set('constr_ubx', ubx);
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
%% acados sim
D=Way();
sim = acados_sim(sim_model,sim_opts);
% N_sim = ocp_N;
N_sim = 800;
% N_sim = 1;
x_sim = zeros(nx, N_sim+1);
x_sim(:,1) = x0; % initial state
u_sim = zeros(nu, N_sim);

yr_e_new = [110;x0(2) + x0(4)*T;pi/2;5;0];
% x_traj_init=[linspace(0,x0(4)*T,ocp_N+1);linspace(0,0,ocp_N+1);zeros(1,ocp_N+1);10*ones(1,ocp_N+1);zeros(1,ocp_N+1)];
u_traj_init = zeros(nu, ocp_N);
C_val_monitor=[];
COST=[]; dt=0.02;
p_ref = zeros(1,ocp_N); p_ref(1)=140;
v_obs= 6.5;
for yy= 2:ocp_N
    p_ref(yy) = p_ref(yy-1) - v_obs*dt;
end
Obstaclecenter =[p_ref(1)]
%%
    tic;
for ii= 1 : N_sim
%%%
    current_state = x_sim(:, ii); % Assuming the current state is [x; y; ...]
    closest_idx = findClosestPoint(current_state(1:2), D);
    if closest_idx + 100 <= size(D, 2)
        reference_points = D(:, closest_idx:closest_idx+100);
    else
%         % If there are not enough points, use the last available segment
%         reference_points = D(:, end-100:end);
  % If there are not enough points, extend the trajectory
        remaining_points = size(D, 2) - closest_idx + 1;
        reference_points = D(:, closest_idx:end);
        
        % Calculate the number of additional points needed
        additional_points = 101 - remaining_points;
        
        % Extend the trajectory in the same direction
        last_x = D(1, end);
        last_y = D(2, end);
        
        new_x = last_x - (1:additional_points) * 0.1; % Assuming 0.1m per step
        new_y = last_y * ones(1, additional_points);
        
        extended_points = [new_x; new_y];
        
        % Combine the existing and extended points
        reference_points = [reference_points, extended_points];
    end
    x_traj_init = [reference_points; zeros(1, size(reference_points, 2)); x0(4)*ones(1, size(reference_points, 2));zeros(1, size(reference_points, 2))];

    for jj = 0 : ocp_N - 1
        kk = min(jj + 1, ocp_N-1);
        yref = [u_traj_init(1,kk);u_traj_init(2,kk);x_traj_init(1,jj + 1);x_traj_init(2,jj+1);x_traj_init(3,jj+1);x_traj_init(4,jj+1);x_traj_init(5,jj+1)];
        ocp.set('cost_y_ref', yref, jj);
    end
    ocp.set('cost_y_ref_e', yr_e_new, ocp_N);
	ocp.set('constr_x0', x_sim(:,ii));
    for jj=0:ocp_N-1
        ocp.set('p', p_ref(jj+1), jj);
    end
% 	ocp.set('init_x', x_traj_init);
% 	ocp.set('init_u', u_traj_init);
	ocp.solve();
	if 1
        disp('')
		status = ocp.get('status');
		sqp_iter = ocp.get('sqp_iter');
		time_tot = ocp.get('time_tot');
		time_lin = ocp.get('time_lin');
		time_qp_sol = ocp.get('time_qp_sol');
        ii
        cost = ocp.get_cost();
        COST = [COST cost];

		fprintf('\nstatus = %d, sqp_iter = %d, time_int = %f [ms] (time_lin = %f [ms], time_qp_sol = %f [ms])\n',...
            status, sqp_iter, time_tot*1e3, time_lin*1e3, time_qp_sol*1e3);
        if status~=0
            disp('acados ocp solver failed');
        end
	end
x_traj = ocp.get('x');
u_traj = ocp.get('u');
% x_traj_init = [x_traj(:,2:end), x_traj(:,end)]; % Also I think it needs to be shifted properly
u_traj_init = [u_traj(:,2:end), u_traj(:,end)];
% u_traj_init (1,:) = 0;
yr_e_new = x_traj_init(:,end); %this is not enough I choose shift it with the simulated state at the next instant I think
u_sim(:,ii) = ocp.get('u', 0);
sim.set('x',x_sim(:,ii));
sim.set('u', u_sim(:,ii));
sim.solve();
x_sim(:,ii+1) = sim.get('xn');
p_ref = p_ref - v_obs*dt;
Obstaclecenter=[Obstaclecenter p_ref(1)];
end



avg_time_solve = toc/N_sim
%% Plotting
if 1
    figure(100)
plot(x_sim(1,:),x_sim(2,:),'r*-');
axis equal
% xlim([min(x_sim(1,:)) - 5, max(x_sim(1,:)) + 5]);
% ylim([min(x_sim(2,:)) - 5, max(x_sim(2,:)) + 5]);
xlabel('x [m]');ylabel('y [m]'); title('Trajectory'); grid on; hold on;
% hold on; plot(obstacle_center(1),obstacle_center(2),'ko',LineWidth=5); legend('car 1','Obstacle'); title('Trajectory XY'); xlabel('x'); ylabel('y');
% Define the center and radius
% center_x = 50;center_y = 0;
radius = dsafe;
hold on
plot(Obstaclecenter,-5*ones(length(Obstaclecenter)),'b')
road_width = 20;
road_length = 200;
merging_lane_width = 10;
merging_lane_position = 100;

% Plot road
hold on;

% Main road
rectangle('Position', [0, -road_width/2, road_length, road_width], 'FaceColor', [0, 1, 1, 0.5]);

% Merging lane
% rectangle('Position', [merging_lane_position - merging_lane_width/2, -road_width, merging_lane_width, road_width], 'FaceColor', 'r');
rectangle('Position', [100 -50 20 40], 'FaceColor', [0, 1, 0, 0.5]);

% Set axis properties
axis equal;
xlabel('x [m]');
ylabel('y [m]');
title('Road');
grid on;

hold off;
% Generate points on the circle
% theta = linspace(0, 2*pi, 101); x_circle = Obstaclecenter + radius * cos(theta);y_circle = 0 + radius * sin(theta);
% plot(x_circle, y_circle, 'b-', 'LineWidth', 2); % 'b-' means blue solid line

figure (200)
subplot(3,1,1)
plot(1:N_sim,u_sim(1,:)); title('Acceleration');
subplot(3,1,2)
plot(1:N_sim,u_sim(2,:)); title ('Steering Rate');
subplot(3,1,3)
plot(1:N_sim+1,x_sim(4,:)); title ('Velocity');

figure (300)
plot(1:N_sim,COST); title('Cost')
grid on

end
return
%% Animation 
%% Animation 
if 1
    % Define figure for the animation
    figure(700);
    axis equal;
    xlabel('x [m]');
    ylabel('y [m]');
    title('Trajectory');
    grid on;
    rectangle('Position', [0, -road_width/2, road_length, road_width], 'FaceColor', [0, 1, 1, 0.5]);

% Merging lane
% rectangle('Position', [merging_lane_position - merging_lane_width/2, -road_width, merging_lane_width, road_width], 'FaceColor', 'r');
rectangle('Position', [100 -50 20 40], 'FaceColor', [0, 1, 0, 0.5]);
    hold on;

    % Initialize the plot for the car trajectory
    car_trajectory = plot(x_sim(1,1), x_sim(2,1), 'r*-');
    
    % Initialize the plot for the obstacle circle
    obstacle_circle = plot(Obstaclecenter(1), -5, 'bo');

    % Set up the axis limits
    xlim([min(x_sim(1,:)) - 30, max(x_sim(1,:)) + 30]);
    ylim([min(x_sim(2,:)) - 30, max(x_sim(2,:)) + 30]);

    % Animation loop
    for k = 2:length(x_sim)
        % Update the car's position
        set(car_trajectory, 'XData', x_sim(1,1:k), 'YData', x_sim(2,1:k));
        
        % Update the obstacle position
        set(obstacle_circle, 'XData', Obstaclecenter(k), 'YData', -5);
        
        % Pause to create animation effect
        pause(0.025); % Adjust the pause time to control the speed of the animation
    end

    hold off;
end
