%% Scenario 1 car with an obstacle in the middle
%Testing with a new reference trajectory strategy implementation
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
% nlp_solver = 'sqp_rti';
%nlp_solver_exact_hessian = 'false';
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
model = DYNtwocar()
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
% 	nbu = 0;
    nbu = nu;
	nh = nu;
	nh_e = 0;
end
% cost
%% Normalizing the weights
Nx=200^2;Ny=Nx;Ntheta=2*pi^2;Nv=30^2;Ns=0.5^2;Na=23^2;Nss=2^2;
%% Note that first 4 elements are for u probably (double check later) 
Vu = zeros(ny, nu); for ii=1:nu  Vu(ii,ii)=1.0; end % input-to-output matrix in lagrange term
Vx = zeros(ny, nx); for ii=1:nx Vx(nu+ii,ii)=1.0; end % state-to-output matrix in lagrange term
Vx_e = zeros(ny_e, nx); for ii=1:nx Vx_e(ii,ii)=1.0; end % state-to-output matrix in mayer term
W = eye(ny); % weight matrix in lagrange term
% W(1,1) = 1e10; W(2,2) = 1e-10; W(3,3) = 1e3; W(4,4) = 1e2; W(5,5)= 1e-1; W(6,6)= 1e-2; W(7,7)= 1e-7;


W(1,1) = 1e1; W(2,2) = 1e-10; W(3,3) = 1e10; W(4,4) = 1e-10; 
W(5,5)= 1e3; W(6,6)= 1e2; W(7,7)= 1e-1;W(8,8) = 1e2; W(9,9) = 1e-7;
W(10,10) = 1e3; W(11,11) = 1e2; W(12,12)= 1e-1; W(13,13)= 1e-2; W(14,14)= 1e-7;
W_e = eye(10);
W_e(1,1) = 1e1; W_e(2,2)= 1e1; W_e(3,3)=1e-5; W_e(4,4)=1e2; W_e(5,5)=1e-10;
W_e(6,6) = 1e1; W_e(7,7)= 1e1; W_e(8,8)=1e-5; W_e(9,9)=1e-1; W_e(10,10)=1e-10;


% W(1,1) = 1e1/Na; W(2,2) = 1e-10/Nss; W(3,3) = 1e1/Na; W(4,4) = 1e-10/Nss; 
% W(5,5)= 1e5/Nx; W(6,6)= 1e5/Ny; W(7,7)= 1e-5/Ntheta;W(8,8) = 1e-5/Nv; W(9,9) = 1e-7/Ns;
% W(10,10) = 1e4/Nx; W(11,11) = 1e4/Ny; W(12,12)= 1e-5/Ntheta; W(13,13)= 1e-5/Nv; W(14,14)= 1e-7/Ns;
% W_e = eye(10);
% W_e(1,1) = 1e5/Nx; W_e(2,2)= 1e5/Ny; W_e(3,3)=1e-5/Ntheta; W_e(4,4)=1e-3/Nv; W_e(5,5)=1e-10/Ns;
% W_e(6,6) = 1e5/Nx; W_e(7,7)= 1e5/Ny; W_e(8,8)=1e-5/Ntheta; W_e(9,9)=1e-3/Nv; W_e(10,10)=1e-10/Ns;
yr = zeros(ny, 1);
yr_e = [10*T;0;0;10;0;80;0;0;10;0];
% constraints
x0 = [0; 0; 0;10;0;100;0;0;-10;0];
% Jbx = zeros(1, nx); for ii=1:nbx Jbx(ii,ii)=1.0; end
% lbx = 0*ones(nbx, 1);
% ubx =  120*ones(nbx, 1);
Jbu = zeros(nbu, nu); for ii=1:nbu Jbu(ii,ii)=1.0; end
lbu = [-11.5;-1;-11.5;-1];
ubu = [+11.5;+1;+11.5;+1];
%% Bound on State (Velocity)
% % Define the index for the velocity state
% idx_velocity = 4;
% 
% % Define Jbx to constrain the velocity (4th state variable)
% Jbx = zeros(1, nx); 
% Jbx(idx_velocity) = 1; % only constrain the velocity state
% 
% % Set lower and upper bounds for velocity
% lbx = 0; % minimum velocity
% ubx = inf; % no upper bound constraint on velocity
%% New CBF 
% radius_car = 0.25; 
% radius_obs = 1.0;
% safety_distance = 0.50;
% obstacle_center = [50; 0];  % Position of the obstacle center (Assumed to be a point mass for now)

x1 = model.sym_x(1:3);
v1 = model.sym_x(4);
x2 = model.sym_x(6:8);
v2 = model.sym_x(9);
global alpha 
global dsafe
alpha = 5.0; dsafe= 5.0;
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
N_sim = 500;
% N_sim = 1;
x_sim = zeros(nx, N_sim+1);
x_sim(:,1) = x0; % initial state
u_sim = zeros(nu, N_sim);

yr_e_new = [x0(4)*T;0;0;10;0;100+x0(9)*T;0;0;-10;0];
% x_traj_init=[linspace(0,x0(4)*T,ocp_N+1);linspace(0,0,ocp_N+1);zeros(1,ocp_N+1);10*ones(1,ocp_N+1);zeros(1,ocp_N+1)];
u_traj_init = zeros(nu, ocp_N);
C_val_monitor=[];
COST=[];

% obstacle_position = [50; 0]; % Example obstacle at (50, 0)
% x_val=[x0(1);x0(2);x0(3)]; v_val=x0(4);
%     goal_x = 100;
% goal_y = 0;
    tic;
for ii= 1 : N_sim
%         C_val = evaluate_CBF(x_val, v_val);
%         C_val_monitor = [C_val_monitor C_val];
%             dist_to_goal = sqrt((goal_x - x_sim(1, ii))^2 + (goal_y - x_sim(2, ii))^2);
%             step_size = min(dist_to_goal, T * x_sim(4,ii)); 
             step_size1=T*x_sim(4,ii);
             step_size2=T*x_sim(9,ii);
             x_next1 = linspace(x_sim(1, ii), x_sim(1, ii) + step_size1, ocp_N + 1);
             x_next2 = linspace(x_sim(6, ii), x_sim(6, ii) + step_size2, ocp_N + 1);
             y_next = linspace(0, 0, ocp_N + 1);
           x_traj_init = [x_next1; y_next; zeros(1, ocp_N + 1); 10 * ones(1, ocp_N + 1); zeros(1, ocp_N + 1);x_next2; y_next; zeros(1, ocp_N + 1); -10 * ones(1, ocp_N + 1); zeros(1, ocp_N + 1)];

    for jj = 0 : ocp_N - 1
        kk = min(jj + 1, ocp_N-1);
        yref = [u_traj_init(1,kk);u_traj_init(2,kk);u_traj_init(3,kk);u_traj_init(4,kk);x_traj_init(1,jj + 1);x_traj_init(2,jj+1);x_traj_init(3,jj+1);x_traj_init(4,jj+1);x_traj_init(5,jj+1);x_traj_init(6,jj + 1);x_traj_init(7,jj+1);x_traj_init(8,jj+1);x_traj_init(9,jj+1);x_traj_init(10,jj+1)];
        ocp.set('cost_y_ref', yref, jj);
    end
    ocp.set('cost_y_ref_e', yr_e_new, ocp_N);
	ocp.set('constr_x0', x_sim(:,ii));
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
end
avg_time_solve = toc/N_sim
%% Plotting
if 1
    figure(100)
%%%%
    % Define road properties
    road_length = 250;  % Length of the road
    road_width = 40;    % Width of the road
    
    % Plot the road as a rectangle (centered at y=0)
    rectangle('Position', [-10, -road_width/2, road_length, road_width], 'FaceColor', [0.7 0.7 0.7], 'EdgeColor', 'none');
    hold on;
% Plot lane markings at the center, top, and bottom of the road
plot([-10 road_length], [-road_width/4, -road_width/4], 'w--', 'LineWidth', 2);  % Bottom lane marking
plot([-10 road_length], [road_width/4, road_width/4], 'w--', 'LineWidth', 2);    % Top lane marking

    hold on
%%%
% Plot the starting positions of agents
start_pos_agent1 = [x_sim(1,1), x_sim(2,1)];  % Initial position of Agent 1
start_pos_agent2 = [x_sim(6,1), x_sim(7,1)];  % Initial position of Agent 2
plot(start_pos_agent1(1),start_pos_agent1(2),start_pos_agent2(1),start_pos_agent2(2));

% Add arrows pointing to the initial positions
quiver(start_pos_agent1(1), start_pos_agent1(2), 0, 5, 'ko', 'LineWidth', 1.5, 'MaxHeadSize', 2); % Agent 1
quiver(start_pos_agent2(1), start_pos_agent2(2), 0, 5, 'ko', 'LineWidth', 1.5, 'MaxHeadSize', 2); % Agent 2

% Add text labels for the initial positions
text(start_pos_agent1(1), start_pos_agent1(2) + 5, 'Agent 1 Start', 'Color', 'k', 'FontSize', 10, 'HorizontalAlignment', 'center');
text(start_pos_agent2(1), start_pos_agent2(2) + 5, 'Agent 2 Start', 'Color', 'k', 'FontSize', 10, 'HorizontalAlignment', 'center');


plot(x_sim(1,1),x_sim(2,1),'bo');
hold on

h1=plot(x_sim(1,:),x_sim(2,:),'r*');
hold on
h2=plot(x_sim(6,:),x_sim(7,:),'b','LineWidth',3);
legend('Agent 1 (Red)', 'Agent 2 (Blue)', 'Location', 'Best');  % Specify the legend labels directly
axis equal
% xlim([min(x_sim(1,:)) - 5, max(x_sim(1,:)) + 5]);
% ylim([min(x_sim(2,:)) - 5, max(x_sim(2,:)) + 5]);
xlabel('x [m]','Interpreter','latex');ylabel('y [m]','Interpreter','latex'); title('Trajectory','Interpreter','latex'); grid on; hold on; grid minor;
legend([h1, h2], {'Agent 1', 'Agent 2'});


%%

figure (200)
subplot(3,2,1)
plot(1:N_sim,u_sim(1,:)); title('Acceleration1'); grid on; grid minor;
subplot(3,2,2)
plot(1:N_sim,u_sim(2,:)); title ('Steering Rate1');grid on; grid minor;
subplot(3,2,3)
plot(1:N_sim+1,x_sim(4,:)); title ('Velocity1');grid on; grid minor;
subplot(3,2,4)
plot(1:N_sim,u_sim(3,:)); title('Acceleration2');grid on; grid minor;
subplot(3,2,5)
plot(1:N_sim,u_sim(4,:)); title ('Steering Rate2');grid on; grid minor;
subplot(3,2,6)
plot(1:N_sim+1,x_sim(9,:)); title ('Velocity2');grid on; grid minor;
saveas(gcf, 'LQR_Exo_botharms.pdf','pdf');

figure (300)
plot(1:N_sim,COST); title('Cost','Interpreter','latex')
grid on
grid minor

end
return
%% Animation 
if 1
% Define figure for the animation
figure(700);
axis equal;
xlabel('x [m]');
ylabel('y [m]');
title('Trajectory');
grid on;
hold on;

% % Plot the obstacle
% center_x = 50;
% center_y = 0;
% radius = dsafe;
% theta = linspace(0, 2*pi, 100);
% x_circle = center_x + radius * cos(theta);
% y_circle = center_y + radius * sin(theta);
% plot(x_circle, y_circle, 'b-', 'LineWidth', 2); % Plot the obstacle

% Initialize the plot for the car trajectory
car_trajectory1 = plot(x_sim(1,1), x_sim(2,1), 'r*-');
car_trajectory2 = plot(x_sim(6,1), x_sim(7,1), 'bo');


% Set up the axis limits
% xlim([min(x_sim(1,:)) - 5, max(x_sim(1,:)) + 5]);
% ylim([min(x_sim(2,:)) - 5, max(x_sim(2,:)) + 5]);

% Animation loop
for k = 2:length(x_sim)
    % Update the car's position
    set(car_trajectory1, 'XData', x_sim(1,1:k), 'YData', x_sim(2,1:k));
    set(car_trajectory2, 'XData', x_sim(6,1:k), 'YData', x_sim(7,1:k));

    
    % Pause to create animation effect
    pause(0.01); % Adjust the pause time to control the speed of the animation
    
    % Optionally, add a small circle or marker to show the current car position
    plot(x_sim(1,k), x_sim(2,k), 'r*'); 
    plot(x_sim(6,k), x_sim(7,k), 'bo'); 
end

% hold off;
end