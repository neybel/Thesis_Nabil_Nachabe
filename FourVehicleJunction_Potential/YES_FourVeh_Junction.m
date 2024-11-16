%Four car junction (straight lines)
%%
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
%% create model entries
model = DYNFourcarExternalCoupling();
h = 0.02;T = ocp_N*h;
% dims
nx = model.nx;nu = model.nu;ny = nu+nx; ny_e = nx; ng = 0;ng_e = 0; nbx = 0;
linear_constraints = 0; 
if linear_constraints
	nbu = nu; nh = 0;nh_e = 0;
else
    nbu = nu;nh = nu;nh_e = 0;
end
% constraints
x0 = [115; -40; pi/2;5;0;95;40;1.5*pi;5;0;165;10;pi;5;0;45;-10;0;5;0];
% Jbx = zeros(1, nx); for ii=1:nbx Jbx(ii,ii)=1.0; end% lbx = 0*ones(nbx, 1);% ubx =  120*ones(nbx, 1);
Jbu = zeros(nbu, nu); for ii=1:nbu Jbu(ii,ii)=1.0; end
lbu = [-11.5;-1;-11.5;-1;-11.5;-1;-11.5;-1];
ubu = [+11.5;+1;+11.5;+1;+11.5;+1;+11.5;+1];
%% New CBF (6 CBFS!!)
x1 = model.sym_x(1:3);v1 = model.sym_x(4);
x2 = model.sym_x(6:8);v2 = model.sym_x(9);
x3 = model.sym_x(11:13);v3 = model.sym_x(14);
x4 = model.sym_x(16:18);v4 = model.sym_x(19);

global alpha 
global dsafe
alpha = 2; dsafe= 10;
% alpha = 11; dsafe= 20.0;
%% C12 case 1
dist_squared1 = ((x1(1) - x2(1))^2) + ((x1(2) - x2(2))^2) - dsafe;
h1 =dist_squared1;
a1 = x1(1) - x2(1); a2 = v1*cos(x1(3))-v2*cos(x2(3)); b1 = x1(2) - x2(2); b2 = v1*sin(x1(3))-v2*sin(x2(3));
dh_dt1 = 2*((a1*a2) + (b1*b2));
C1 = + dh_dt1 + alpha * h1; % choice between + or -
%% C13 case 2
dist_squared2 = ((x1(1) - x3(1))^2) + ((x1(2) - x3(2))^2) - dsafe;
h2 =dist_squared2;
a1_2 = x1(1) - x3(1); a2_2 = v1*cos(x1(3))-v3*cos(x3(3)); b1_2 = x1(2) - x3(2); b2_2 = v1*sin(x1(3))-v3*sin(x3(3));
dh_dt2 = 2*((a1_2*a2_2) + (b1_2*b2_2));
C2 = + dh_dt2 + alpha * h2;
%% C14 case 3
dist_squared3 = ((x1(1) - x4(1))^2) + ((x1(2) - x4(2))^2) - dsafe;
h3 =dist_squared3;
a1_3 = x1(1) - x4(1); a2_3 = v1*cos(x1(3))-v4*cos(x4(3)); b1_3 = x1(2) - x4(2); b2_3 = v1*sin(x1(3))-v4*sin(x4(3));
dh_dt3 = 2*((a1_3*a2_3) + (b1_3*b2_3));
C3 = + dh_dt3 + alpha * h3;
%% C23 case 4
dist_squared4 = ((x2(1) - x3(1))^2) + ((x2(2) - x3(2))^2) - dsafe;
h4 =dist_squared4;
a1_4 = x2(1) - x3(1); a2_4 = v2*cos(x2(3))-v3*cos(x3(3)); b1_4 = x2(2) - x3(2); b2_4 = v2*sin(x2(3))-v3*sin(x3(3));
dh_dt4 = 2*((a1_4*a2_4) + (b1_4*b2_4));
C4 = + dh_dt4 + alpha * h4;
%% C24 case 5
dist_squared5 = ((x2(1) - x4(1))^2) + ((x2(2) - x4(2))^2) - dsafe;
h5 =dist_squared5;
a1_5 = x2(1) - x4(1); a2_5 = v2*cos(x2(3))-v4*cos(x4(3)); b1_5 = x2(2) - x4(2); b2_5 = v2*sin(x2(3))-v4*sin(x4(3));
dh_dt5 = 2*((a1_5*a2_5) + (b1_5*b2_5));
C5 = + dh_dt5 + alpha * h5;
%% C34 case 6
dist_squared6 = ((x3(1) - x4(1))^2) + ((x3(2) - x4(2))^2) - dsafe;
h6 =dist_squared6;
a1_6 = x3(1) - x4(1); a2_6 = v3*cos(x3(3))-v4*cos(x4(3)); b1_6 = x3(2) - x4(2); b2_6 = v3*sin(x3(3))-v4*sin(x4(3));
dh_dt6 = 2*((a1_6*a2_6) + (b1_6*b2_6));
C6 = + dh_dt6 + alpha * h6;


C=[C1;C2;C3;C4;C5;C6];
constr.expr_h = C;

%% acados ocp model
ocp_model = acados_ocp_model();ocp_model.set('name', model_name);ocp_model.set('T', T);ocp_model.set('sym_x', model.sym_x);ocp_model.set('sym_u', model.sym_u);ocp_model.set('sym_xdot', model.sym_xdot);ocp_model.set('sym_p', model.sym_p);
ocp_model.set('cost_type', cost_type);ocp_model.set('cost_type_e', cost_type);ocp_model.set('cost_expr_ext_cost', model.expr_ext_cost);ocp_model.set('cost_expr_ext_cost_e', model.expr_ext_cost_e);
ocp_model.set('dyn_type', 'explicit');ocp_model.set('dyn_expr_f', model.expr_f_expl);
%% constraints
ocp_model.set('constr_x0', x0);
if 1
if (nh>0)
ocp_model.set('constr_expr_h',constr.expr_h);
ocp_model.set('constr_lh', zeros(6,1)); % Ensure dh_dt + alpha * h is always positive
ocp_model.set('constr_uh', 1e8*ones(6,1));
nsh = 1;
Jsh = 1;
end
end
ocp_model.set('constr_Jbu', Jbu);ocp_model.set('constr_lbu', lbu);ocp_model.set('constr_ubu', ubu);
% ocp_model.set('constr_Jbx', Jbx);ocp_model.set('constr_lbx', lbx);ocp_model.set('constr_ubx', ubx);
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
ocp_opts.set('qp_solver_iter_max', qp_solver_iter_max);ocp_opts.set('sim_method', ocp_sim_method);ocp_opts.set('sim_method_num_stages', ocp_sim_method_num_stages);ocp_opts.set('sim_method_num_steps', ocp_sim_method_num_steps);
ocp_opts.opts_struct
%% acados ocp
ocp = acados_ocp(ocp_model, ocp_opts);
if GENERATE_C_CODE == 1
    ocp.generate_c_code()
end
%% acados sim model
sim_model = acados_sim_model();
% symbolics
sim_model.set('sym_x', model.sym_x);sim_model.set('sym_u', model.sym_u);sim_model.set('sym_xdot', model.sym_xdot);sim_model.set('sym_p', model.sym_p);
% model
sim_model.set('T', T/ocp_N);sim_model.set('dyn_type', 'explicit');sim_model.set('dyn_expr_f', model.expr_f_expl);
%% acados sim opts
sim_opts = acados_sim_opts();sim_opts.set('compile_interface', compile_interface);sim_opts.set('codgen_model', codgen_model);sim_opts.set('num_stages', sim_num_stages);sim_opts.set('num_steps', sim_num_steps);sim_opts.set('method', sim_method);sim_opts.set('sens_forw', sim_sens_forw);
%% acados sim
sim = acados_sim(sim_model,sim_opts);
N_sim = 850;
x_sim = zeros(nx, N_sim+1);x_sim(:,1) = x0; % initial stateu_sim = zeros(nu, N_sim);
% yr_e_new = [115;x0(2)+x0(4)*T;pi/2;x0(4);95;x0(6)-x0(9)*T;5;1.5*pi;x0(9);0;x0(11)-x0(14)*T;10;pi;5;0;x0(16)+x0(19)*T;-10;0;5;0];
% x_traj_init=[linspace(0,x0(4)*T,ocp_N+1);linspace(0,0,ocp_N+1);zeros(1,ocp_N+1);10*ones(1,ocp_N+1);zeros(1,ocp_N+1)];
u_traj_init = zeros(nu, ocp_N); COST=[]; STATUS=[];
    tic;
for ii= 1 : N_sim
    	ocp.set('constr_x0', x_sim(:,ii));
    step_size1=T*x_sim(4,ii);
    step_size2=T*x_sim(9,ii);
    step_size3=T*x_sim(14,ii);
    step_size4=T*x_sim(19,ii);
              y_next1 = linspace(x_sim(2, ii), x_sim(2, ii) + step_size1, ocp_N + 1);
              y_next2 = linspace(x_sim(7, ii), x_sim(7, ii) - step_size2, ocp_N + 1);
              x_next3= linspace(x_sim(11,ii),x_sim(11,ii)-step_size3,ocp_N+1);
              x_next4= linspace(x_sim(16,ii),x_sim(16,ii) + step_size3,ocp_N+1);
x_traj_init = [x0(1) * ones(1, ocp_N + 1);y_next1; (pi/2)*ones(1, ocp_N + 1); x0(4)*ones(1, ocp_N + 1);zeros(1,ocp_N+1); x0(6) * ones(1, ocp_N + 1); y_next2; (1.5*pi)*ones(1, ocp_N + 1); x0(9) * ones(1, ocp_N + 1); zeros(1, ocp_N + 1);x_next3; x0(12)*ones(1,ocp_N+1);(pi)*ones(1, ocp_N + 1); x0(14)*ones(1, ocp_N + 1);zeros(1,ocp_N+1);x_next4;x0(17)*ones(1,ocp_N+1); (0)*ones(1, ocp_N + 1); x0(19)*ones(1, ocp_N + 1);zeros(1,ocp_N+1)];
           p_ref=x_traj_init;
    for jj = 0 : ocp_N - 1
        ocp.set('p', p_ref(:,jj+1), jj);
    end
              ocp.set('p', p_ref(:,end), ocp_N);
% 	ocp.set('init_x', x_traj_init);
% 	ocp.set('init_u', u_traj_init);
	ocp.solve();
	if 1
        disp('')
		status = ocp.get('status');sqp_iter = ocp.get('sqp_iter');time_tot = ocp.get('time_tot');time_lin = ocp.get('time_lin');time_qp_sol = ocp.get('time_qp_sol');
        ii
        cost = ocp.get_cost();COST = [COST cost];STATUS=[STATUS status];
		fprintf('\nstatus = %d, sqp_iter = %d, time_int = %f [ms] (time_lin = %f [ms], time_qp_sol = %f [ms])\n',...
            status, sqp_iter, time_tot*1e3, time_lin*1e3, time_qp_sol*1e3);
        if status~=0
            disp('acados ocp solver failed');
        end
	end
x_traj = ocp.get('x'); u_traj = ocp.get('u'); u_traj_init = [u_traj(:,2:end), u_traj(:,end)];
% yr_e_new = x_traj_init(:,end);
u_sim(:,ii) = ocp.get('u', 0);sim.set('x',x_sim(:,ii));sim.set('u', u_sim(:,ii));sim.solve();x_sim(:,ii+1) = sim.get('xn');
end
avg_time_solve = toc/N_sim
if any(STATUS == 4)
    disp('there have been a status 4!!')
    indices = find(STATUS == 4)
end

%% Plotting
if 1
    figure(100)
    road_width = 40;
road_length = 200;
merging_lane_width = 40;
merging_lane_position = 100;

% Main road
rectangle('Position', [0, -road_width/2, road_length, road_width], 'FaceColor', [0, 1, 1, 0.5]);

% Merging lane
% rectangle('Position', [merging_lane_position - merging_lane_width/2, -road_width, merging_lane_width, road_width], 'FaceColor', 'r');
rectangle('Position', [85 -80 40 140], 'FaceColor', [0, 0, 0, 0.5]);
hold on 
% Plot the starting positions of agents
start_pos_agent1 = [x_sim(1,1), x_sim(2,1)];  % Initial position of Agent 1
start_pos_agent2 = [x_sim(6,1), x_sim(7,1)];  % Initial position of Agent 2
start_pos_agent3 = [x_sim(11,1), x_sim(12,1)];  % Initial position of Agent 1
start_pos_agent4 = [x_sim(16,1), x_sim(17,1)];  % Initial position of Agent 1

plot(start_pos_agent1(1),start_pos_agent1(2),start_pos_agent2(1),start_pos_agent2(2),start_pos_agent3(1),start_pos_agent3(2),start_pos_agent4(1),start_pos_agent4(2));

% Add arrows pointing to the initial positions
quiver(start_pos_agent1(1), start_pos_agent1(2), 0, 5, 'go', 'LineWidth', 1.5, 'MaxHeadSize', 2); % Agent 1
quiver(start_pos_agent2(1), start_pos_agent2(2), 0, 5, 'go', 'LineWidth', 1.5, 'MaxHeadSize', 2); % Agent 2
quiver(start_pos_agent3(1), start_pos_agent3(2), 0, 5, 'go', 'LineWidth', 1.5, 'MaxHeadSize', 2); % Agent 2
quiver(start_pos_agent4(1), start_pos_agent4(2), 0, 5, 'go', 'LineWidth', 1.5, 'MaxHeadSize', 2); % Agent 2


% Add text labels for the initial positions
text(start_pos_agent1(1), start_pos_agent1(2) - 5, 'Agent 1 Start', 'Color', 'k', 'FontSize', 10, 'HorizontalAlignment', 'center');
text(start_pos_agent2(1), start_pos_agent2(2) + 5, 'Agent 2 Start', 'Color', 'k', 'FontSize', 10, 'HorizontalAlignment', 'center');
text(start_pos_agent3(1), start_pos_agent3(2) + 5, 'Agent 3 Start', 'Color', 'k', 'FontSize', 10, 'HorizontalAlignment', 'center');
text(start_pos_agent4(1), start_pos_agent4(2) + 5, 'Agent 4 Start', 'Color', 'k', 'FontSize', 10, 'HorizontalAlignment', 'center');



plot(x_sim(1,1),x_sim(2,1),'bo');
hold on
%%%%%
x_merge_lane = 0:10:200; % Adjust based on the merging lane position and length
y_merge_lane = 0 * ones(size(x_merge_lane)); % Straight Y coordinates for merging dashed line
plot(x_merge_lane, y_merge_lane, 'w--', 'LineWidth', 2); % White dashed line
hold on 
y_merge_lane2 = -80:10:60; % Adjust based on the merging lane position and length
x_merge_lane2 = 105 * ones(size(y_merge_lane2)); % Straight Y coordinates for merging dashed line
plot(x_merge_lane2, y_merge_lane2, 'w--', 'LineWidth', 2); % White dashed line
hold on
plot(x_sim(1,:),x_sim(2,:),'r*',x_sim(6,:),x_sim(7,:),'r*',x_sim(11,:),x_sim(12,:),'bo',x_sim(16,:),x_sim(17,:),'bo');
axis equal
xlabel('x [m]');ylabel('y [m]'); title('Trajectory'); grid on; grid minor;

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
saveas(gcf, 'ContIn1.pdf','pdf');


figure (300)
subplot(3,2,1)
plot(1:N_sim,u_sim(5,:)); title('Acceleration3');grid on; grid minor;
subplot(3,2,2)
plot(1:N_sim,u_sim(6,:)); title ('Steering Rate3');grid on; grid minor;
subplot(3,2,3)
plot(1:N_sim+1,x_sim(14,:)); title ('Velocity3');grid on; grid minor;
subplot(3,2,4)
plot(1:N_sim,u_sim(7,:)); title('Acceleration4');grid on; grid minor;
subplot(3,2,5)
plot(1:N_sim,u_sim(8,:)); title ('Steering Rate4');grid on; grid minor;
subplot(3,2,6)
plot(1:N_sim+1,x_sim(19,:)); title ('Velocity4');grid on; grid minor;
saveas(gcf, 'ContIn2.pdf','pdf');


figure (400)
plot(1:N_sim,COST); title('Cost')
grid on; grid minor;

end
return
%% Animation
if 1
    % Define figure for the animation
    figure(700);
    rectangle('Position', [0, -road_width/2, road_length, road_width], 'FaceColor', [0, 1, 1, 0.5]);


% Merging lane
% rectangle('Position', [merging_lane_position - merging_lane_width/2, -road_width, merging_lane_width, road_width], 'FaceColor', 'r');
rectangle('Position', [85 -80 40 140], 'FaceColor', [0, 1, 0, 0.5])
    axis equal;
    xlabel('x [m]');
    ylabel('y [m]');
    title('Trajectory');
    grid on;
    hold on
    % Plot merging lane dashed line (straight)
x_merge_lane = 0:10:200; % Adjust based on the merging lane position and length
y_merge_lane = 0 * ones(size(x_merge_lane)); % Straight Y coordinates for merging dashed line
plot(x_merge_lane, y_merge_lane, 'w--', 'LineWidth', 2); % White dashed line
hold on 

y_merge_lane2 = -80:10:60; % Adjust based on the merging lane position and length
x_merge_lane2 = 105 * ones(size(y_merge_lane2)); % Straight Y coordinates for merging dashed line
plot(x_merge_lane2, y_merge_lane2, 'w--', 'LineWidth', 2); % White dashed line


    % Define the circle properties
    theta = linspace(0, 2*pi, 100); 
    radius = sqrt(dsafe)/2; % radius is half of the diameter
    x_circle = radius * cos(theta);
    y_circle = radius * sin(theta);

    % Initialize the plot for the car trajectory
    car1_traj = plot(x_sim(1,1), x_sim(2,1), 'r');
    car2_traj = plot(x_sim(6,1), x_sim(7,1), 'r');
    car3_traj = plot(x_sim(11,1), x_sim(12,1), 'b');
    car4_traj = plot(x_sim(16,1), x_sim(17,1), 'b');

    
    % Initialize the plot for the circle representing each vehicle
    car1_circle = plot(x_sim(1,1) + x_circle, x_sim(2,1) + y_circle, 'r');
    car2_circle = plot(x_sim(6,1) + x_circle, x_sim(7,1) + y_circle, 'r');
    car3_circle = plot(x_sim(11,1) + x_circle, x_sim(12,1) + y_circle, 'b');
    car4_circle = plot(x_sim(16,1) + x_circle, x_sim(17,1) + y_circle, 'b');

    
    % Set up the axis limits based on the initial position and range of motion
%     xlim([min(min(x_sim(1,:)), min(x_sim(6,:))) - 30, max(max(x_sim(1,:)), max(x_sim(6,:))) + 30]);
%     ylim([min(min(x_sim(2,:)), min(x_sim(7,:))) - 30, max(max(x_sim(2,:)), max(x_sim(7,:))) + 30]);
    axis equal;
    
    % Animation loop
    for k = 2:length(x_sim)
        % Plot the trajectories of the cars
        set(car1_traj, 'XData', x_sim(1,1:k), 'YData', x_sim(2,1:k));
        set(car2_traj, 'XData', x_sim(6,1:k), 'YData', x_sim(7,1:k));
        set(car3_traj, 'XData', x_sim(11,1:k), 'YData', x_sim(12,1:k));
        set(car4_traj, 'XData', x_sim(16,1:k), 'YData', x_sim(17,1:k));

        
        % Update the positions of the circles representing the vehicles
        set(car1_circle, 'XData', x_sim(1,k) + x_circle, 'YData', x_sim(2,k) + y_circle);
        set(car2_circle, 'XData', x_sim(6,k) + x_circle, 'YData', x_sim(7,k) + y_circle);
        set(car3_circle, 'XData', x_sim(11,k) + x_circle, 'YData', x_sim(12,k) + y_circle);
        set(car4_circle, 'XData', x_sim(16,k) + x_circle, 'YData', x_sim(17,k) + y_circle);


        % Pause to create animation effect
        pause(0.0050); % Adjust the pause time to control the speed of the animation
    end
end

