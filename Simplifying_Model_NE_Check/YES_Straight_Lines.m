clc
clear all
close all
import casadi.*;
%%

% Define system dimensions
nx = 10;  % Number of state variables
nu = 4;   % Number of control variables

% Define symbolic variables for states, controls, and parameters
x = SX.sym('x', nx, 1);
u = SX.sym('u', nu, 1);
p = SX.sym('p', nx, 1);  % Reference state

% Wheelbase
L = 2.73;  % Wheelbase [m]

% Define cost function weights
W_x = diag([1e1, 1e1, 1e-2, 1e-1, 1e-5, 1e3, 1e1, 1e-2, 1e-1, 1e-10]);
W_u = diag([1e-2, 1e-2, 1e-3, 1e-3]);
W_xe = diag([1e1, 1e1, 1e-2, 1e-1, 1e-2, 1e1, 1e1, 1e-5, 1e-1, 1e-5]);

% Define symbolic variables for states and controls
states = SX.sym('states', nx, 1);
controls = SX.sym('controls', nu, 1);

% Define the system dynamics using the right-hand side equations
dx1 = states(4) * cos(states(3));
dy1 = states(4) * sin(states(3));
dtheta1 = (states(4) / L) * tan(states(5));
dv1 = controls(1);
ddelta1 = controls(2);

dx2 = states(9) * cos(states(8));
dy2 = states(9) * sin(states(8));
dtheta2 = (states(9) / L) * tan(states(10));
dv2 = controls(3);
ddelta2 = controls(4);

% Right-hand side of the dynamics
rhs = [dx1; dy1; dtheta1; dv1; ddelta1; dx2; dy2; dtheta2; dv2; ddelta2];

% Create a CasADi function for the system dynamics
f = Function('f', {states, controls}, {rhs});

% Define the cost expressions
state_cost = (x - p)' * W_x * (x - p);
input_cost = u' * W_u * u;
terminal_cost = (x - p)' * W_xe * (x - p);

% Total cost
cost = state_cost + input_cost;

N = 200; % Number of intervals in the prediction horizon
T = 0.02; % Length of each interval (sampling time)

% Initialize symbolic variables for control inputs and states
U = SX.sym('U', nu, N);  % Control input sequence
X = SX.sym('X', nx, N+1);  % State sequence
P = SX.sym('P', nx, N+1);  % Reference states

% Initialize cost and constraints as symbolic expressions
J = SX(0);  % Initialize cost as symbolic zero
g1 = SX([]);  % Initialize constraints as an empty symbolic array
g2 = SX([]);  % Initialize constraints as an empty symbolic array
g = SX([]);  % Initialize constraints as an empty symbolic array


% Collision avoidance parameters
dsafe = 2.5;

% Set up the cost and constraints over the prediction horizon
for k = 1:N
    % Define current state and control
    st = X(:,k);
    con = U(:,k);
    
    % Reference state for the current interval
    state_ref = P(:, k);
    
    % Update the cost
    obj = (st - state_ref)' * W_x * (st - state_ref) + con' * W_u * con;
    J = J + obj;

    % Compute the next state using the dynamics function
    f_value = f(st, con);
    st_next_euler = st + T * f_value;
    
    % Dynamics constraint (equality constraint)
    g1 = [g1; X(:,k+1) - st_next_euler];

    % Collision avoidance constraint (inequality constraint)
    x1 = X(1:5, k);
    x2 = X(6:10, k);
    dist_squared = ((x1(1) - x2(1))^2) + ((x1(2) - x2(2))^2) - dsafe^2;
    g2 = [g2; dist_squared];
end
g=[g1;g2];

% Add terminal cost
J = J + (X(:,end) - P(:,end))' * W_xe * (X(:,end) - P(:,end));

% Vectorize the decision variables
w = [reshape(U, nu*N, 1); reshape(X, nx*(N+1), 1)];
P = reshape(P, nx*(N+1), 1);

% Define CasADi function for optimization
opts = struct('ipopt', struct('print_level', 5, 'tol', 1e-6, 'max_iter', 1500));

% Define the NLP problem
nlp = struct('x', w, 'f', J, 'g', g, 'p', P);

% Initialize solver
solver = nlpsol('solver', 'ipopt', nlp, opts);
%%

% Initial conditions and constraints
x0 = [110; 0.50; 0; 0; 0; 125; 0.5; pi; 0; 0];
lbx = -inf * ones(nu*N + nx*(N+1), 1);
ubx = inf * ones(nu*N + nx*(N+1), 1);

% Bounds on controls
lbu = repmat([-11.5; -1; -11.5; -1], N, 1);
ubu = repmat([11.5; 1; 11.5; 1], N, 1);

% Apply bounds to decision variables
lbx(1:nu*N) = lbu;
ubx(1:nu*N) = ubu;

% Initial state constraints
lbx(nu*N+1:nu*N+nx) = x0;
ubx(nu*N+1:nu*N+nx) = x0;

% Positive velocity constraints for the entire horizon
for k = 1:N+1
    lbx(nu*N + (k-1)*nx + 4) = 0; % x4 >= 0
    lbx(nu*N + (k-1)*nx + 9) = 0; % x9 >= 0
end

% Initial guess for the decision variables
% x0_guess = repmat(x0, N+1, 1);
% u0_guess = zeros(nu*N, 1);
% w0 = [u0_guess; x0_guess];

% Load reference waypoints
% waypoints = Way();
waypoint_count = N+1;

% Prepare reference states for both vehicles
ref1 = zeros(5, waypoint_count);
ref2 = zeros(5, waypoint_count);

% Vehicle 1 reference state
% ref1(1, :) = waypoints(1, 1:waypoint_count);  % x positions
% ref1(2, :) = waypoints(2, 1:waypoint_count);  % y positions
% ref1(3, :) = (pi/2)*ones(1, waypoint_count);  % orientations
% ref1(4, :) = 2 * ones(1, waypoint_count);  % constant velocity
% ref1(5, :) = zeros(1, waypoint_count);  % steering angle
ref1(1, :) = linspace(110, 110 + 4*waypoint_count*T, waypoint_count);  % x positions (straight line)
ref1(2, :) = 0.5*ones(1, waypoint_count);  % y positions (constant)
ref1(3, :) = 0 * ones(1, waypoint_count);  % orientations
ref1(4, :) = 2 * ones(1, waypoint_count);  % constant velocity
ref1(5, :) = zeros(1, waypoint_count);  % steering angle

% Vehicle 2 reference state
ref2(1, :) = linspace(125, 125 - 4*waypoint_count*T, waypoint_count);  % x positions (straight line)
ref2(2, :) = 0.5*ones(1, waypoint_count);  % y positions (constant)
ref2(3, :) = pi * ones(1, waypoint_count);  % orientations
ref2(4, :) = 4 * ones(1, waypoint_count);  % constant velocity
ref2(5, :) = zeros(1, waypoint_count);  % steering angle

% Combine reference states
p_val = [ref1; ref2];
p_val = reshape(p_val, [], 1);


% Solve the NLP
% Define bounds for equality and inequality constraints
lbg = zeros(size(g)); % Lower bounds for g
ubg = inf(size(g));   % Upper bounds for g

% Set equality bounds for dynamics constraints
lbg(1:numel(X(:,1))) = 0;
ubg(1:length(g1)) = 0;

% Set inequality bounds for collision avoidance constraints
lbg(numel(X(:,1))+1:end) = 0;
ubg(length(g1):end) = inf;

% Solve the NLP
sol = solver('lbx', lbx, 'ubx', ubx, 'lbg', lbg, 'ubg', ubg, 'p', p_val);


w_opt = full(sol.x);
%% Reshaping the solution
        u_opt = reshape(w_opt(1:nu*N), nu, N);
        x_opt = reshape(w_opt(nu*N+1:end), nx, []);

%% Plotting
road_width = 20;road_length = 200;merging_lane_width = 10;merging_lane_position = 100;
% Plot road
figure;
hold on;
% Main road
% rectangle('Position', [0, -road_width/2, road_length, road_width], 'FaceColor', 'g');

% Merging lane
% rectangle('Position', [merging_lane_position - merging_lane_width/2, -road_width, merging_lane_width, road_width], 'FaceColor', 'r');
% rectangle('Position', [95 -60 20 50], 'FaceColor', 'c');

% Set axis properties
axis equal;
xlabel('x [m]');
ylabel('y [m]');
title('Road');
grid on; grid minor
hold on
plot(x_opt(1,:),x_opt(2,:),'r--',x_opt(6,:),x_opt(7,:),'b--');
xlim([100 130])
ylim([-5 5])
legend('agent 1','agent 2')
return
%% Control bounds Plot
figure (200)
subplot(3,2,1)
plot(1:N,u_opt(1,:)); title('Acceleration1');
subplot(3,2,2)
plot(1:N,u_opt(2,:)); title ('Steering Rate1');
subplot(3,2,3)
plot(1:N+1,x_opt(4,:)); title ('Velocity1');
subplot(3,2,4)
plot(1:N,u_opt(3,:)); title('Acceleration2');
subplot(3,2,5)
plot(1:N,u_opt(4,:)); title ('Steering Rate2');
subplot(3,2,6)
plot(1:N+1,x_opt(9,:)); title ('Velocity2');

% figure (300)
% plot(1:N_sim,COST); title('Cost')
% grid on
%% Animation
if 1
    % Define figure for the animation
    figure(700);
    rectangle('Position', [0, -road_width/2, road_length, road_width], 'FaceColor', [0, 1, 1, 0.5]);

% Merging lane
% % rectangle('Position', [merging_lane_position - merging_lane_width/2, -road_width, merging_lane_width, road_width], 'FaceColor', 'r');
% rectangle('Position', [105 -40 10 30], 'FaceColor', [0, 1, 0, 0.5])
    axis equal;
    xlabel('x [m]');
    ylabel('y [m]');
    title('Trajectory');
    grid on;
    hold on;

    % Define the circle properties
    theta = linspace(0, 2*pi, 100);
    radius = sqrt(dsafe/2); % radius is half of the diameter
    x_circle = radius * cos(theta);
    y_circle = radius * sin(theta);

    % Initialize the plot for the car trajectory
    car1_traj = plot(x_opt(1,1), x_opt(2,1), 'r');
    car2_traj = plot(x_opt(6,1), x_opt(7,1), 'b');
    
    % Initialize the plot for the circle representing each vehicle
    car1_circle = plot(x_opt(1,1) + x_circle, x_opt(2,1) + y_circle, 'r');
    car2_circle = plot(x_opt(6,1) + x_circle, x_opt(7,1) + y_circle, 'b');
    
    % Set up the axis limits based on the initial position and range of motion
%     xlim([min(min(x_sim(1,:)), min(x_sim(6,:))), max(max(x_sim(1,:)), max(x_sim(6,:))) ]);
%     ylim([min(min(x_sim(2,:)), min(x_sim(7,:))) , max(max(x_sim(2,:)), max(x_sim(7,:))) ]);
    axis equal;
    
    % Animation loop3
    for k = 2:length(x_opt)
        % Plot the trajectories of the cars
        set(car1_traj, 'XData', x_opt(1,k), 'YData', x_opt(2,k));
        set(car2_traj, 'XData', x_opt(6,k), 'YData', x_opt(7,k));
        
        % Update the positions of the circles representing the vehicles
        set(car1_circle, 'XData', x_opt(1,k) + x_circle, 'YData', x_opt(2,k) + y_circle);
        set(car2_circle, 'XData', x_opt(6,k) + x_circle, 'YData',x_opt(7,k) + y_circle);

        % Pause to create animation effect
        pause(0.05); % Adjust the pause time to control the speed of the animation
    end
end


