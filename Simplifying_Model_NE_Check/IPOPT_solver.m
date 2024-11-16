clc
clear all
close all
% check the dimensions and the reshaping of the solution!!
% check specifically the sol.g and sol.x!! 
%%
import casadi.*

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
W_x = diag([1e3, 1e3, 1e-2, 1e-2, 1e-5, 1e3, 1e1, 1e-2, 1e-2, 1e-10]);
W_u = diag([1e-1, 1e-2, 1e-3, 1e-1]);
W_xe = diag([1e5, 1e5, 1e-2, 1e2, 1e-2, 1e2, 1e1, 1e-5, 1e-1, 1e-5]);

% Define cost expressions
state_cost = (x - p)' * W_x * (x - p);
input_cost = u' * W_u * u;
terminal_cost = (x - p)' * W_xe * (x - p);

% Total cost
cost = state_cost + input_cost;

N = 10; % Number of intervals in the prediction horizon
T = 0.02; % Length of each interval (sampling time)

% Initialize symbolic variables for control inputs and states
U = SX.sym('U', nu, N);  % Control input sequence
X = SX.sym('X', nx, N+1);  % State sequence

% Initialize cost and constraints as symbolic expressions
J = SX(0);  % Initialize cost as symbolic zero
g1 = SX([]);  % Initialize constraints as an empty symbolic array
g2 = SX([]);
g  = SX([]);

% Collision avoidance parameters
dsafe = 4.0;

% Set up the cost and constraints over the prediction horizon
for k = 1:N
    % Define system dynamics using current state and control variables
    xk = X(:,k);
    uk = U(:,k);
    
    dx1 = xk(4) * cos(xk(3));
    dy1 = xk(4) * sin(xk(3));
    dtheta1 = (xk(4) / L) * tan(xk(5));
    dv1 = uk(1);
    ddelta1 = uk(2);

    dx2 = xk(9) * cos(xk(8));
    dy2 = xk(9) * sin(xk(8));
    dtheta2 = (xk(9) / L) * tan(xk(10));
    dv2 = uk(3);
    ddelta2 = uk(4);

    % State transition equations
    f = [dx1; dy1; dtheta1; dv1; ddelta1; dx2; dy2; dtheta2; dv2; ddelta2];
    
    % Integrate dynamics using Euler method
    x_next = X(:,k) + T * f; % Ensure `f` uses symbolic variables from X(:,k) and U(:,k)
    
    % Add stage cost
    J = J + (X(:,k) - p)' * W_x * (X(:,k) - p) + U(:,k)' * W_u * U(:,k);
    
    % Add collision avoidance constraint
    x1 = X(1:5, k);
    x2 = X(6:10, k);
    dist_squared = ((x1(1) - x2(1))^2) + ((x1(2) - x2(2))^2) - dsafe^2;
    g = [g; dist_squared];
end

% Add terminal cost
J = J + (X(:,end) - p)' * W_xe * (X(:,end) - p);

% Vectorize the decision variables
w = [reshape(U, nu*N, 1); reshape(X, nx*(N+1), 1)];

% Check symbolic inputs
disp(['w: ', class(w)]);
disp(['J: ', class(J)]);
disp(['g: ', class(g)]);
disp(['p: ', class(p)]);

% Define CasADi function for optimization
opts = struct('ipopt', struct('print_level', 5, 'tol', 1e-6, 'max_iter', 500));

% Check that all parts of the NLP are symbolic before calling nlpsol
try
    nlp = struct('x', w, 'f', J, 'g', g, 'p', p);
    disp('NLP structure created successfully with symbolic expressions.');
catch ME
    disp(['Error creating NLP structure: ', ME.message]);
end

% Initialize solver
try
    solver = nlpsol('solver', 'ipopt', nlp, opts);
    disp('Solver created successfully.');
catch ME
    disp(['Error creating solver: ', ME.message]);
end
keyboard
%%
% Initial conditions and constraints
x0 = [110; -20; pi/2; 2; 0; 125; 0; pi; 4; 0];
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

% Initial guess for the decision variables
x0_guess = repmat(x0, N+1, 1);
u0_guess = zeros(nu*N, 1);
w0 = [u0_guess; x0_guess];
%%
% Reference state (example)
% ref = [115; -15; pi/2; 5; 0; 130; 5; pi/2; 5; 0]; % Adjust this to your actual reference state
% ref=Way();
% Ref=[zeros(1,N)]
% p_val = ref;
% Load reference waypoints
waypoints = Way();
% nn=N;
% waypoint_count = size(waypoints, 2);
waypoint_count = N+1;

% Prepare reference states for both vehicles
ref1 = zeros(5, waypoint_count);
ref2 = zeros(5, waypoint_count);

% Vehicle 1 reference state
ref1(1, :) = waypoints(1, 1:waypoint_count);  % x positions
ref1(2, :) = waypoints(2, 1:waypoint_count);  % y positions
ref1(3, :) = (pi/2)*ones(1, waypoint_count);  % orientations
ref1(4, :) = 2 * ones(1, waypoint_count);  % constant velocity
ref1(5, :) = zeros(1, waypoint_count);  % steering angle

% Vehicle 2 reference state
ref2(1, :) = linspace(125, 125 - 4*waypoint_count*T, waypoint_count);  % x positions (straight line)
ref2(2, :) = zeros(1, waypoint_count);  % y positions (constant)
ref2(3, :) = pi * ones(1, waypoint_count);  % orientations
ref2(4, :) = 4 * ones(1, waypoint_count);  % constant velocity
ref2(5, :) = zeros(1, waypoint_count);  % steering angle

% Combine reference states
p_val = [ref1; ref2];
%%

% Solve the NLP
sol = solver('lbx', lbx, 'ubx', ubx, 'p', p_val);
keyboard
w_opt = full(sol.x);
%%
% Verify and debug dimensions
if 0
total_length = length(w_opt);
expected_length = nu * N + nx * (N + 1);

disp(['Total length of w_opt: ', num2str(total_length)]);
disp(['Expected length: ', num2str(expected_length)]);

if total_length ~= expected_length
    error('Mismatch in expected and actual lengths. Check the dimensions of the decision variable vector.');
else
    % Additional debugging prints
    disp('Length of control part: ');
    disp(length(w_opt(1:nu*N)));
    disp('Expected control part length: ');
    disp(nu * N);

    disp('Length of state part: ');
    disp(length(w_opt(nu*N+1:end)));
    disp('Expected state part length: ');
    disp(nx * (N + 1));

    % Reshape if lengths match
    if length(w_opt(1:nu*N)) == nu * N && length(w_opt(nu*N+1:end)) == nx * (N + 1)
        u_opt = reshape(w_opt(1:nu*N), nu, N);
        x_opt = reshape(w_opt(nu*N+1:end), nx, []);

        % Display results
        disp('Optimal control inputs:');
        disp(u_opt);
        disp('Optimal state trajectory:');
        disp(x_opt);
    else
        error('Length mismatch in control or state part.');
    end
end
end
%%
% w_opt_corrected = w_opt(:, end);  % Extract the last column if it contains the relevant data
x_my1=[];
y_my1=[];
x_my2=[];
y_my2=[];
for j= 1:N+1
    x_my1=[x_my1 w_opt(N*nu+1+10,j)];
    y_my1=[y_my1 w_opt(N*nu+1+1+10,j)];
    x_my2=[x_my2 w_opt(N*nu+1+5+10,j)];
    y_my2=[y_my2 w_opt(N*nu+1+6+10,j)];
end
figure;
hold on;
road_width = 20;
road_length = 200;
merging_lane_width = 10;
merging_lane_position = 100;

% Main road
rectangle('Position', [0, -road_width/2, road_length, road_width], 'FaceColor', 'g');

% Merging lane
% rectangle('Position', [merging_lane_position - merging_lane_width/2, -road_width, merging_lane_width, road_width], 'FaceColor', 'r');
rectangle('Position', [95 -60 20 50], 'FaceColor', 'c');

% Set axis properties
axis equal;
xlabel('x [m]');
ylabel('y [m]');
title('Road');
grid on;
hold on
plot(x_my1,y_my1,'r--',x_my2,y_my2,'b--');
%% 
return

w_opt_corrected = w_opt(:, end);  % Extract the last column if it contains the relevant data
%%%% Extracted data is wrong, fix it ya hmar!



% Extract optimal control and state trajectories
u_opt = reshape(w_opt_corrected(1:nu*N), nu, N);
x_opt = reshape(w_opt_corrected(nu*N+1:end), nx, N+1);

% Display results
disp('Optimal control inputs:');
disp(u_opt);
disp('Optimal state trajectory:');
disp(x_opt);
% Vectorize decision variables correctly

road_width = 20;
road_length = 200;
merging_lane_width = 10;
merging_lane_position = 100;

% Plot road
figure;
hold on;

% Main road
rectangle('Position', [0, -road_width/2, road_length, road_width], 'FaceColor', 'g');

% Merging lane
% rectangle('Position', [merging_lane_position - merging_lane_width/2, -road_width, merging_lane_width, road_width], 'FaceColor', 'r');
rectangle('Position', [95 -60 20 50], 'FaceColor', 'c');

% Set axis properties
axis equal;
xlabel('x [m]');
ylabel('y [m]');
title('Road');
grid on;
hold on
plot(x_opt(1,:),x_opt(2,:),'r--',x_opt(6,:),x_opt(7,:),'b--');
%% Animation
if 1
    % Define figure for the animation
    figure(700);
    rectangle('Position', [0, -road_width/2, road_length, road_width], 'FaceColor', [0, 1, 1, 0.5]);

% Merging lane
% rectangle('Position', [merging_lane_position - merging_lane_width/2, -road_width, merging_lane_width, road_width], 'FaceColor', 'r');
rectangle('Position', [105 -40 10 30], 'FaceColor', [0, 1, 0, 0.5])
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
    car1_traj = plot(x_my1(1), y_my1(1), 'r');
    car2_traj = plot(x_my2(1), y_my2(1), 'b');
    
    % Initialize the plot for the circle representing each vehicle
    car1_circle = plot(x_my1(1) + x_circle, y_my1(1) + y_circle, 'r');
    car2_circle = plot(x_my2(1) + x_circle, y_my2(1) + y_circle, 'b');
    
    % Set up the axis limits based on the initial position and range of motion
%     xlim([min(min(x_sim(1,:)), min(x_sim(6,:))), max(max(x_sim(1,:)), max(x_sim(6,:))) ]);
%     ylim([min(min(x_sim(2,:)), min(x_sim(7,:))) , max(max(x_sim(2,:)), max(x_sim(7,:))) ]);
    axis equal;
    
    % Animation loop
    for k = 2:length(x_my1)
        % Plot the trajectories of the cars
        set(car1_traj, 'XData', x_my1(k), 'YData', y_my1(k));
        set(car2_traj, 'XData', x_my2(k), 'YData', y_my2(k));
        
        % Update the positions of the circles representing the vehicles
        set(car1_circle, 'XData', x_my1(k) + x_circle, 'YData', y_my1(k) + y_circle);
        set(car2_circle, 'XData', x_my2(k) + x_circle, 'YData',y_my2(k) + y_circle);

        % Pause to create animation effect
        pause(0.015); % Adjust the pause time to control the speed of the animation
    end
end
