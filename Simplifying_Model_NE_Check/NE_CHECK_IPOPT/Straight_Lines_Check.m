% Extract the optimal trajectory for Vehicle 2 from the joint solution
clc
clear all
close all
import casadi.*;
%%
L = 2.73;  % Wheelbase [m]
T=0.02;
dsafe=2.50;
X_opt = load('Straight_Lines_Traj.mat');
h=0;
if h
X2_opt=X_opt.x_opt(6:10,:);
else
    X2_opt=X_opt.x_opt(1:5,:);
end
% X_opt = reshape(w_opt(nu*N + 1:end), nx, N+1);
nu=2;nx=5;
N=200;
% X2_opt = X_opt(6:10, :);

% Define a new OCP for Vehicle 1, fixing the trajectory of Vehicle 2
states1 = SX.sym('states1', 5, 1); % State variables for Vehicle 1
controls1 = SX.sym('controls1', 2, 1); % Control variables for Vehicle 1

% Define the dynamics for Vehicle 1
dx1 = states1(4) * cos(states1(3));
dy1 = states1(4) * sin(states1(3));
dtheta1 = (states1(4) / L) * tan(states1(5));
dv1 = controls1(1);
ddelta1 = controls1(2);

rhs1 = [dx1; dy1; dtheta1; dv1; ddelta1];
f1 = Function('f1', {states1, controls1}, {rhs1});

% Define new cost function weights for Vehicle 1
if h
W_x1 = diag([1e1, 1e1, 1e-2, 1e-1, 1e-5]);
W_xe = diag([1e1, 1e1, 1e-2, 1e-1, 1e-2]);
W_u1 = diag([1e-2, 1e-2]);
else
    W_x1 = diag([1e3, 1e1, 1e-2, 1e-1, 1e-10]);
W_u1 = diag([1e-3, 1e-3]);
W_xe = diag([1e1, 1e1, 1e-5, 1e-1, 1e-5]);
end
% Initialize symbolic variables for Vehicle 1 OCP
U1 = SX.sym('U1', 2, N); % Control input sequence for Vehicle 1
X1 = SX.sym('X1', 5, N+1); % State sequence for Vehicle 1
P1 = SX.sym('P1', 5, N+1); % Reference states for Vehicle 1

% Initialize cost and constraints for Vehicle 1 OCP
J1 = SX(0);
g1 = SX([]);
g2 = SX([]);
g = SX([]);


% Set up the cost and constraints over the prediction horizon for Vehicle 1
for k = 1:N
    st1 = X1(:,k);
    con1 = U1(:,k);
    state_ref1 = P1(:, k);
    
    % Update the cost
    obj1 = (st1 - state_ref1)' * W_x1 * (st1 - state_ref1) + con1' * W_u1 * con1;
    J1 = J1 + obj1;

    % Compute the next state using the dynamics function
    f_value1 = f1(st1, con1);
    st_next_euler1 = st1 + T * f_value1;
    
    % Dynamics constraint (equality constraint)
    g1 = [g1; X1(:,k+1) - st_next_euler1];

    % Collision avoidance constraint with fixed Vehicle 2 trajectory
    x1 = X1(1:5, k);
    x2 = X2_opt(:, k);
    dist_squared = ((x1(1) - x2(1))^2) + ((x1(2) - x2(2))^2) - dsafe^2;
    g2 = [g2; dist_squared];
end
g=[g1;g2];

% Add terminal cost for Vehicle 1
J1 = J1 + (X1(:,end) - P1(:,end))' * W_xe * (X1(:,end) - P1(:,end));

% Vectorize the decision variables for Vehicle 1 OCP
w1 = [reshape(U1, 2*N, 1); reshape(X1, 5*(N+1), 1)];
P1 = reshape(P1, 5*(N+1), 1);

% Define the new NLP problem for Vehicle 1
opts = struct('ipopt', struct('print_level', 5, 'tol', 1e-6, 'max_iter', 1500));
nlp1 = struct('x', w1, 'f', J1, 'g', g, 'p', P1);

% Initialize solver for Vehicle 1
solver1 = nlpsol('solver1', 'ipopt', nlp1, opts);

%
%% Prepare reference states for Vehicle 1
waypoint_count = N+1;
if h
% waypoints = Way();
ref1 = zeros(5, waypoint_count);
% Vehicle 1 reference state
ref1(1, :) = linspace(110, 110 + 4*waypoint_count*T, waypoint_count);  % x positions (straight line)
ref1(2, :) = 0.5*ones(1, waypoint_count);  % y positions (constant)
ref1(3, :) = 0 * ones(1, waypoint_count);  % orientations
ref1(4, :) = 2 * ones(1, waypoint_count);  % constant velocity
ref1(5, :) = zeros(1, waypoint_count);  % steering angle
else 
% Vehicle 2 reference state
ref2 = zeros(5, waypoint_count);
ref2(1, :) = linspace(125, 125 - 4*waypoint_count*T, waypoint_count);  % x positions (straight line)
ref2(2, :) = 0.5*ones(1, waypoint_count);  % y positions (constant)
ref2(3, :) = pi * ones(1, waypoint_count);  % orientations
ref2(4, :) = 4 * ones(1, waypoint_count);  % constant velocity
ref2(5, :) = zeros(1, waypoint_count);  % steering angle angle
ref1=ref2;
end

p1_val = ref1;
p1_val = reshape(p1_val, [], 1);
%%
if h
x0 = [110; 0.5; 0; 0; 0];
else 
    x0 = [125; 0.5; pi; 0; 0];
end

lbx1 = -inf * ones(nu*N + nx*(N+1), 1);
ubx1 = inf * ones(nu*N + nx*(N+1), 1);

% Bounds on controls
lbu1 = repmat([-11.5; -1], N, 1);
ubu1 = repmat([11.5; 1], N, 1);

% Apply bounds to decision variables
lbx1(1:nu*N) = lbu1;
ubx1(1:nu*N) = ubu1;

% Initial state constraints
lbx1(nu*N+1:nu*N+nx) = x0;
ubx1(nu*N+1:nu*N+nx) = x0;

% Positive velocity constraints for the entire horizon
for k = 1:N
    lbx1(nu*N + (k-1)*nx + 4) = 0; % x4 >= 0
    lbx1(nu*N + (k-1)*nx + 9) = 0; % x9 >= 0
end
% lbg1 = zeros(size(g)); % Lower bounds for g
% ubg1 = inf(size(g));   % Upper bounds for g
lbg1 = zeros(size(g1)); % Lower bounds for g1 (dynamics constraints)
ubg1 = zeros(size(g1)); % Upper bounds for g1 (dynamics constraints)
lbg2 = zeros(size(g2)); % Lower bounds for g2 (collision avoidance constraints)
ubg2 = inf(size(g2));   % Upper bounds for g2 (collision avoidance constraints)
lbg = [lbg1; lbg2];
ubg = [ubg1; ubg2];

% Set equality bounds for dynamics constraints
% lbg1(1:numel(X1(:,1))) = 0;
% ubg1(1:length(g1)) = 0;
% 
% % Set inequality bounds for collision avoidance constraints
% lbg1(numel(X1(:,1))+1:end) = 0;
% ubg1(length(g1):end) = inf;

%%
% Solve the NLP for Vehicle 1
sol1 = solver1('lbx', lbx1, 'ubx', ubx1, 'lbg', lbg, 'ubg', ubg, 'p', p1_val);
w1_opt = full(sol1.x);

% Extract the optimal trajectory for Vehicle 1
X1_opt = reshape(w1_opt(2*N + 1:end), 5, N+1);
x_opt=X1_opt;
%%
%% Plotting
road_width = 20;road_length = 200;merging_lane_width = 10;merging_lane_position = 100;
% Plot road
figure;
hold on;
% Main road
rectangle('Position', [0, -road_width/2, road_length, road_width], 'FaceColor', 'g');

% Merging lane
% rectangle('Position', [merging_lane_position - merging_lane_width/2, -road_width, merging_lane_width, road_width], 'FaceColor', 'r');
% rectangle('Position', [95 -60 20 50], 'FaceColor', 'c');

% Set axis properties
axis equal;
xlabel('x [m]');
ylabel('y [m]');
title('Road');
grid on;
hold on
plot(x_opt(1,:),x_opt(2,:),'r--',X2_opt(1,:),X2_opt(2,:),'b--');
xlim([100 130])
ylim([-5 5])
