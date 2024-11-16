%% MPC usign IPOPT
%%%% fix the referencing part!!
clc
clear all
close all
import casadi.*

%% System Parameters
nx = 10;  % Number of state variables
nu = 4;   % Number of control variables
N = 100;   % Prediction horizon
T = 0.01;  % Time step

%% Define Symbolic Variables
x = SX.sym('x', nx, 1);
u = SX.sym('u', nu, 1);
p = SX.sym('p', nx, 1);  % Reference state

L = 2.73;  % Wheelbase [m]

% Define Cost Function Weights
W_x = diag([1e1, 1e3, 1e-2, 1e-1, 1e-5, 1e3, 1e1, 1e-2, 1e-1, 1e-10]);
W_u = diag([1e-3, 1e1, 1e-3, 1e3]);
W_xe = diag([1e1, 1e1, 1e-2, 1e-1, 1e-2, 1e2, 1e1, 1e-5, 1e-1, 1e-5]);

% Dynamics
dx1 = x(4) * cos(x(3));
dy1 = x(4) * sin(x(3));
dtheta1 = (x(4) / L) * tan(x(5));
dv1 = u(1);
ddelta1 = u(2);

dx2 = x(9) * cos(x(8));
dy2 = x(9) * sin(x(8));
dtheta2 = (x(9) / L) * tan(x(10));
dv2 = u(3);
ddelta2 = u(4);

rhs = [dx1; dy1; dtheta1; dv1; ddelta1; dx2; dy2; dtheta2; dv2; ddelta2];
f = Function('f', {x, u}, {rhs});

%% Setup the Cost and Constraints
U = SX.sym('U', nu, N);  % Control input sequence
X = SX.sym('X', nx, N+1);  % State sequence
P = SX.sym('P', nx, N+1);  % Reference states

% Cost
J = SX(0);
g1 = SX([]);  % Dynamics constraints
g2 = SX([]);  % Collision constraints

% Collision avoidance parameters
dsafe = 9;

for k = 1:N
    st = X(:,k);
    con = U(:,k);
    state_ref = P(:, k);
    
    % Cost function
    obj = (st - state_ref)' * W_x * (st - state_ref) + con' * W_u * con;
    J = J + obj;

    % Dynamics
    f_value = f(st, con);
    st_next_euler = st + T * f_value;
    g1 = [g1; X(:,k+1) - st_next_euler];
    
    % Collision Avoidance
    x1 = X(1:5, k);
    x2 = X(6:10, k);
    dist_squared = ((x1(1) - x2(1))^2) + ((x1(2) - x2(2))^2) - dsafe^2;
    g2 = [g2; dist_squared];
end
J = J + (X(:,end) - P(:,end))' * W_xe * (X(:,end) - P(:,end));

g = [g1; g2];

% Decision Variables
w = [reshape(U, nu*N, 1); reshape(X, nx*(N+1), 1)];
P = reshape(P, nx*(N+1) , 1);

% Define NLP Problem
nlp = struct('x', w, 'f', J, 'g', g, 'p', P);

% Set up solver options
opts = struct('ipopt', struct('print_level', 5, 'tol', 1e-6, 'max_iter', 500));
solver = nlpsol('solver', 'ipopt', nlp, opts);

%% MPC Setup
% Initial state
x0 = [110; -20; pi/2; 0; 0; 125; 0.5; pi; 0; 0];

% Bounds
lbx = -inf * ones(nu*N + nx*(N+1), 1);
ubx = inf * ones(nu*N + nx*(N+1), 1);

lbu = repmat([-11.5; -1; -11.5; -1], N, 1);
ubu = repmat([11.5; 1; 11.5; 1], N, 1);
lbx(1:nu*N) = lbu;
ubx(1:nu*N) = ubu;

lbx(nu*N+1:nu*N+nx) = x0;
ubx(nu*N+1:nu*N+nx) = x0;

for k = 1:N+1
    lbx(nu*N + (k-1)*nx + 4) = 0;
    lbx(nu*N + (k-1)*nx + 9) = 0;
end

%%  Reference waypoints
waypoints = Way_IPOPT1();
waypoint_count = N+1;

ref1 = zeros(5, waypoint_count);
ref2 = zeros(5, waypoint_count);

ref1(1, :) = waypoints(1, 1:waypoint_count);
ref1(2, :) = waypoints(2, 1:waypoint_count);
ref1(3, :) = (pi/2)*ones(1, waypoint_count);
ref1(4, :) = 2 * ones(1, waypoint_count);
ref1(5, :) = zeros(1, waypoint_count);

ref2(1, :) = linspace(125, 125 - 4*waypoint_count*T, waypoint_count);
ref2(2, :) = 0.5*ones(1, waypoint_count);
ref2(3, :) = pi * ones(1, waypoint_count);
ref2(4, :) = 2 * ones(1, waypoint_count);
ref2(5, :) = zeros(1, waypoint_count);

p_val = [ref1; ref2];
p_val = reshape(p_val, [], 1);

% Initial guess
u0_guess = zeros(nu*N, 1);
w0 = [u0_guess; p_val];

% Loop for MPC
num_steps = 300;  % Number of control steps
x_current = x0;   % Initial state
x_current_numeric=x0;
X_sim=[];U_sim=[];
ref1_end=ref1(:,end);
ref2_end=ref2(:,end);
for step = 1:num_steps
    % Update reference trajectory
    ref1(:,1:end-1) = ref1(:,2:end);   % Shift reference states for Vehicle 1
    ref2(:,1:end-1) = ref2(:,2:end);   % Shift reference states for Vehicle 2

    % New reference state for the next position
%     ref1(:,end) = x_current_numeric(1:5);
%     ref2(:,end) = x_current_numeric(6:10);
    ref1(:, end) = ref1(:, end);  % Or some desired target/reference value
    ref2(:, end) = ref2(:, end);  % Or some desired target/reference value

    % Update reference trajectory
    p_val = [ref1; ref2];
    p_val = reshape(p_val, [], 1);

    % Solve the optimization problem
    lbg = zeros(size(g));
    ubg = inf(size(g));
    lbg(1:numel(X(:,1))) = 0;
    ubg(1:length(g1)) = 0;
    lbg(numel(X(:,1))+1:end) = 0;
    ubg(length(g1):end) = inf;

    sol = solver('x0',w0,'lbx', lbx, 'ubx', ubx, 'lbg', lbg, 'ubg', ubg, 'p', p_val);

    % Extract the optimal solution
    w_opt = full(sol.x);
    u_opt = reshape(w_opt(1:nu*N), nu, N);
    x_opt = reshape(w_opt(nu*N+1:end), nx, []);

    % Apply the first control input
    u_current = u_opt(:,1);  % Control input for the current time step
    U_sim=[U_sim,u_current];
    
    % Update the state using the dynamics (for simulation purposes)
    f_value = f(x_current, u_current);
    x_next = x_current + T * f_value;
    
    % Update the current state
    x_current = x_next;
    x_current_numeric=full(x_current);

    X_sim=[X_sim,x_current_numeric];

    % Update initial guess for the next iteration
if N > 1
    % Shift control inputs and states if N > 1
    u_opt_shifted = [u_opt(:,2:end), u_opt(:,end)]; % Shifted control inputs
    x_opt_shifted = [x_opt(:,2:end), x_opt(:,end)]; % Shifted states
else
    % If N is 1, just repeat the single entry
    u_opt_shifted = [u_opt, u_opt]; % Duplicate the single entry
    x_opt_shifted = [x_opt, x_opt]; % Duplicate the single entry
end
        w0 = [reshape(u_opt_shifted , nu*N, 1); reshape(x_opt_shifted, nx*(N+1), 1)];

    
    % Optionally: Plot results or log data
end
%% Plotting
road_width = 20;road_length = 200;merging_lane_width = 10;merging_lane_position = 100;
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
plot(X_sim(1,:),X_sim(2,:),'r--',X_sim(6,:),X_sim(7,:),'b--');


%% Control inputs
figure(2)
subplot(2,2,1)
plot(1:num_steps,U_sim(1,:));
title('A1')
subplot(2,2,2)
plot(1:num_steps,U_sim(2,:));
title('S1')
subplot(2,2,3)
plot(1:num_steps,U_sim(3,:));
title('A2');
subplot(2,2,4)
plot(1:num_steps,U_sim(4,:));
title('S2')

