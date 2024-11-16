%% IPOPT solver
%Testing the typical problem that I am solving with acados using the IPOPT
%solver
%% Dynamics
import casadi.*

% Define system dimensions
nx = 10;  % Number of state variables
nu = 4;   % Number of control variables

% Define symbolic variables for states, controls, and parameters
x = SX.sym('x', nx, 1);
u = SX.sym('u', nu, 1);
p = SX.sym('p', nx, 1);  % Reference state

% Define system dynamics
L = 2.73;  % Wheelbase [m]

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

% State transition equations
f = [dx1; dy1; dtheta1; dv1; ddelta1; dx2; dy2; dtheta2; dv2; ddelta2];

%% Cost Function Set up
% Define cost function weights
W_x = diag([1e3, 1e3, 1e-2, 1e-2, 1e-5, 1e3, 1e1, 1e-2, 1e-2, 1e-10]);
W_u = diag([7.5, 1e-1, 1e-3, 1e-1]);
W_xe = diag([1e5, 1e5, 1e-2, 1e2, 1e-2, 1e2, 1e1, 1e-5, 1e-1, 1e-5]);

% Define cost expressions
state_cost = (x - p)' * W_x * (x - p);
input_cost = u' * W_u * u;
terminal_cost = (x - p)' * W_xe * (x - p);

% Total cost
cost = state_cost + input_cost;

% Define CasADi functions for dynamics and cost
f_fun = Function('f', {x, u}, {f});
% cost_fun = Function('cost', {x, u, p}, {cost});
cost_fun = Function('cost', {x, u, p}, {cost});
terminal_cost_fun = Function('terminal_cost', {x, p}, {terminal_cost});

%% OCP
N = 20;
T = 0.02;

% Initialize variables
U = SX.sym('U', nu, N);  % Control input sequence
X = SX.sym('X', nx, N+1);  % State sequence

% Initialize cost and constraints
J = 0;
g = [];

% Collision avoidance parameters
dsafe = 30.50;

% Set up the cost and constraints over the prediction horizon
for k = 1:N
    % Integrate dynamics using Euler method
    X(:,k+1) = X(:,k) + T * f_fun(X(:,k), U(:,k));
    
    % Add stage cost
    J = J + cost_fun(X(:,k), U(:,k), p);
    
    % Add collision avoidance constraint
    x1 = X(1:3, k);
    x2 = X(6:8, k);
    dist_squared = ((x1(1) - x2(1))^2) + ((x1(2) - x2(2))^2) - dsafe^2;
    g = [g; dist_squared];
end

% Add terminal cost
J = J + terminal_cost_fun(X(:,end), p);

% Vectorize the decision variables
w = [reshape(U, nu*N, 1); reshape(X, nx*(N+1), 1)];

% Define CasADi function for optimization
opts = struct('ipopt', struct('print_level', 0, 'tol', 1e-8));
nlp = struct('x', w, 'f', J, 'g', g, 'p', p);
solver = nlpsol('solver', 'ipopt', nlp, opts);

% Initial conditions and constraints
x0 = [110; -20; pi/2; 5; 0; 125; 0; pi; 5; 0];
lbx = -inf*ones(nu*N + nx*(N+1), 1);
ubx = inf*ones(nu*N + nx*(N+1), 1);

% Bounds on states and controls
lbu = repmat([-11.5; -1; -11.5; -1], N, 1);
ubu = repmat([11.5; 1; 11.5; 1], N, 1);

% Constraint on initial state
lbx(end-nx+1:end) = x0;
ubx(end-nx+1:end) = x0;

% Initial guess
x0_guess = repmat(x01, N+1, 1);
u0_guess = zeros(nu*N, 1);
w0 = [u0_guess; x0_guess];

% Load reference waypoints
waypoints = Way(); % Assuming you have the function Way() to generate waypoints

% Main MPC loop
x_traj = x0;
u_traj = [];

for i = 1:100  % Number of simulation steps
    % Update the reference state
    ref = waypoints(:, i);
    p_val = [ref; zeros(nx-length(ref), 1)]; % Ensure the reference state vector matches the dimensions

    % Solve the NLP
    sol = solver('x0', w0, 'lbx', lbx, 'ubx', ubx, 'p', p_val);
    w_opt = full(sol.x);

    % Extract control input
    u_opt = reshape(w_opt(1:nu*N), nu, N);
    u_traj(:, i) = u_opt(:,1);

    % Apply control input to the system
    x_next = full(f_fun(x_traj(:,i), u_traj(:,i)));
    x_traj(:, i+1) = x_next;

    % Shift horizon
    x0 = x_next;
    lbx(end-nx+1:end) = x0;
    ubx(end-nx+1:end) = x0;

    % Update initial guess
    x0_guess = [x0; repmat(x0, N, 1)];
    w0 = [u0_guess; x0_guess];
end

% Plot results (if needed)
