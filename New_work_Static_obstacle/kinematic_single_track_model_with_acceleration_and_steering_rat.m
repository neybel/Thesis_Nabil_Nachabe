%% New Dynamics one car
function model = kinematic_single_track_model_with_acceleration_and_steering_rate()

import casadi.*

%% system dimensions
nx = 5; % Including longitudinal velocity v and steering angle delta
nu = 2; % Including rate of steering angle delta_dot and acceleration a

%% system parameters
L = 2.73;  % wheelbase [m]

%% named symbolic variables
x = SX.sym('x');          % x position [m]
y = SX.sym('y');          % y position [m]
theta = SX.sym('theta');  % orientation [rad]
v = SX.sym('v');          % longitudinal velocity [m/s]
delta = SX.sym('delta');  % steering angle [rad]

%% (unnamed) symbolic variables
sym_x = vertcat(x, y, theta, v, delta);
sym_xdot = SX.sym('xdot', nx, 1);
sym_u = SX.sym('u', nu, 1);  % Input vector

%% Dynamics
delta_dot = sym_u(2); % Rate of change of steering angle
a = sym_u(1);         % Acceleration

dx = v * cos(theta);
dy = v * sin(theta);
dtheta = (v / L) * tan(delta);
dv = a;
ddelta = delta_dot;

expr_f_expl = vertcat(dx, dy, dtheta, dv, ddelta);
expr_f_impl = expr_f_expl - sym_xdot;

%% Constraints
expr_h = vertcat(sym_u(1), sym_u(2)); % Constraints on rate of steering angle and acceleration
% Input constraints
lbu = [-5; -pi]; % Adjusted input lower bounds
ubu = [5; pi];   % Adjusted input upper bounds

%% Cost
W_x = diag([1e4, 1e-2, 1e-2, 1e2, 1]); % Adjusted weights
W_u = diag([1e-1; 1e-1]);               % Adjusted weights
W_xe = diag([1e4, 1, 1, 1e2, 1]);       % Adjusted weights
yr_e = [10; 0; 0; 0; 0];                % Adjusted terminal reference
expr_ext_cost_e = (sym_x - yr_e)' * W_xe * (sym_x - yr_e);
expr_ext_cost = (sym_u' * W_u * sym_u) + (sym_x' * W_x * sym_x);

% Nonlinear least squares
cost_expr_y = vertcat(sym_x, sym_u);

% Populate structure
model.nx = nx;
model.nu = nu;
model.sym_x = sym_x;
model.sym_xdot = sym_xdot;
model.sym_u = sym_u;
model.expr_f_expl = expr_f_expl;
model.expr_f_impl = expr_f_impl;
model.expr_h = expr_h;
model.expr_ext_cost = expr_ext_cost;
model.expr_ext_cost_e = expr_ext_cost_e;
model.cost_expr_y = cost_expr_y;

% Input constraints
model.lbu = lbu;
model.ubu = ubu;

end
