function model = newdyn()

import casadi.*

%% system dimensions
nx = 5; 
nu = 2; 
np = 1;

%% system parameters
L = 2.73;  % wheelbase [m]

%% named symbolic variables
x = SX.sym('x');          % x position [m]
y = SX.sym('y');          % y position [m]
theta = SX.sym('theta');  % orientation [rad]
v = SX.sym('v');          % longitudinal velocity [m/s]
delta = SX.sym('delta');  % steering angle [rad]
p = SX.sym('p',1);
%% (unnamed) symbolic variables
sym_x = vertcat(x, y, theta, v, delta)
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
P=[p;p1];

%% Constraints
expr_h = vertcat(sym_u(1), sym_u(2)); % Constraints on rate of steering angle and acceleration
% Nonlinear least squares
cost_expr_y = vertcat(sym_x, sym_u);
% Populate structure
model.nx = nx;
model.nu = nu;
model.np = np;
model.sym_p = P;
model.sym_x = sym_x;
model.sym_xdot = sym_xdot;
model.sym_u = sym_u;
model.expr_f_expl = expr_f_expl;
model.expr_h = expr_h;
model.cost_expr_y = cost_expr_y;
end
