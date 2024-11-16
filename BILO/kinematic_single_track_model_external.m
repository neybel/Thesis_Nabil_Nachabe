function model = kinematic_single_track_model()

import casadi.*

%% system dimensions
nx = 3;
nu = 2;
np = 3;

%% system parameters
L = 0.05;  % wheelbase [m]

%% named symbolic variables
x = SX.sym('x');          % x position [m]
y = SX.sym('y');          % y position [m]
theta = SX.sym('theta');  % orientation [rad]
v = SX.sym('v');          % longitudinal velocity [m/s]
delta = SX.sym('delta');  % steering angle [rad]

%% (unnamed) symbolic variables
sym_x = vertcat(x, y, theta);
sym_xdot = SX.sym('xdot', nx, 1);
sym_u = vertcat(v, delta);

%% dynamics
dx = v * cos(theta);
dy = v * sin(theta);
dtheta = (v / L) * tan(delta);

expr_f_expl = vertcat(dx, dy, dtheta);
expr_f_impl = expr_f_expl - sym_xdot;

%% constraints
expr_h = sym_u;
% Input constraints (you can adjust these based on the system limits)
lbu = [0; -pi];
ubu = [5; +pi];

%% cost
W_x = diag([1e4, 1e2, 1e2]);
W_u = diag([1e-1;1e-1]);
W_xe = diag([1e4,10,10]); 

% Reference state
% x_ref = SX.sym('x_ref', nx, 1);
p = SX.sym('p',3);


% Cost expressions
expr_ext_cost_e = (sym_x - p)' * W_xe * (sym_x - p);
expr_ext_cost = (sym_u' * W_u * sym_u) + ((sym_x - p)' * W_x * (sym_x - p));

% nonlinear least squares
cost_expr_y = vertcat(sym_x, sym_u);

%% populate structure
model.nx = nx;
model.nu = nu;
model.np = np;
model.sym_x = sym_x;
model.sym_xdot = sym_xdot;
model.sym_u = sym_u;
model.sym_p = p;
model.expr_f_expl = expr_f_expl;
model.expr_f_impl = expr_f_impl;
model.expr_h = expr_h;
model.expr_ext_cost = expr_ext_cost;
model.expr_ext_cost_e = expr_ext_cost_e;
model.cost_expr_y = cost_expr_y;

% Store reference state for later use
% model.x_ref = x_ref;

end
