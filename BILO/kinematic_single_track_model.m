
function model = kinematic_single_track_model()

import casadi.*

%% system dimensions
nx = 3;
nu = 2;

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
%% cost
W_x = diag([1e4, 1e-2, 1e-2]);
W_u = diag([1e-1;1e-1]);
W_xe = diag([1e4,1,1]); 
yr_e = [10;0;0];
expr_ext_cost_e = (sym_x - yr_e)'* W_xe * (sym_x - yr_e);
expr_ext_cost = (sym_u' * W_u * sym_u) + (sym_x' * W_x * sym_x);
% nonlinear least sqares
cost_expr_y = vertcat(sym_x, sym_u);
% W = blkdiag(W_x, W_u);
model.cost_expr_y_e = sym_x;
% model.W_e = W_x;

%% populate structure
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
% model.W = W;
% Define initial and final conditions
% model.x0 = [0; 0; pi/4];
% model.constr_x0 = model.x0;

% Introduce input constraints
% model.lbu = lbu;
% model.ubu = ubu;
% model.W_u = W_u;
% Define constraints for the final state
% x_final = [10;0; 0];
% model.constr_x0_e = x_final;
end
