%% New Dynamics one car
function model = DYNforNE_Check()

import casadi.*

%% system dimensions
nx = 5; % Including longitudinal velocity v and steering angle delta
nu = 2; % Including rate of steering angle delta_dot and acceleration a
np= 5+4;

%% system parameters
L = 2.73;  % wheelbase [m]

%% named symbolic variables
x1 = SX.sym('x1');          % x position [m]
y1 = SX.sym('y1');          % y position [m]
theta1 = SX.sym('theta1');  % orientation [rad]
v1 = SX.sym('v1');          % longitudinal velocity [m/s]
delta1 = SX.sym('delta1');  % steering angle [rad]
p1 = SX.sym('p1',4);
%% (unnamed) symbolic variables
sym_x = vertcat(x1, y1, theta1, v1, delta1);
sym_xdot = SX.sym('xdot', nx, 1);
sym_u = SX.sym('u', nu, 1);  % Input vector

%% Dynamics
delta_dot1 = sym_u(2); % Rate of change of steering angle
a1 = sym_u(1);         % Acceleration
% delta_dot2 = sym_u(4); % Rate of change of steering angle
% a2 = sym_u(3);  

dx1 = v1 * cos(theta1);
dy1 = v1 * sin(theta1);
dtheta1 = (v1 / L) * tan(delta1);
dv1 = a1;
ddelta1 = delta_dot1;

%% External cost formulation
% W_x = diag([1e3,1e3,1e-2,1e-2,1e-5,1e3,1e1,1e-2,1e-2,1e-10]);
W_x = diag([1e3,1e1,1e-2,1e-2,1e-10]);
% W_u=diag([7.5,1e-1,1e-3,1e-1]);
W_u=diag([1e-3,1e-1]);
% W_xe= diag([1e5,1e5,1e-2,1e2,1e-2,1e1,1e1,1e-5,1e-1,1e-5]);
W_xe= diag([1e2,1e1,1e-5,1e-1,1e-5]);
p = SX.sym('p',5);

%% Cost expressions
state_cost = (sym_x - p)' * W_x * (sym_x - p);
input_cost = sym_u' * W_u * sym_u;
terminal_cost = (sym_x - p)' * W_xe * (sym_x - p);
% 
% expr_ext_cost = (1 / Gamma1) * log(exp(Gamma1 * (state_cost + input_cost)));
% expr_ext_cost_e = (1 / Gamma1) * log(exp(Gamma1 * terminal_cost));
%% Coupling
% % Alternative 1: Weighted Euclidean Distance (here same weight for both)
c=0.95;
coupling_term = 0;
%%

state_cost = state_cost + coupling_term;
expr_ext_cost = state_cost + input_cost;
expr_ext_cost_e = terminal_cost; % Terminal cost remains unchanged
%%

expr_f_expl = vertcat(dx1, dy1, dtheta1, dv1, ddelta1);
expr_f_impl = expr_f_expl - sym_xdot;

% Nonlinear least squares
cost_expr_y = vertcat(sym_x, sym_u);
P=[p;p1];

% Populate structure
model.nx = nx;
model.nu = nu;
model.np = np;
model.sym_x = sym_x;
model.sym_xdot = sym_xdot;
model.sym_u = sym_u;
model.expr_f_expl = expr_f_expl;
model.expr_f_impl = expr_f_impl;
model.cost_expr_y = cost_expr_y;
model.expr_ext_cost = expr_ext_cost;
model.expr_ext_cost_e = expr_ext_cost_e;
model.sym_p = P;
end
