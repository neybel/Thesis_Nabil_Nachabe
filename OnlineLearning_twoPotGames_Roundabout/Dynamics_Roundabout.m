%% New Dynamics one car
function model = Dynamics_Roundabout()

import casadi.*

%% system dimensions
nx = 10; % Including longitudinal velocity v and steering angle delta
nu = 4; % Including rate of steering angle delta_dot and acceleration a
np= 10;

%% system parameters
L = 2.73;  % wheelbase [m]

%% named symbolic variables
x1 = SX.sym('x1');       y1 = SX.sym('y1');        theta1 = SX.sym('theta1'); v1 = SX.sym('v1');  delta1 = SX.sym('delta1');  
x2 = SX.sym('x2');       y2 = SX.sym('y2');        theta2 = SX.sym('theta2'); v2 = SX.sym('v2');  delta2 = SX.sym('delta2');

%% (unnamed) symbolic variables
sym_x = vertcat(x1, y1, theta1, v1, delta1,x2,y2,theta2,v2,delta2);
sym_xdot = SX.sym('xdot', nx, 1);
sym_u = SX.sym('u', nu, 1);  % Input vector

%% Dynamics
delta_dot1 = sym_u(2);a1 = sym_u(1); 
delta_dot2 = sym_u(4); a2 = sym_u(3);  
 

dx1 = v1 * cos(theta1);dy1 = v1 * sin(theta1);dtheta1 = (v1 / L) * tan(delta1);dv1 = a1;ddelta1 = delta_dot1;
dx2 = v2 * cos(theta2);dy2 = v2 * sin(theta2);dtheta2 = (v2 / L) * tan(delta2);dv2 = a2;ddelta2 = delta_dot2;

%% External cost formulation
% W_x1 = [1e1,1e1,1e-4,1e-4,1e-4]; %Car from down to up 
% % W_x2 = [1e1,1e1,1e-4,1e-4,1e-4]; % Car from up to down
% W_x2 = [1e1,1e1,1e-4,1e-4,1e-4]; % Car from left to right
% % W_x4 = [1e1,1e1,1e-4,1e-4,1e-4]; % Car from right to left
% W_x = diag([W_x1,W_x2]);
% 
% % W_u = diag([1e-4,1e-2,1e-4,1e-2]);
% W_u = diag([1,1,1,1]);


p = SX.sym('p',10);
p1=SX.sym('p',14);
W_x = diag([p1(1:10)]);
W_u=diag([p1(11:14)]);
W_xe1=[1e-1,1e-1,1e-5,1e-4,1e-5];
% W_xe2=[1e-1,1e-1,1e-5,1e-4,1e-5];
W_xe2=[1e-1,1e-1,1e-5,1e-4,1e-5];
% W_xe4=[1e-1,1e-1,1e-5,1e-4,1e-5];

W_xe= diag([W_xe1,W_xe2]);
P=[p;p1];

%% Coupling
% % Alternative 1: Weighted Euclidean Distance (here same weight for both)
c = 1.0; %before it was 0.95
c13 = 1/(c*((x1 - x2)^2 + (y1 - y2)^2));
coupling_term = c13;

%% Cost expressions
% state_cost = (sym_x - p)' * W_x * (sym_x - p);
% input_cost = sym_u' * W_u * sym_u;
% terminal_cost = (sym_x - p)' * W_xe * (sym_x - p);
state_cost = (sym_x - p)' * W_x * (sym_x - p);
input_cost = sym_u' * W_u * sym_u;
terminal_cost = (sym_x - p)' * W_xe * (sym_x - p);

expr_ext_cost = state_cost + input_cost + coupling_term;

expr_ext_cost_e = terminal_cost;

%%
% state_cost = state_cost + coupling_term;
% expr_ext_cost = state_cost + input_cost;
% expr_ext_cost_e = terminal_cost; 
%%
expr_f_expl = vertcat(dx1, dy1, dtheta1, dv1, ddelta1,dx2, dy2, dtheta2, dv2, ddelta2);
expr_f_impl = expr_f_expl - sym_xdot;

% Nonlinear least squares
cost_expr_y = vertcat(sym_x, sym_u);

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
