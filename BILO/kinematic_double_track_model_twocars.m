function model = kinematic_double_track_model_twocars()

import casadi.*

%% system dimensions
nx = 6; % 3 states for each car (position x, position y, orientation theta)
nu = 4; % 2 control inputs for each car (longitudinal velocity, steering angle)

%% system parameters
L = 0.5;  % wheelbase [m]

%% named symbolic variables
x1 = SX.sym('x1');         % x position of car 1 [m]
y1 = SX.sym('y1');         % y position of car 1 [m]
theta1 = SX.sym('theta1'); % orientation of car 1 [rad]
v1 = SX.sym('v1');         % longitudinal velocity of car 1 [m/s]
delta1 = SX.sym('delta1'); % steering angle of car 1 [rad]

x2 = SX.sym('x2');         % x position of car 2 [m]
y2 = SX.sym('y2');         % y position of car 2 [m]
theta2 = SX.sym('theta2'); % orientation of car 2 [rad]
v2 = SX.sym('v2');         % longitudinal velocity of car 2 [m/s]
delta2 = SX.sym('delta2'); % steering angle of car 2 [rad]


%% (unnamed) symbolic variables
sym_x = vertcat(x1,y1,theta1,x2,y2,theta2);
sym_xdot = SX.sym('xdot', nx, 1);
sym_u = vertcat(v1,delta1,v2,delta2);
% sym_u = SX.sym('u', nu, 1); % Define control inputs as decision variables
%% dynamics for car 1
dx1 = v1 * cos(theta1);
dy1 = v1 * sin(theta1);
dtheta1 = (v1 / L) * tan(delta1);
% %% dynamics for car 1
% dx1 = sym_u(1) * cos(theta1);
% dy1 = sym_u(1) * sin(theta1);
% dtheta1 = (sym_u(1) / L) * tan(sym_u(2)); 

%% dynamics for car 2
dx2 = v2 * cos(theta2);
dy2 = v2 * sin(theta2);
dtheta2 = (v2 / L) * tan(delta2);
%% dynamics for car 2
% dx2 = sym_u(3) * cos(theta2);
% dy2 = sym_u(3) * sin(theta2);
% dtheta2 = (sym_u(3) / L) * tan(sym_u(4));
%%

expr_f_expl = vertcat(dx1,dy1,dtheta1,dx2,dy2,dtheta2);
expr_f_impl = expr_f_expl - sym_xdot;

%% constraints
expr_h = sym_u;

%% cost
%% populate structure
model.nx = nx;
model.nu = nu;
model.sym_x = sym_x;
model.sym_xdot = sym_xdot;
model.sym_u = sym_u;
model.expr_f_expl = expr_f_expl;
model.expr_f_impl = expr_f_impl;
model.expr_h = expr_h;

% Add other model fields as needed

end
