%% New Dynamics one car
function model = DYNFourcarExternalCoupling_3_test_withrandom()

import casadi.*

%% system dimensions
nx = 20; % Including longitudinal velocity v and steering angle delta
nu = 8; % Including rate of steering angle delta_dot and acceleration a
np= 20;

%% system parameters
L = 2.73;  % wheelbase [m]

%% named symbolic variables
x1 = SX.sym('x1');       y1 = SX.sym('y1');        theta1 = SX.sym('theta1'); v1 = SX.sym('v1');  delta1 = SX.sym('delta1');  
x2 = SX.sym('x2');       y2 = SX.sym('y2');        theta2 = SX.sym('theta2'); v2 = SX.sym('v2');  delta2 = SX.sym('delta2');
x3 = SX.sym('x3');       y3 = SX.sym('y3');        theta3 = SX.sym('theta3'); v3 = SX.sym('v3');  delta3 = SX.sym('delta3');  
x4 = SX.sym('x4');       y4 = SX.sym('y4');        theta4 = SX.sym('theta4'); v4 = SX.sym('v4');  delta4 = SX.sym('delta4');  


%% (unnamed) symbolic variables
sym_x = vertcat(x1, y1, theta1, v1, delta1,x2,y2,theta2,v2,delta2,x3,y3,theta3,v3,delta3,x4,y4,theta4,v4,delta4);
sym_xdot = SX.sym('xdot', nx, 1);
sym_u = SX.sym('u', nu, 1);  % Input vector

%% Dynamics
delta_dot1 = sym_u(2);a1 = sym_u(1); 
delta_dot2 = sym_u(4); a2 = sym_u(3);  
delta_dot3 = sym_u(6); a3 = sym_u(5);  
delta_dot4 = sym_u(8); a4 = sym_u(7);  

dx1 = v1 * cos(theta1);dy1 = v1 * sin(theta1);dtheta1 = (v1 / L) * tan(delta1);dv1 = a1;ddelta1 = delta_dot1;
dx2 = v2 * cos(theta2);dy2 = v2 * sin(theta2);dtheta2 = (v2 / L) * tan(delta2);dv2 = a2;ddelta2 = delta_dot2;
dx3 = v3 * cos(theta3);dy3 = v3 * sin(theta3);dtheta3 = (v3/ L) * tan(delta3);dv3 = a3;ddelta3 = delta_dot3;
dx4 = v4 * cos(theta4);dy4 = v4 * sin(theta4);dtheta4 = (v4 / L) * tan(delta4);dv4 = a4;ddelta4 = delta_dot4;

%% External cost formulation
Gamma1 = -5.0; Gamma2 = -5.0; 
Gamma3 = +5.0; Gamma4 = + 5.0;
% W_x1 = [1e1,1e2,1e-2,1e0,1e-15]; %Car from down to up 
% W_x2 = [1e3,1e1,1e-2,1e1,1e-10]; % Car from up to down
% W_x3 = [1e1,1e2,1e-2,1e0,1e-15]; % Car from left to right
% W_x4 = [1e3,1e1,1e-2,1e0,1e-10]; % Car from right to left

W_x1 = [0.05,1e-3,1e-4,1e-2,1e-4]; %Car from down to up 
W_x2 = [1e1,1e1,1e-4,1e-2,1e-4]; % Car from up to down
W_x3 = [1e1,1e1,1e-4,1e-2,1e-4]; % Car from left to right
W_x4 = [1e1,1e1,1e-4,1e-2,1e-4]; % Car from right to left
W_x = diag([W_x1,W_x2,W_x3,W_x4]);


% W_u = diag([1e0,0.50,1e0,0.50,1e-1,1e0,1e-1,1e0]);
% W_u = diag([1e-1,1e-3,1e0,1e-3,1e-1,1e0,1e-1,1e0]);


% W_u = diag([1e-3,1e-5,1e-1,1e-7,1e-3,1e-7,1e-1,1e-5]);
W_u = diag([1e-4,5,1e-2,0.5,1e-2,0.5,1e-2,0.5]);


W_xe1=[1e-1,1e-1,1e-5,1e-3,1e-5];
W_xe2=[1e-1,1e-1,1e-5,1e-3,1e-5];
W_xe3=[1e-1,1e-1,1e-5,1e-3,1e-5];
W_xe4=[1e-1,1e-1,1e-5,1e-3,1e-5];

% W_xe1=[1e1,1e2,1e-2,1e1,1e-10];
% W_xe2=[1e1,1e2,1e-2,1e1,1e-10];
% W_xe3=[1e1,1e2,1e-2,1e1,1e-10];
% W_xe4=[1e1,1e2,1e-2,1e1,1e-10];
W_xe= diag([W_xe1,W_xe2,W_xe3,W_xe4]);

p = SX.sym('p',20);

%% Coupling
% % Alternative 1: Weighted Euclidean Distance (here same weight for both)
c = 1.0; %before it was 0.95
c12 = 1/(c*((x1 - x2)^2 + (y1 - y2)^2));
c13 = 1/(c*((x1 - x3)^2 + (y1 - y3)^2));
c14 = 1/(c*((x1 - x4)^2 + (y1 - y4)^2));
c23 = 1/(c*((x2 - x3)^2 + (y2 - y3)^2));
c24 = 1/(c*((x2 - x4)^2 + (y2 - y4)^2));
c34 = 1/(c*((x3 - x4)^2 + (y3 - y4)^2));

coupling_term = c12+c13+c14+c23+c24+c34;

%% Cost expressions
% state_cost = (sym_x - p)' * W_x * (sym_x - p);
% input_cost = sym_u' * W_u * sym_u;
% terminal_cost = (sym_x - p)' * W_xe * (sym_x - p);
state_cost1 = (sym_x(1:5) - p(1:5))' * W_x(1:5, 1:5) * (sym_x(1:5) - p(1:5));
input_cost1 = sym_u(1:2)' * W_u(1:2, 1:2) * sym_u(1:2);
state_cost2 = (sym_x(6:10) - p(6:10))' * W_x(6:10, 6:10) * (sym_x(6:10) - p(6:10));
input_cost2 = sym_u(3:4)' * W_u(3:4, 3:4) * sym_u(3:4);
state_cost3 = (sym_x(11:15) - p(11:15))' * W_x(11:15, 11:15) * (sym_x(11:15) - p(11:15));
input_cost3 = sym_u(5:6)' * W_u(5:6, 5:6) * sym_u(5:6);
state_cost4 = (sym_x(16:20) - p(16:20))' * W_x(16:20, 16:20) * (sym_x(16:20) - p(16:20));
input_cost4 = sym_u(7:8)' * W_u(7:8, 7:8) * sym_u(7:8);

terminal_cost1 = (sym_x(1:5) - p(1:5))' * W_xe(1:5, 1:5) * (sym_x(1:5) - p(1:5));
terminal_cost2 = (sym_x(6:10) - p(6:10))' * W_xe(6:10, 6:10) * (sym_x(6:10) - p(6:10));
terminal_cost3 = (sym_x(11:15) - p(11:15))' * W_xe(11:15, 11:15) * (sym_x(11:15) - p(11:15));
terminal_cost4 = (sym_x(16:20) - p(16:20))' * W_xe(16:20, 16:20) * (sym_x(16:20) - p(16:20));

expr_ext_cost1 = (1 / Gamma1) * log(exp(Gamma1 * (state_cost1 + input_cost1)));
expr_ext_cost2 = (1 / Gamma2) * log(exp(Gamma2 * (state_cost2 + input_cost2)));
expr_ext_cost3 = (1 / Gamma3) * log(exp(Gamma3 * (state_cost3 + input_cost3)));
expr_ext_cost4 = (1 / Gamma4) * log(exp(Gamma4 * (state_cost4 + input_cost4)));
expr_ext_cost = expr_ext_cost1 + expr_ext_cost2 +expr_ext_cost3+ expr_ext_cost4 +coupling_term;

expr_ext_cost_e1 = (1 / Gamma1) * log(exp(Gamma1 * terminal_cost1)) ; % before i added here for the 4 rand/10 to not be in a frozen robot situation
expr_ext_cost_e2 = (1 / Gamma2) * log(exp(Gamma2 * terminal_cost2)) ;
expr_ext_cost_e3 = (1 / Gamma3) * log(exp(Gamma3 * terminal_cost3)) ;
expr_ext_cost_e4 = (1 / Gamma4) * log(exp(Gamma4 * terminal_cost4)) ;
expr_ext_cost_e = expr_ext_cost_e1 + expr_ext_cost_e2 +expr_ext_cost_e4 +expr_ext_cost_e3;

%%
% state_cost = state_cost + coupling_term;
% expr_ext_cost = state_cost + input_cost;
% expr_ext_cost_e = terminal_cost; 
%%
expr_f_expl = vertcat(dx1, dy1, dtheta1, dv1, ddelta1,dx2, dy2, dtheta2, dv2, ddelta2,dx3, dy3, dtheta3, dv3, ddelta3,dx4, dy4, dtheta4, dv4, ddelta4);
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
model.sym_p = p;
end
