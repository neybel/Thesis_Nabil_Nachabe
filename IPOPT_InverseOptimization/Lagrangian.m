function model = Lagriangian()
import casadi.*

%% system dimensions
nx = 9; % I am considering these as optimizable parameters
nu = 0; %not needed I think 
np= 12; % non optimizable parameters
Wx1=SX.sym('Wx1');Wx2=SX.sym('Wx2');Wx3=SX.sym('Wx3');Wx4=SX.sym('Wx4');Wx5=SX.sym('Wx5');
W_x=diag([Wx1,Wx2,Wx3,Wx4,Wx5]);
Wu1=SX.sym('Wu1');Wu2=SX.sym('Wu2');
W_u=diag([Wu1,Wu2]);
Gamma1 = -5.0;
p = SX.sym('p',7);
p_ref=SX.sym('p_ref',5);
%%
state_cost = (p(1:5) - p_ref(1:5))' * W_x(1:5, 1:5) * (p(1:5) - p_ref(1:5));
input_cost = p(6:7)' * W_u(1:2, 1:2) * p(6:7);
expr_ext_cost = (1 / Gamma1) * log(exp(Gamma1 * (state_cost + input_cost)));
%jacobian(expr_ext_cost1,p)
%% Dynamics to append to the lagrangian
lambda1=SX.sym('lambda1',5);
% nx = 5;  % State dimension
% nu = 2;  % Input dimension
L = 2.73;  % Wheelbase [m]


% Symbolic variables
x1 = p(1);
y1 = p(2);   
theta1 = p(3); 
v1 = p(4);       % longitudinal velocity
delta1 = p(5);  % steering angle

% sym_x = [x1; y1; theta1; v1; delta1];  % State vector

% sym_u = SX.sym('u', nu, 1);  % Input vector
delta_dot1 = p(7);  % rate of steering angle change
a1 = p(6);  % acceleration

% Dynamics equations
dx1 = v1 * cos(theta1);
dy1 = v1 * sin(theta1);
dtheta1 = (v1 / L) * tan(delta1);
dv1 = a1;
ddelta1 = delta_dot1;

% Symbolic dynamics vector
f_k = [dx1; dy1; dtheta1; dv1; ddelta1];
dt=0.02;
p_next=SX.sym('p_next',5);
D = lambda1'*(p_next-p(1:5)-dt*f_k);

Lagrangian = expr_ext_cost + D;
%% Control bound to append
uba=11.5; ubs=1;
Lambda2=SX.sym('Lambda2',4);
Bound = Lambda2(1)*(p(6) + uba) + Lambda2(2)*(-p(6) + uba) +Lambda2(3)*(p(7) + ubs)  +Lambda2(4)*(-p(7) + ubs);
Lagrangian = Lagrangian + Bound;
%% 
sym_x=vertcat(Wx1,Wx2,Wx3,Wx4,Wx5,Wu1,Wu2,lambda1,Lambda2);
P=vertcat(p,p_ref,p_next);
%% Gradient 
G_Lx= jacobian(Lagrangian,p(1:5));
G_Lu=jacobian(Lagrangian,p(6:7));
% G_L=G_Lx+G_Lu;
G_L = vertcat(G_Lx', G_Lu');
% test=jacobian(Lagrangian,p)
G=dot(G_L,G_L);
   %% Dummy dynamics function (all zeros)
%    sym_xdot = SX.sym(11, 1);
   sym_xdot = SX.sym('xdot', nx, 1);
%    model.sym_xdot = sym_xdot; % Dummy dynamics
%    expr_f_expl = vertcat(dx1, dy1, dtheta1, dv1, ddelta1);
%    expr_f_expl = zeros(11,1);
   expr_f_expl = vertcat(Wx1,Wx2,Wx3,Wx4,Wx5,Wu1,Wu2,lambda1,Lambda2);
%    expr_f_impl = expr_f_expl - sym_xdot;
    model.expr_f_expl = expr_f_expl;
%     model.expr_f_impl = expr_f_impl;

%%
model.nx = nx;
model.nu = nu;
model.np = np;

model.sym_u = SX.sym('u', nu, 1); % Dummy control variable
model.sym_xdot = sym_xdot; % Dummy dynamics

model.sym_x = sym_x;
model.sym_p = P;
model.expr_ext_cost = G;
model.expr_ext_cost_e = G;
end


% Dynamics function f(x, u)
% f = Function('f', {sym_x, sym_u}, {sym_xdot});
%x_val = [0; 0; 0; 10; 0];  % Example state vector
%u_val = [0; 1];  % Example control input vector
% xdot_val = f(x_val, u_val);  % Evaluate dynamics at (x_val, u_val)
