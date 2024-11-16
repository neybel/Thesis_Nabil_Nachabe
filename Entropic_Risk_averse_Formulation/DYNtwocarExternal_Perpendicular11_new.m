%% New Dynamics one car
function model = DYNtwocarExternal_Perpendicular11()

import casadi.*

%% system dimensions
nx = 10; % Including longitudinal velocity v and steering angle delta
nu = 4; % Including rate of steering angle delta_dot and acceleration a
np= 10;

%% system parameters
L = 2.73;  % wheelbase [m]

%% named symbolic variables
x1 = SX.sym('x1');          % x position [m]
y1 = SX.sym('y1');          % y position [m]
theta1 = SX.sym('theta1');  % orientation [rad]
v1 = SX.sym('v1');          % longitudinal velocity [m/s]
delta1 = SX.sym('delta1');  % steering angle [rad]
x2 = SX.sym('x2');          % x position [m]
y2 = SX.sym('y2');          % y position [m]
theta2 = SX.sym('theta2');  % orientation [rad]
v2 = SX.sym('v2');          % longitudinal velocity [m/s]
delta2 = SX.sym('delta2');  % steering angle [rad]

%% (unnamed) symbolic variables
sym_x = vertcat(x1, y1, theta1, v1, delta1,x2,y2,theta2,v2,delta2);
sym_xdot = SX.sym('xdot', nx, 1);
sym_u = SX.sym('u', nu, 1);  % Input vector

%% Dynamics
delta_dot1 = sym_u(2); % Rate of change of steering angle
a1 = sym_u(1);         % Acceleration
delta_dot2 = sym_u(4); % Rate of change of steering angle
a2 = sym_u(3);  

dx1 = v1 * cos(theta1);
dy1 = v1 * sin(theta1);
dtheta1 = (v1 / L) * tan(delta1);
dv1 = a1;
ddelta1 = delta_dot1;

dx2 = v2 * cos(theta2);
dy2 = v2 * sin(theta2);
dtheta2 = (v2 / L) * tan(delta2);
dv2 = a2;
ddelta2 = delta_dot2;
%% External cost formulation
% Gamma1 = -1;  % Adjust this parameter based on your risk preference
% Gamma2 = +1;  % Adjust this parameter based on your risk preference
Gamma1 = 1;  % Adjust this parameter based on your risk preference
Gamma2 = +0.1;  % Adjust this parameter based on your risk preference
%%%% My observation for gamma lower but still + we get a more risk seeking
%%%% behavior, i.e if gamma2 is lower than gamma1, we get that car 2 is
%%%% more aggressive. (This is not working with the ngative gammas though!)
%%% NB: switch gamma1 equal 0.1 and gamma 2 equal 1 in order to see the
%%% opposite behavior and focus on the distance travelled and time of
%%% travel as well!!
Gamma = [Gamma1, Gamma2];
num_samples = 10;  % Number of Monte Carlo samples

W_x = diag([1e1,1e2,1e-2,1e0,1e-5,1e2,1e1,1e-2,1e0,1e-5]);
W_u=diag([1e-3,1e-1,1e-3,1e-1]);
W_xe= diag([1e1,1e1,1e-2,1e-3,1e-2,1e1,1e1,1e-2,1e-3,1e-2]);
p = SX.sym('p',10);

%% Define stochastic cost expressions
state_cost = 0;
input_cost = 0;

for i = 1:2
    idx = (i-1)*5 + (1:5);
%     noise1=normrnd(0, 1e-3);%noise on x and y
%     noise2=normrnd(0, 1e-5);% noise on the remaining states
%     noise=[noise1;noise1;noise2;noise2;noise2];
%     state_cost_i = ((sym_x(idx)+noise) - p(idx))' * W_x(idx, idx) * ((sym_x(idx)+noise) - p(idx));
%     input_cost_i = (sym_u(2*i-1:2*i))' * W_u(2*i-1:2*i, 2*i-1:2*i) * (sym_u(2*i-1:2*i));
    
    % Initialize the Monte Carlo approximation for the expected value
    mc_samples = SX.zeros(num_samples, 1);
    for j = 1:num_samples
        % Add noise to the state
        noise1=normrnd(0, 1e-1);%noise on x and y
        noise2=normrnd(0, 1e-3);% noise on the remaining states
%         noise=[noise1;noise1;noise2;noise2;noise2];
        noise=[noise1;noise2];
%         state_cost_i = ((sym_x(idx)+noise) - p(idx))' * W_x(idx, idx) * ((sym_x(idx)+noise) - p(idx));
        state_cost_i = ((sym_x(idx)) - p(idx))' * W_x(idx, idx) * ((sym_x(idx)) - p(idx));
        input_cost_i = (sym_u(2*i-1:2*i)+noise)' * W_u(2*i-1:2*i, 2*i-1:2*i) * (sym_u(2*i-1:2*i)+noise);
        state_cost_sample = state_cost_i;
        input_cost_sample = input_cost_i;
        mc_samples(j) = exp(Gamma(i) * (state_cost_sample + input_cost_sample));
    end
    epsilon = 1e-6;  % Small epsilon to avoid log(0)
    expected_value = sum(mc_samples) / num_samples + epsilon;
    if any(Gamma)
    state_cost = state_cost + (1 / Gamma(i)) * log(expected_value);
    else
 state_cost = state_cost + log(expected_value);
    end
end
coupling_term=0;

%% Total cost function
% regularization_term = 1e-6;  % Small regularization term
regularization_term=0;
expr_ext_cost = state_cost + coupling_term;
expr_ext_cost = expr_ext_cost + regularization_term * norm(sym_u);


%% Terminal cost (unchanged)
terminal_cost1 = (sym_x(1:5) - p(1:5))' * W_xe(1:5, 1:5) * (sym_x(1:5) - p(1:5));
terminal_cost2 = (sym_x(6:10) - p(6:10))' * W_xe(6:10, 6:10) * (sym_x(6:10) - p(6:10));
expr_ext_cost_e = terminal_cost1 + terminal_cost2;

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
model.sym_p = p;
end
