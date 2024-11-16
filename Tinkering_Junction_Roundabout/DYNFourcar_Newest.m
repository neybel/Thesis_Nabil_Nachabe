function model = DYNFourcar_Newest()
import casadi.*
%% Define system dimensions
nx = 20; % Including longitudinal velocity v and steering angle delta
nu = 8;  % Including rate of steering angle delta_dot and acceleration a
np = 20;

%% Define symbolic variables for states, controls, and parameters
sym_x = SX.sym('sym_x', nx, 1);
sym_xdot = SX.sym('sym_xdot', nx, 1);
sym_u = SX.sym('sym_u', nu, 1);
p = SX.sym('p', np, 1);  % Reference state

%% Define cost function weights
W_x1 = [1e-1,1e-1,1e-4,1e-2,1e-4]; %Car from down to up 
W_x2 = [1e-1,1e-1,1e-4,1e-2,1e-4]; % Car from up to down
W_x3 = [1e-1,1e-1,1e-4,1e-2,1e-4]; % Car from left to right
W_x4 = [1e-1,1e-1,1e-4,1e-2,1e-4]; % Car from right to left
W_x = diag([W_x1,W_x2,W_x3,W_x4]);

W_u = diag([1e-2,1e-2,1e-2,1e-2,1e-2,1e-2,1e-2,1e-2]);

W_xe1=[1e-1,1e-1,1e-5,1e-3,1e-5];
W_xe2=[1e-1,1e-1,1e-5,1e-3,1e-5];
W_xe3=[1e-1,1e-1,1e-5,1e-3,1e-5];
W_xe4=[1e-1,1e-1,1e-5,1e-3,1e-5];
W_xe= diag([W_xe1,W_xe2,W_xe3,W_xe4]);

%% Parameters for entropic risk measure
Gamma1 = 1; Gamma2 = 1; 
Gamma3 = 1; Gamma4 = 1;
Gamma = [Gamma1, Gamma2, Gamma3, Gamma4];
num_samples = 10;  % Number of Monte Carlo samples

%% Define stochastic cost expressions
state_cost = 0;
input_cost = 0;

for i = 1:4
    idx = (i-1)*5 + (1:5);
    state_cost_i = (sym_x(idx) - p(idx))' * W_x(idx, idx) * (sym_x(idx) - p(idx));
    input_cost_i = sym_u(2*i-1:2*i)' * W_u(2*i-1:2*i, 2*i-1:2*i) * sym_u(2*i-1:2*i);

    % Convert symbolic expressions to functions
    f_state_cost_i = Function('f_state_cost_i', {sym_x, p}, {state_cost_i});
    f_input_cost_i = Function('f_input_cost_i', {sym_u}, {input_cost_i});
    
    % Evaluate costs numerically for debugging
    state_cost_i_num = full(f_state_cost_i(rand(nx,1), rand(np,1))); % Random inputs for testing
    input_cost_i_num = full(f_input_cost_i(rand(nu,1))); % Random inputs for testing
    
    fprintf('State Cost %d: %f\n', i, state_cost_i_num);
    fprintf('Input Cost %d: %f\n', i, input_cost_i_num)
    
    % Initialize the Monte Carlo approximation for the expected value
    mc_samples = SX.zeros(num_samples, 1);
    for j = 1:num_samples
        % Add randomness to the state and input cost
        state_cost_sample = state_cost_i + normrnd(0, 0.001);
        input_cost_sample = input_cost_i + normrnd(0, 0.001);
        mc_samples(j) = exp(Gamma(i) * (state_cost_sample + input_cost_sample));
%                 fprintf('MC Sample %d: %f\n', j, full(mc_samples(j)));

    end
    epsilon = 1e-6;  % Small epsilon to avoid log(0)
    % Compute the expected value
    expected_value = sum(mc_samples) / num_samples + epsilon;
%         fprintf('Expected Value: %f\n', full(expected_value));

    
    % Add to the total cost
    state_cost = state_cost + (1 / Gamma(i)) * log10(expected_value);
end

%% Coupling Term (unchanged)
c = 1.0;
c12 = 1/(c*((sym_x(1) - sym_x(6))^2 + (sym_x(2) - sym_x(7))^2));
c13 = 1/(c*((sym_x(1) - sym_x(11))^2 + (sym_x(2) - sym_x(12))^2));
c14 = 1/(c*((sym_x(1) - sym_x(16))^2 + (sym_x(2) - sym_x(17))^2));
c23 = 1/(c*((sym_x(6) - sym_x(11))^2 + (sym_x(7) - sym_x(12))^2));
c24 = 1/(c*((sym_x(6) - sym_x(16))^2 + (sym_x(7) - sym_x(17))^2));
c34 = 1/(c*((sym_x(11) - sym_x(16))^2 + (sym_x(12) - sym_x(17))^2));
% coupling_term = c12 + c13 + c14 + c23 + c24 + c34;
coupling_term=0;
% fprintf('Coupling Term: %f\n', full(coupling_term));


%% Total cost function
regularization_term = 1e-6;  % Small regularization term

expr_ext_cost = state_cost + coupling_term;
expr_ext_cost = expr_ext_cost + regularization_term * norm(sym_u);


%% Terminal cost (unchanged)
terminal_cost1 = (sym_x(1:5) - p(1:5))' * W_xe(1:5, 1:5) * (sym_x(1:5) - p(1:5));
terminal_cost2 = (sym_x(6:10) - p(6:10))' * W_xe(6:10, 6:10) * (sym_x(6:10) - p(6:10));
terminal_cost3 = (sym_x(11:15) - p(11:15))' * W_xe(11:15, 11:15) * (sym_x(11:15) - p(11:15));
terminal_cost4 = (sym_x(16:20) - p(16:20))' * W_xe(16:20, 16:20) * (sym_x(16:20) - p(16:20));
expr_ext_cost_e = terminal_cost1 + terminal_cost2 + terminal_cost3 + terminal_cost4;

%% Define Dynamics (unchanged)
L = 2.73;  % wheelbase [m]

dx1 = sym_x(4) * cos(sym_x(3));
dy1 = sym_x(4) * sin(sym_x(3));
dtheta1 = (sym_x(4) / L) * tan(sym_x(5));
dv1 = sym_u(1);
ddelta1 = sym_u(2);

dx2 = sym_x(9) * cos(sym_x(8));
dy2 = sym_x(9) * sin(sym_x(8));
dtheta2 = (sym_x(9) / L) * tan(sym_x(10));
dv2 = sym_u(3);
ddelta2 = sym_u(4);

dx3 = sym_x(14) * cos(sym_x(13));
dy3 = sym_x(14) * sin(sym_x(13));
dtheta3 = (sym_x(14) / L) * tan(sym_x(15));
dv3 = sym_u(5);
ddelta3 = sym_u(6);

dx4 = sym_x(19) * cos(sym_x(18));
dy4 = sym_x(19) * sin(sym_x(18));
dtheta4 = (sym_x(19) / L) * tan(sym_x(20));
dv4 = sym_u(7);
ddelta4 = sym_u(8);

expr_f_expl = vertcat(dx1, dy1, dtheta1, dv1, ddelta1, dx2, dy2, dtheta2, dv2, ddelta2, dx3, dy3, dtheta3, dv3, ddelta3, dx4, dy4, dtheta4, dv4, ddelta4);
expr_f_impl = expr_f_expl - sym_xdot;

%% Nonlinear least squares
cost_expr_y = vertcat(sym_x, sym_u);

%% Populate structure
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
