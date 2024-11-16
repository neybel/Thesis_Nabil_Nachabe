import casadi.*
horizon_length = 100; % Shooting nodes

% Define the optimization variables
Wx1 = SX.sym('Wx1');
Wx2 = SX.sym('Wx2');
Wx3 = SX.sym('Wx3');
Wx4 = SX.sym('Wx4');
Wx5 = SX.sym('Wx5');
Wu1 = SX.sym('Wu1');
Wu2 = SX.sym('Wu2');
Lambda2 = SX.sym('Lambda2', 4);
lambda1 = SX.sym('lambda1', 5);
% Optimization variables
W = [Wx1; Wx2; Wx3; Wx4; Wx5; Wu1; Wu2; Lambda2; lambda1];

% Non optimizable variables
p = SX.sym('p', 7);
p_ref = SX.sym('p_ref', 5 * horizon_length); % Adjusted for horizon length
p_next = SX.sym('p_next', 5);

%% Normalization factors (e.g., standard deviations or ranges)
state_norm_factors = [10; 10; 1; 1; 1]; % Example normalization factors for states
input_norm_factors = [1; 1]; % Example normalization factors for inputs
%%
W_x = diag([Wx1, Wx2, Wx3, Wx4, Wx5]);
W_u = diag([Wu1, Wu2]);

% Define the objective function components
Gamma1 = -5.0;
state_cost = (p(1:5) - p_ref(1:5))' * W_x(1:5, 1:5) * (p(1:5) - p_ref(1:5));
input_cost = p(6:7)' * W_u(1:2, 1:2) * p(6:7);
expr_ext_cost = (1 / Gamma1) * log(exp(Gamma1 * (state_cost + input_cost)));

% Define the dynamics and constraints
L = 2.73;
dx1 = p(4) * cos(p(3));
dy1 = p(4) * sin(p(3));
dtheta1 = (p(4) / L) * tan(p(5));
dv1 = p(6);
ddelta1 = p(7);
f_k = [dx1; dy1; dtheta1; dv1; ddelta1];

dt = 0.02;
D = lambda1' * (p_next - p(1:5) - dt * f_k);

% Control bounds
uba = 11.5;
ubs = 1;
Bound = Lambda2(1) * (p(6) + uba) + Lambda2(2) * (-p(6) + uba) + Lambda2(3) * (p(7) + ubs) + Lambda2(4) * (-p(7) + ubs);

% Combine everything into the Lagrangian
Lagrangian = expr_ext_cost + D + Bound;

% Gradient of the Lagrangian
G_Lx = jacobian(Lagrangian, p(1:5));
G_Lu = jacobian(Lagrangian, p(6:7));
G_L = vertcat(G_Lx', G_Lu');
G = dot(G_L, G_L);
%%
% Define the constraint: sum of Wx1 to Wx5 equals 1
constraint_sum_wx = Wx1 + Wx2 + Wx3 + Wx4 + Wx5 - 1;
%%

% Load your data
load('U_star.mat'); % U_star should be a 2x500 matrix
load('X_star.mat'); % X_star should be a 5x501 matrix
load('References.mat'); % References should be a 5x600 matrix

% Adjust sizes of X_star and References as needed
U_star=u_sim(1:2,:);
X_star=x_sim(1:5,:);
References = [D1;(pi/2)*ones(1,600);5*ones(1,600);zeros(1,600)];

N = size(U_star, 2); % Number of time steps (500)
% N=1;
horizon_length = 100; % Shooting nodes

% Initialize arrays to store optimal weights
W_opt_array = zeros(16, N);
W0 = zeros(16, 1);

% Define the NLP problem
nlp = struct('x', W, 'f', G, 'g', constraint_sum_wx, 'p', vertcat(p, p_ref, p_next));

% Set options for the IPOPT solver
opts = struct('ipopt', struct('print_level', 5, 'tol', 1e-6, 'max_iter', 500));

% Create the solver
solver = nlpsol('solver', 'ipopt', nlp, opts);

% Loop through each time step
for t = 1:N
    % Extract p and p_ref for the current time step
    p_val = [X_star(:, t); U_star(:, t)];
    if t + horizon_length <= size(References, 2)
        p_ref_val = References(:, t:(t + horizon_length - 1));
    else
        p_ref_val = References(:, t:end);
    end

    % Flatten p_ref to a column vector
    p_ref_val = p_ref_val(:);

    % Define p_next (the next state in the horizon)
    if t + 1 <= N
        p_next_val = X_star(1:5, t + 1);
    else
        p_next_val = X_star(1:5, t); % Use the last state if at the end
    end

    % Create the parameters vector
    P_val = [p_val; p_ref_val; p_next_val];

    % Solve the problem
    sol = solver('x0', W0, 'p', P_val, 'lbg', 0, 'ubg', 0);
%     sol = solver('p', P_val, 'lbg', 0, 'ubg', 0);

    % Extract the optimal solution
    W_opt = full(sol.x);
    W_opt_array(:, t) = W_opt;
    W0 = W_opt;
end

% Display or use W_opt_array as needed
% disp(W_opt_array);
