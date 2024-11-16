import casadi.*

horizon_length = 100; % Shooting nodes
d_safe = 15.50; % Safety distance in meters
alpha = 10.0; % Collision avoidance parameter

% Define the optimization variables
Wx1 = SX.sym('Wx1');
Wx2 = SX.sym('Wx2');
Wx3 = SX.sym('Wx3');
Wx4 = SX.sym('Wx4');
Wx5 = SX.sym('Wx5');
Wu1 = SX.sym('Wu1');
Wu2 = SX.sym('Wu2');
Lambda2 = SX.sym('Lambda2', 4); %% for the control bounds
lambda1 = SX.sym('lambda1', 5); %%% for the dynamics 
lambda_collision = SX.sym('lambda_collision', 3); % Lagrange multipliers for collision avoidance
W = [Wx1; Wx2; Wx3; Wx4; Wx5; Wu1; Wu2; Lambda2; lambda1; lambda_collision];

% Define the parameters
p = SX.sym('p', 7);
p_ref = SX.sym('p_ref', 5 * horizon_length); % Adjusted for horizon length
p_next = SX.sym('p_next', 5);
p_others = SX.sym('p_others', 5 * 3); % Assuming 3 other vehicles

% Normalization factors (e.g., standard deviations or ranges)
state_norm_factors = [100; 100; 0.0001; 0.1; 0.0001]; % Example normalization factors for states
input_norm_factors = [0.01; 1]; % Example normalization factors for inputs

% Normalize states and inputs
p_norm = [p(1:5)./state_norm_factors; p(6:7)./input_norm_factors];
p_ref_norm = reshape(p_ref, [5, horizon_length]) ./ state_norm_factors;
p_ref_norm = p_ref_norm(:);
p_next_norm = p_next ./ state_norm_factors;

% Define weights
W_x_raw = [Wx1, Wx2, Wx3, Wx4, Wx5];
W_u_raw = [Wu1, Wu2];

% Normalized weights
W_x = W_x_raw / sum(W_x_raw);
W_u = W_u_raw / sum(W_u_raw);

% Define the objective function components
Gamma1 = -5.0;
state_cost = (p_norm(1:5) - p_ref_norm(1:5))' * diag(W_x) * (p_norm(1:5) - p_ref_norm(1:5));
input_cost = p_norm(6:7)' * diag(W_u) * p_norm(6:7);
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
D = lambda1' * (p_next_norm - p_norm(1:5) - dt * f_k ./ state_norm_factors);

%%%%%%
% Control bounds
uba = 11.5;
ubs = 1;
Bound = Lambda2(1) * (p(6) + uba) + Lambda2(2) * (-p(6) + uba) + Lambda2(3) * (p(7) + ubs) + Lambda2(4) * (-p(7) + ubs);
%%%%%%

% Define collision avoidance constraints
% Assuming p_others contains states of 3 other vehicles (5 states each)
h = SX.zeros(3, 1);
C = SX.zeros(3, 1);

for i = 1:3
    x_other = p_others((i-1)*5 + 1 : i*5);
    
    dist_squared = ((p(1) - x_other(1))^2) + ((p(2) - x_other(2))^2) - d_safe^2;
    h(i) = dist_squared;
    
    a1 = p(1) - x_other(1); 
    a2 = p(4) * cos(p(3)) - x_other(4) * cos(x_other(3));
    b1 = p(2) - x_other(2); 
    b2 = p(4) * sin(p(3)) - x_other(4) * sin(x_other(3));
    dh_dt = 2 * (a1 * a2 + b1 * b2);
    
    C(i) = dh_dt + alpha * h(i);
end

% Combine everything into the Lagrangian
Lagrangian = expr_ext_cost + D + Bound + lambda_collision' * C;

% Gradient of the Lagrangian
G_Lx = jacobian(Lagrangian, p(1:5));
G_Lu = jacobian(Lagrangian, p(6:7));
G_L = vertcat(G_Lx', G_Lu');
G = dot(G_L, G_L);

% Define the constraints: sum of W_x_raw equals 1 and W_x_raw, W_u_raw > 0
% constraint_sum_wx = sum(W_x_raw) - 1;
constraint_sum_wx = 0;
constraints_positivity = [W_x_raw'; W_u_raw'];

% Load your data
load('U_star.mat'); % U_star should be a 2x500 matrix
load('X_star.mat'); % X_star should be a 5x501 matrix
load('References.mat'); % References should be a 5x600 matrix

% Adjust sizes of X_star and References as needed
U_star = u_sim(1:2, :);
X_star = x_sim(1:5, :);
References = [D1; zeros(3, 600)];

N = size(U_star, 2); % Number of time steps (500)

% Initialize arrays to store optimal weights
W_opt_array = zeros(19, N); % Adjusted for new W size
% W0 = [10; 10; 0.0001; 0.01; 0.0001; 1; 1; ones(4, 1); ones(5, 1); ones(3, 1)]; % Initial weights based on forward optimization
W0 =  [100; 100; 0.001; 0.1; 0.001; 1; 1; ones(4, 1); ones(5, 1); ones(3, 1)]; % Initial weights based on forward optimization


% Define the NLP problem
nlp = struct('x', W, 'f', G, 'g', [constraint_sum_wx; constraints_positivity], 'p', vertcat(p, p_ref, p_next, p_others));

% Set options for the IPOPT solver
opts = struct('ipopt', struct('print_level', 5, 'tol', 1e-6, 'max_iter', 500));

% Create the solver
solver = nlpsol('solver', 'ipopt', nlp, opts);

% Define the bounds for the constraints
lbg = [0; zeros(7, 1)];
ubg = [0; inf*ones(7, 1)];
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
    p_others_val = reshape(x_sim(6:end, t), [5*3, 1]); % Assuming 3 other vehicles
    P_val = [p_val; p_ref_val; p_next_val; p_others_val];

    % Solve the problem
    sol = solver('x0', W0, 'p', P_val, 'lbg', lbg, 'ubg', ubg);
%     sol = solver('p', P_val, 'lbg', lbg, 'ubg', ubg);


    % Extract the optimal solution
    W_opt = full(sol.x);
    W_opt_array(:, t) = W_opt;
    W0 = W_opt;
end

% Display or use W_opt_array as needed
% disp(W_opt_array);
subplot(2,1,1)
plot(1:N,W_opt_array(1,:),'r--')
subplot(2,1,2)
plot(1:N,W_opt_array(2,:),'b--')