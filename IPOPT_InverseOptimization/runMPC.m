%% Define runMPC Function
%%% Note: Technically we are runnign one OCP here so technically not an MPC!
function cost = runMPC(X_sim, U_sim, W_x, W_u)
    import casadi.*
    
    %% Define System Parameters
    nx = size(X_sim, 1);
    nu = size(U_sim, 1);
    N = size(X_sim, 2);
    T = 0.02; % Time step
    
    % Define Symbolic Variables
    x = SX.sym('x', nx, 1);
    u = SX.sym('u', nu, 1);
    
    % Ensure W_x and W_u are diagonal matrices with appropriate sizes
%     W_x = diag(W_x(:)); % W_x should be nx x nx
%     W_u = diag(W_u(:)); % W_u should be nu x nu
    W_x = diag(diag(W_x)); % Ensure W_x is a diagonal matrix
    W_u = diag(diag(W_u)); % Ensure W_u is a diagonal matrix
    W_xe = diag([1e1, 1e1, 1e-2, 1e-1, 1e-2, 1e2, 1e1, 1e-5, 1e-1, 1e-5]); % Example, adjust if needed
    
    %% Dynamics
    L = 2.73;  % Wheelbase [m]
    dx1 = x(4) * cos(x(3));
    dy1 = x(4) * sin(x(3));
    dtheta1 = (x(4) / L) * tan(x(5));
    dv1 = u(1);
    ddelta1 = u(2);
    
    dx2 = x(9) * cos(x(8));
    dy2 = x(9) * sin(x(8));
    dtheta2 = (x(9) / L) * tan(x(10));
    dv2 = u(3);
    ddelta2 = u(4);
    
    rhs = [dx1; dy1; dtheta1; dv1; ddelta1; dx2; dy2; dtheta2; dv2; ddelta2];
    f = Function('f', {x, u}, {rhs});
    
    %% Setup the Cost and Constraints
    U = SX.sym('U', nu, N);  % Control input sequence
    X = SX.sym('X', nx, N+1);  % State sequence
    P = SX.sym('P', nx, N+1);  % Reference states
    
    % Cost
    J = SX(0);
    g1 = SX([]);  % Dynamics constraints
    g2 = SX([]);  % Collision constraints
    
    % Collision avoidance parameters
    dsafe = 1;
    
    for k = 1:N
        st = X(:,k);
        con = U(:,k);
        ref = P(:, k);
        
        % Ensure the dimensions of the matrices are correct
        obj = (st - ref)' * W_x * (st - ref) + con' * W_u * con;
        J = J + obj;
        
        % Dynamics
        f_value = f(st, con);
        st_next_euler = st + T * f_value;
        g1 = [g1; X(:,k+1) - st_next_euler];
        
        % Collision Avoidance
        x1 = X(1:5, k);
        x2 = X(6:10, k);
        dist_squared = ((x1(1) - x2(1))^2) + ((x1(2) - x2(2))^2) - dsafe^2;
        g2 = [g2; dist_squared];
    end
    J = J + (X(:,end) - P(:,end))' * W_xe * (X(:,end) - P(:,end));
    
    g = [g1; g2];
    
    %% Define NLP Problem
    w = [reshape(U, nu*N, 1); reshape(X, nx*(N+1), 1)];
    P = reshape(P, nx*(N+1), 1);
    
    nlp = struct('x', w, 'f', J, 'g', g, 'p', P);
    
    % Set up solver options
    opts = struct('ipopt', struct('print_level', 4, 'tol', 1e-6, 'max_iter', 500));
    solver = nlpsol('solver', 'ipopt', nlp, opts);
    
    % Setup the parameters
    p_val = reshape(X_sim, [], 1);
%%% my line
%     extra = p_val(nx*(N+1)+1:nx*(N+1)+10,1);
extra = p_val(end-9:end);
%     p_val=[p_val(1:nx*(N+1),1);extra;p_val(nx*(N+1)+1:end,1)];
    p_val=[p_val;extra];


    %%%%
    
    % Solve the optimization problem
    lbg = zeros(size(g));
    ubg = inf(size(g));
    lbg(1:numel(X_sim(:,1))) = 0;
    ubg(1:length(g1)) = 0;
    lbg(numel(X_sim(:,1))+1:end) = 0;
    ubg(length(g1):end) = inf;
    
%     sol = solver('x0', zeros(size(w)), 'lbx', -inf * ones(size(w)), 'ubx', inf * ones(size(w)), 'lbg', lbg, 'ubg', ubg, 'p', p_val);
    sol = solver('lbx', -inf * ones(size(w)), 'ubx', inf * ones(size(w)), 'lbg', lbg, 'ubg', ubg, 'p', p_val);
    % Extract cost
    cost = full(sol.f);
end
