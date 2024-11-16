%% You need to run the joint optimization first (Run Pre_IBR.m)
%%%% check the referencing in the solvers 1 and 2 (the p how is it being
%%%% treated)
%% Setting up solver 1
import casadi.*
nu=2;nx=5;
% N=300;L=2.73; T=0.02; dsafe=4.0;
% X2_opt = X_opt(6:10, :);

states1 = SX.sym('states1', 5, 1); % State variables for Vehicle 1
controls1 = SX.sym('controls1', 2, 1); % Control variables for Vehicle 1

% Define the dynamics for Vehicle 1
dx1 = states1(4) * cos(states1(3));
dy1 = states1(4) * sin(states1(3));
dtheta1 = (states1(4) / L) * tan(states1(5));
dv1 = controls1(1);
ddelta1 = controls1(2);

rhs1 = [dx1; dy1; dtheta1; dv1; ddelta1];
f1 = Function('f1', {states1, controls1}, {rhs1});

% Define new cost function weights for Vehicle 1

% W_x1 = diag([1e3, 1e3, 1e-2, 1e1, 1e-5]);
% W_xe = diag([1e5, 1e5, 1e-2, 1e-1, 1e-2]);
% W_u1 = diag([1e-1, 1e-2]);
W_x1 = diag([W_x(1,1), W_x(2,2), W_x(3,3), W_x(4,4), W_x(5,5)]);
W_xe1 = diag([W_xe(1,1), W_xe(2,2), W_xe(3,3), W_xe(4,4), W_xe(5,5)]);
W_u1 = diag([W_u(1,1), W_u(2,2)]);

% Initialize symbolic variables for Vehicle 1 OCP
U1 = SX.sym('U1', 2, N); % Control input sequence for Vehicle 1
X1 = SX.sym('X1', 5, N+1); % State sequence for Vehicle 1
P1 = SX.sym('P1', 5, N+1); % Reference states for Vehicle 1

P11=SX.sym('P11',5,N+1); %reference of the fixed vehicle (car 2 here)



% Initialize cost and constraints for Vehicle 1 OCP
J1 = SX(0);
g1 = SX([]);
g2 = SX([]);
g = SX([]);


% Set up the cost and constraints over the prediction horizon for Vehicle 1
for k = 1:N
    st1 = X1(:,k);
    con1 = U1(:,k);
    state_ref1 = P1(:, k);
    
    % Update the cost
    obj1 = (st1 - state_ref1)' * W_x1 * (st1 - state_ref1) + con1' * W_u1 * con1;
    J1 = J1 + obj1;

    % Compute the next state using the dynamics function
    f_value1 = f1(st1, con1);
    st_next_euler1 = st1 + T * f_value1;
    
    % Dynamics constraint (equality constraint)
    g1 = [g1; X1(:,k+1) - st_next_euler1];

    % Collision avoidance constraint with fixed Vehicle 2 trajectory
    x1 = X1(1:5, k);
    x2 = P11(:, k);
    dist_squared = ((x1(1) - x2(1))^2) + ((x1(2) - x2(2))^2) - dsafe^2;
    g2 = [g2; dist_squared];
end
g_1=[g1;g2];

% Add terminal cost for Vehicle 1
J1 = J1 + (X1(:,end) - P1(:,end))' * W_xe1 * (X1(:,end) - P1(:,end));

% Vectorize the decision variables for Vehicle 1 OCP
w1 = [reshape(U1, 2*N, 1); reshape(X1, 5*(N+1), 1)];
P1 = reshape(P1, 5*(N+1), 1);
P11 = reshape(P11,5*(N+1),1);
P1_new=[P1;P11];

% Define the new NLP problem for Vehicle 1
% opts = struct('ipopt', struct('print_level', 5, 'tol', 1e-6, 'max_iter', 1500));
nlp1 = struct('x', w1, 'f', J1, 'g', g_1, 'p', P1_new);

% Initialize solver for Vehicle 1
solver1 = nlpsol('solver1', 'ipopt', nlp1, opts);
%% Setting up solver 2
% X_opt = load('traj_ipopt.mat');
% % X_opt = reshape(w_opt(nu*N + 1:end), nx, N+1);
% nu=2;nx=5;
% N=400;
% X2_opt = X_opt(6:10, :);
% nu=2;nx=5;
% N=400;L=2.73; T=0.02; dsafe=2.50;
% Define a new OCP for Vehicle 1, fixing the trajectory of Vehicle 2
states1 = SX.sym('states1', 5, 1); % State variables for Vehicle 1
controls1 = SX.sym('controls1', 2, 1); % Control variables for Vehicle 1

% Define the dynamics for Vehicle 1
dx1 = states1(4) * cos(states1(3));
dy1 = states1(4) * sin(states1(3));
dtheta1 = (states1(4) / L) * tan(states1(5));
dv1 = controls1(1);
ddelta1 = controls1(2);

rhs1 = [dx1; dy1; dtheta1; dv1; ddelta1];
f1 = Function('f1', {states1, controls1}, {rhs1});

% Define new cost function weights for Vehicle 1
% W_x1 = diag([1e3, 1e3, 1e-2, 1e-1, 1e-10]);
% W_u1 = diag([1e-3, 1e-3]);
% W_xe = diag([1e2, 1e1, 1e-5, 1e-1, 1e-5]);
W_x2 = diag([W_x(6,6), W_x(7,7), W_x(8,8), W_x(9,9), W_x(10,10)]);
W_xe2 = diag([W_xe(6,6), W_xe(7,7), W_xe(8,8), W_xe(9,9), W_xe(10,10)]);
W_u2 = diag([W_u(3,3), W_u(4,4)]);



% Initialize symbolic variables for Vehicle 1 OCP
U1 = SX.sym('U1', 2, N); % Control input sequence for Vehicle 1
X1 = SX.sym('X1', 5, N+1); % State sequence for Vehicle 1
P1 = SX.sym('P1', 5, N+1); % Reference states for Vehicle 1

P12=SX.sym('P12',5,N+1); %reference of the fixed vehicle (car 2 here)



% Initialize cost and constraints for Vehicle 1 OCP
J1 = SX(0);
g1 = SX([]);
g2 = SX([]);
g = SX([]);


% Set up the cost and constraints over the prediction horizon for Vehicle 1
for k = 1:N
    st1 = X1(:,k);
    con1 = U1(:,k);
    state_ref1 = P1(:, k);
    
    % Update the cost
    obj1 = (st1 - state_ref1)' * W_x2 * (st1 - state_ref1) + con1' * W_u2 * con1;
    J1 = J1 + obj1;

    % Compute the next state using the dynamics function
    f_value1 = f1(st1, con1);
    st_next_euler1 = st1 + T * f_value1;
    
    % Dynamics constraint (equality constraint)
    g1 = [g1; X1(:,k+1) - st_next_euler1];

    % Collision avoidance constraint with fixed Vehicle 2 trajectory
    x1 = X1(1:5, k);
    x2 = P12(:, k);
    dist_squared = ((x1(1) - x2(1))^2) + ((x1(2) - x2(2))^2) - dsafe^2;
    g2 = [g2; dist_squared];
end
g_2=[g1;g2];

% Add terminal cost for Vehicle 1
J1 = J1 + (X1(:,end) - P1(:,end))' * W_xe2 * (X1(:,end) - P1(:,end));

% Vectorize the decision variables for Vehicle 1 OCP
w1 = [reshape(U1, 2*N, 1); reshape(X1, 5*(N+1), 1)];
P1 = reshape(P1, 5*(N+1), 1);
P12 = reshape(P12,5*(N+1),1);
P2_new=[P1;P12];

% Define the new NLP problem for Vehicle 1
% opts = struct('ipopt', struct('print_level', 5, 'tol', 1e-6, 'max_iter', 1500));
nlp2 = struct('x', w1, 'f', J1, 'g', g_2, 'p', P2_new);

% Initialize solver for Vehicle 1
solver2 = nlpsol('solver2', 'ipopt', nlp2, opts);
%% solver 1/2 stuffs (fixing car 2 and solve for 1) regarding the bounds
% check external function
x01 = [110; -20; pi/2; 2; 0];
x02 = [125; 0.5; pi; 4; 0];

lbx1 = -inf * ones(nu*N + nx*(N+1), 1);
ubx1 = inf * ones(nu*N + nx*(N+1), 1);
lbx2 = -inf * ones(nu*N + nx*(N+1), 1);
ubx2 = inf * ones(nu*N + nx*(N+1), 1);

% Bounds on controls
lbu1 = repmat([-11.5; -1], N, 1);
ubu1 = repmat([11.5; 1], N, 1);
lbu2=lbu1; ubu2=ubu1;

% Apply bounds to decision variables
lbx1(1:nu*N) = lbu1;
ubx1(1:nu*N) = ubu1;
lbx2(1:nu*N) = lbu2;
ubx2(1:nu*N) = ubu2;

% Initial state constraints
lbx1(nu*N+1:nu*N+nx) = x01;
ubx1(nu*N+1:nu*N+nx) = x01;
lbx2(nu*N+1:nu*N+nx) = x02;
ubx2(nu*N+1:nu*N+nx) = x02;

% Positive velocity constraints for the entire horizon
for k = 1:N
    lbx1(nu*N + (k-1)*nx + 4) = 0; % x4 >= 0
    lbx1(nu*N + (k-1)*nx + 9) = 0; % x9 >= 0
    lbx2(nu*N + (k-1)*nx + 4) = 0; % x4 >= 0
    lbx2(nu*N + (k-1)*nx + 9) = 0; % x9 >= 0
end
% lbg1 = zeros(size(g)); % Lower bounds for g
% ubg1 = inf(size(g));   % Upper bounds for g
lbg1 = zeros(size(g1)); % Lower bounds for g1 (dynamics constraints)
ubg1 = zeros(size(g1)); % Upper bounds for g1 (dynamics constraints)
lbg2 = zeros(size(g2)); % Lower bounds for g2 (collision avoidance constraints)
ubg2 = inf(size(g2));   % Upper bounds for g2 (collision avoidance constraints)
lbg = [lbg1; lbg2];
ubg = [ubg1; ubg2];
%% Referencing
% Load reference waypoints
waypoints = Way_IPOPT();
waypoint_count = N+1;

% Prepare reference states for both vehicles
ref1 = zeros(5, waypoint_count);
ref2 = zeros(5, waypoint_count);

% Vehicle 1 reference state
ref1(1, :) = waypoints(1, 1:waypoint_count);  % x positions
ref1(2, :) = waypoints(2, 1:waypoint_count);  % y positions
ref1(3, :) = (pi/2)*ones(1, waypoint_count);  % orientations
ref1(4, :) = 2 * ones(1, waypoint_count);  % constant velocity
ref1(5, :) = zeros(1, waypoint_count);  % steering angle

% Vehicle 2 reference state
ref2(1, :) = linspace(125, 125 - 4*waypoint_count*T, waypoint_count);  % x positions (straight line)
ref2(2, :) = 0.5*ones(1, waypoint_count);  % y positions (constant)
ref2(3, :) = pi * ones(1, waypoint_count);  % orientations
ref2(4, :) = 2 * ones(1, waypoint_count);  % constant velocity
ref2(5, :) = zeros(1, waypoint_count);

%% IBR
% Initial trajectories for vehicles
X1_init = x_opt(1:5, :);
X2_init = x_opt(6:10, :);
% X1_opt=X1_init;
% X2_opt=X2_init;

% u0_guess=reshape(u_opt,[],1);
% u0_guess1= u0_guess(1:end/2,:);
% u0_guess2= u0_guess((end/2)+1:end,:);

u0_guess1 = reshape(u_opt(1:2, :), [], 1); % Initial guess for control inputs of vehicle 1
u0_guess2 = reshape(u_opt(3:4, :), [], 1); 
% Set initial trajectories for iterated best response
X1_prev = X1_init;
X2_prev = X2_init;

% Iteration parameters
max_iterations = 100;
tolerance = 1e-6;
converged = false;
iteration = 0;

while ~converged && iteration < max_iterations
    iteration = iteration + 1;
    
    % Fix X2_prev and solve for X1
    X2_opt = X2_prev;
%     p1_val = reshape([ref1; X2_opt], [], 1); % Combine ref1 and X2_opt for parameter vector
a1=reshape(ref1,[],1);
a2=reshape(X2_opt,[],1);
p1_val=[a1;a2];

%     u0_guess1 = zeros(nu*N, 1);
%     u0_guess2= u0_guess1;
    
%     w01 = [u0_guess1; reshape(ref1,[],1)];
    w01 = [u0_guess1; reshape(X1_prev,[],1)];

%         w01 = [u0_guess1; reshape(X1_init,[],1)];



    sol1 = solver1('x0',w01,'lbx', lbx1, 'ubx', ubx1, 'lbg', lbg, 'ubg', ubg, 'p', p1_val);
    w1_opt = full(sol1.x);
    X1_new = reshape(w1_opt(2*N + 1:end), 5, N+1);
    
    % Fix X1_new and solve for X2
    X1_opt = X1_new;


%     p2_val = reshape([X1_opt; ref2], [], 1); % Combine X1_opt and ref2;  I think this was wrong as in ordering
%     p2_val = reshape([ref2; X1_opt], [], 1); % Combine X1_opt and ref2 for parameter vector

a3=reshape(ref2,[],1);
a4=reshape(X1_opt,[],1);
p2_val=[a3;a4];

  
%     w02 = [u0_guess2; reshape(ref2,[],1)];
      w02 = [u0_guess2; reshape(X2_prev,[],1)];
%       w02 = [u0_guess2; reshape(X2_init,[],1)];



    sol2 = solver2('x0',w02,'lbx', lbx2, 'ubx', ubx2, 'lbg', lbg, 'ubg', ubg, 'p', p2_val);
    w2_opt = full(sol2.x);
    X2_new = reshape(w2_opt(2*N + 1:end), 5, N+1);
    
    % Check for convergence 
    %%% 'fro' refer to the frobenius norm -> which is technically the
    %%% euclidean norm for matrices; which sums the squares of all elements and takes the square root of the sum
    diff_X1 = norm(X1_new - X1_prev, 'fro');
    diff_X2 = norm(X2_new - X2_prev, 'fro');
    
    if diff_X1 < tolerance && diff_X2 < tolerance
        converged = true;
    else
        % Update previous trajectories
        X1_prev = X1_new;
        X2_prev = X2_new;
    end
end

if converged
    disp('Nash Equilibrium found');
else
    disp('Maximum iterations reached without convergence');
end

% Final trajectories
X1_opt = X1_new;
X2_opt = X2_new;
keyboard
%% Plotting
road_width = 20;road_length = 200;merging_lane_width = 10;merging_lane_position = 100;
% Plot road
figure;
hold on;
% Main road
rectangle('Position', [0, -road_width/2, road_length, road_width], 'FaceColor', 'g');

% Merging lane
% rectangle('Position', [merging_lane_position - merging_lane_width/2, -road_width, merging_lane_width, road_width], 'FaceColor', 'r');
rectangle('Position', [95 -60 20 50], 'FaceColor', 'c');

% Set axis properties
axis equal;
xlabel('x [m]');
ylabel('y [m]');
title('Road');
grid on;
hold on
plot(X1_opt(1,:),X1_opt(2,:),'r--',X2_opt(1,:),X2_opt(2,:),'b--');
return


