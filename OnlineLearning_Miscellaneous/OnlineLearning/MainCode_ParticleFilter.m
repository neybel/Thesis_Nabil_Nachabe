if 1
[ocp,sim]=SettingUp();
end


num_particles = 100;
% Define the ranges for different components of W_x
W_x_ranges = [
    100, 1000;   % Range for x
    100, 1000;   % Range for y
    1e-5, 1e-1;  % Range for v1
    1e-5, 1e-1;  % Range for v2
    1e-5, 1e-1;  % Range for theta1
    100, 1000;  % Range for theta2
    100, 1000;  % Range for steering1
    1e-5, 1e-1;  % Range for steering2
    1e-5, 1e-1;  % Range for additional parameter 1
    1e-5, 1e-1   % Range for additional parameter 2
];

% Initial state X0, observed trajectory X_sim, and fixed W_u
% W_x = [1e3;1e3;1e-2;1e-2;1e-5;1e3;1e3;1e-2;1e-2;1e-10];
W_u=[1e-3;1e-1;1e-3;1e-1];
x0 = [110; -5; pi/2;10;0;115;0;pi;10;0];

% N is the prediction horizon
load('ground_truth_trajectory.mat');
%% Now only with 1 MPC iteration is the implementation, eventually later we need to loop for the MPC in here
X_sim = x_star(:,:,1); %% State Observations 
N = size(X_sim, 2) - 1; %% Number of shooting nodes from which we extracted from the observed state, this is also equal to the shootign nodes beign used in solving the OCPs in here!

estimated_W_x = particleFilter(X_sim, x0, N, num_particles, W_x_ranges,W_u,ocp);

