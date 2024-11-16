clc
clear all
close all
%% Setting Up acados (OCP and Sim)
if 1
[ocp,sim]=SettingUp();
end
%% Preliminary Parameters for the particle filter.
num_particles = 250;
% Define the ranges for different components of W_x
W_x_ranges = [
    500, 5000;   % Range for x1
    500, 5000;   % Range for y1
    1e-5, 1e-1;  % Range for theta1
    1e-5, 1e-1;  % Range for v1
    1e-5, 1e-1;  % Range for s1
    500, 5000;  % Range for x2
    500, 5000;  % Range for y2
    1e-5, 1e-1;  % Range for theta2
    1e-5, 1e-1;  % Range for v2
    1e-5, 1e-1   % Range for s2
];

% Initial state X0, observed trajectory X_sim, and fixed W_u
% W_x = [1e3;1e3;1e-2;1e-2;1e-5;1e3;1e3;1e-2;1e-2;1e-10];
W_u=[1e-3;1e-1;1e-3;1e-1];
x0 = [110; -5; pi/2;10;0;115;0;pi;10;0];

% N is the prediction horizon
load('ground_truth_trajectory.mat');

x_mpc=[x0];u_mpc=[];w_evol=[];
range_factor = 0.9;  % Start with a larger factor
num_iterations=44;
weight_history = zeros(num_iterations, num_particles);
%% Loop through each MPC iteration
for k=1:num_iterations
X_sim = x_star(:,:,k); %% State Observations 
N = size(X_sim, 2) - 1; %% Number of shooting nodes from which we extracted from the observed state, this is also equal to the shootign nodes beign used in solving the OCPs in here!
[particles,estimated_W_x] = particleFilter(X_sim, x0, N, num_particles, W_x_ranges,W_u,ocp);
%%%% Continuing with the introducation of the next step (MPC iterations)
[traj,u0] = OCP_solve(ocp,x0,estimated_W_x.W_x', W_u)
X_sim_next=Sim_model(sim,u0,x0)
x0=X_sim_next;
x_mpc=[x_mpc x0];u_mpc=[u_mpc u0]; w_evol=[w_evol estimated_W_x.W_x'];
    weight_history(k, :) = particles.weights;
% range_factor = max(0.05, range_factor * 0.9); % Example reduction strategy
% W_x_ranges = updateWxRanges(estimated_W_x.W_x', range_factor); % Adjust 0.1 to control range size
end
%% Figs
%% Fig x_mpc traj
plot(x_mpc(1,:),x_mpc(2,:),'r--',x_mpc(6,:),x_mpc(7,:),'b--')
grid on
return
%% Fig X_mpc traj + x_sim (groundtruth) Comparasion
%%%% Comparaision with the initial observation state
load('Sim_ground_truth.mat');
% hold on
%   road_width = 5;road_length = 60;merging_lane_width = 5;merging_lane_position = 100;rectangle('Position', [60, -2.5, road_length, road_width], 'FaceColor', [0, 1, 1, 0.5]);
%   rectangle('Position', [107.5 -15 7.5 12.5], 'FaceColor', [0, 1, 0, 0.5]);
hold on 
plot(x_sim(1,:),x_sim(2,:),'k--',x_sim(6,:),x_sim(7,:),'g--');
axis equal
xlabel('x [m]');ylabel('y [m]'); title('Trajectory'); grid on; hold on;
    hold on 
 legend('xsimlearned1','xsimleaneard2','xsimgroundtruth1','xsimgroundtruth2');
%% A bar showing each particle (x axis) to its weight before resampling (y axis).!
% Assuming 'particles.weights' contains the weights before resampling.

figure;
bar(1:num_particles, particles.weights);
title('Particle Weights Before Resampling');
xlabel('Particle Index');
ylabel('Weight');
xlim([-10 num_particles]); % Limit x-axis to number of particles
grid on
ylim([0 1.1*max(particles.weights)]);
%% A bar showing how many times each particle is chosen after the resampling procedure!
% Count the number of times each particle was selected
resample_counts = histcounts(resample_indices, 1:(num_particles+1));

% Plot the resampling counts
figure;
bar(1:num_particles, resample_counts);
title('Resampling Counts for Each Particle');
xlabel('Particle Index');
ylabel('Number of Times Resampled');
xlim([-10 num_particles]); % Limit x-axis to number of particles
ylim([-2 1.2*max(resample_counts)])
grid on
