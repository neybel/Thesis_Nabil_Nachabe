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
W_xu_ranges = [
    0.1, 200;   % Range for x1
    0.1, 200;   % Range for y1
    1e-6, 1e-3;  % Range for theta1
    1e-6, 1e-3;  % Range for v1
    1e-6, 1e-3;  % Range for s1
    1e-3  1e-1;
    1e-2  10  
];
% Initial state X0, observed trajectory X_sim, and fixed W_u
%  W_x1 = [1e1;1e1;1e-4;1e-4;1e-4]; %Car from down to up 
%             W_x2 = [1e1;1e1;1e-4;1e-4;1e-4]; % Car from left to right
%             W_u = [1e-2;1;1e-2;1];
global car 
car = 1; % car = 1 means the ego is car 1 (starting from down) 
if car
W_xu1 = [1e1;1e1;1e-4;1e-4;1e-4;1e-2;1];
else
W_xu1 = [1e1;1e1;1e-4;1e-4;1e-4;1e-2;1];
end
x0 = [0;-60;pi/2;0;0;-30;0;0;0;0];
% N is the prediction horizon
% load('ground_truth_trajectory1.mat');
load('ground_truth_trajectory2.mat');


x_mpc=[x0];u_mpc=[];w_evol=[];
range_factor = 1;  % Start with a larger factor
num_iterations= 120;
weight_history = zeros(num_iterations, num_particles);
% initial_guess = [2000; 2000; 1e-2; 1e-2; 1e-5;0.005;0.25];

initial_guess = [12.5; 12.5; 1e-2; 1e-2; 1e-5;3e-2;0.50];
% initial_guess = [100; 100; 1e-2; 1e-2; 1e-5;3e-1;1.5];


ESS=[];
std_dev = [7.5; 7.5; 1e-3; 1e-3; 1e-6; 3e-3;0.25];
% std_dev = [70; 70; 1e-3; 1e-3; 1e-6; 3e-2;1];

ocp_N=20;
test_theta1=(pi/2)*ones(1,ocp_N+1);
test_theta2=(0)*ones(1, ocp_N + 1);
    testtheta1=test_theta1; testtheta2=test_theta2;
    h = waitbar(0, sprintf('Processing iteration 1 of %d...', num_iterations));

%% Loop through each MPC iteration
for k=1:num_iterations
X_sim = x_star(:,:,k); %% State Observations 
% X_sim = X_sim(6:end,:);
U_sim = u_star(:,:,k);
N = size(X_sim, 2) - 1; %% Number of shooting nodes from which we extracted from the observed state, this is also equal to the shootign nodes beign used in solving the OCPs in here!
[particles,estimated_W_xu,ess] = particleFilter(X_sim, U_sim ,x0, N, num_particles, W_xu_ranges,W_xu1,ocp,initial_guess,range_factor,car,std_dev,testtheta1,testtheta2);
ESS=[ESS ess];
%%%% Continuing with the introducation of the next step (MPC iterations)
[traj,contr,u0,test_theta1,test_theta2] = OCP_solve(ocp,x0,estimated_W_xu.W_xu', W_xu1,car,testtheta1,testtheta2);
X_sim_next=Sim_model(sim,u0,x0);
x0=X_sim_next;
x_mpc=[x_mpc x0];u_mpc=[u_mpc u0]; w_evol=[w_evol estimated_W_xu.W_xu'];
    weight_history(k, :) = particles.weights;
range_factor = max(0.05, range_factor * 0.9850); %Reduction strategy
% W_xu_ranges = updateWxuRanges(estimated_W_xu.W_xu', range_factor); 
initial_guess=estimated_W_xu.W_xu';
    testtheta1=test_theta1; testtheta2=test_theta2;
 waitbar(k / num_iterations, h, ...
        sprintf('Processing iteration %d of %d...', k, num_iterations));
end
close(h);
%% Figs
%% Fig x_mpc traj
  plotRoundaboutWithLanes();
hold on;
plot(x_mpc(1,:),x_mpc(2,:),'r--',x_mpc(6,:),x_mpc(7,:),'b--')
grid on
return
%% Fig X_mpc traj + x_sim (groundtruth) Comparasion
%%%% Comparaision with the initial observation state
% load('Sim_ground_truth1.mat');
load('Sim_traj.mat');
% hold on
%   road_width = 5;road_length = 60;merging_lane_width = 5;merging_lane_position = 100;rectangle('Position', [60, -2.5, road_length, road_width], 'FaceColor', [0, 1, 1, 0.5]);
%   rectangle('Position', [107.5 -15 7.5 12.5], 'FaceColor', [0, 1, 0, 0.5]);
hold on 
plot(x_sim(1,:),x_sim(2,:),'k--',x_sim(6,:),x_sim(7,:),'p--');
axis equal
xlabel('x [m]');ylabel('y [m]'); title('Trajectory'); grid on; hold on;
    hold on 
 legend('xsimlearned1','xsimleaneard2','xsimgroundtruth1','xsimgroundtruth2');
 %% Fig w_evol
 %% Fig w_evol

 % Initialization
w_mean = mean(w_evol, 2);   % Mean of weights at each iteration
w_std = std(w_evol, 0, 2);  % Standard deviation of weights at each iteration

 subplot(3,1,1)
 plot(1:num_iterations,w_evol(1,:),1:num_iterations,w_evol(2,:))
 hold on

% Shaded area for Wx1
fill([1:num_iterations fliplr(1:num_iterations)], ...
     [w_evol(1,:) + w_std(1,:) fliplr(w_evol(1,:) - w_std(1,:))], ...
     'b', 'FaceAlpha', 0.3, 'EdgeColor', 'none');
 grid on
 legend('Wx1','Wx2');
 % Shaded area for Wu1

 subplot(3,1,2)
 plot(1:num_iterations,w_evol(6,:))
 hold on
 fill([1:num_iterations fliplr(1:num_iterations)], ...
     [w_evol(6,:) + w_std(6,:) fliplr(w_evol(6,:) - w_std(6,:))], ...
     'g', 'FaceAlpha', 0.3, 'EdgeColor', 'none');
 grid on
 legend('Wu1')
 subplot(3,1,3)
 plot(1:num_iterations,w_evol(7,:))
 hold on
 % Shaded area for Wu2
fill([1:num_iterations fliplr(1:num_iterations)], ...
     [w_evol(7,:) + w_std(7,:) fliplr(w_evol(7,:) - w_std(7,:))], ...
     'm', 'FaceAlpha', 0.3, 'EdgeColor', 'none');
 legend('Wu2');
 grid on

 %% Figs u comparaision
 load('Sim_traj.mat');
 subplot(2,2,1)
 plot(1:length(u_mpc),u_sim(1,1:length(u_mpc)),'r')
 hold on
  plot(1:length(u_mpc),u_mpc(1,1:length(u_mpc)),'b')
 title('Acceleration 1')
 grid on 
 legend('Ground_truth','Learned')
  subplot(2,2,2)
 plot(1:length(u_mpc),u_sim(2,1:length(u_mpc)),'r')
  hold on
  plot(1:length(u_mpc),u_mpc(2,1:length(u_mpc)),'b')
  title('Steering Rate 1')
  grid on
   legend('Ground_truth','Learned')

  subplot(2,2,3)
 plot(1:length(u_mpc),u_sim(3,1:length(u_mpc)),'r')
  hold on
  plot(1:length(u_mpc),u_mpc(3,1:length(u_mpc)),'b')
  title('Acceleration 2')
  grid on
   legend('Ground_truth','Learned')

  subplot(2,2,4)
 plot(1:length(u_mpc),u_sim(4,1:length(u_mpc)),'r')
  hold on
  plot(1:length(u_mpc),u_mpc(4,1:length(u_mpc)),'b')
   title('Steering Rate 2')
   grid on
    legend('Ground_truth','Learned')




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
%% Animation 
if 1


dsafe=12.50;
plotRoundaboutWithLanes();
%     axis equal;
    xlabel('x [m]');
    ylabel('y [m]');
    title('Trajectory');
    grid on;
    hold on;

    % Define the circle properties
    theta = linspace(0, 2*pi, 100);
    radius = sqrt(dsafe/2); % radius is half of the diameter
    x_circle = radius * cos(theta);
    y_circle = radius * sin(theta);

    % Initialize the plot for the car trajectory
    car1_traj = plot(x_mpc(1,1), x_mpc(2,1), 'r');
    car2_traj = plot(x_mpc(6,1), x_mpc(7,1), 'b');
    
    % Initialize the plot for the circle representing each vehicle
    car1_circle = plot(x_mpc(1,1) + x_circle, x_mpc(2,1) + y_circle, 'r');
    car2_circle = plot(x_mpc(6,1) + x_circle, x_mpc(7,1) + y_circle, 'b');
    
    % Set up the axis limits based on the initial position and range of motion
%     xlim([min(min(x_mpc(1,:)), min(x_mpc(6,:)))-5, max(max(x_mpc(1,:)), max(x_mpc(6,:)))+5 ]);
%     ylim([min(min(x_mpc(2,:)), min(x_mpc(7,:)))-10 , max(max(x_mpc(2,:)), max(x_mpc(7,:)))+10 ]);
%     axis equal;
%     
    % Animation loop
    for k = 2:length(x_mpc)
        % Plot the trajectories of the cars
        set(car1_traj, 'XData', x_mpc(1,1:k), 'YData', x_mpc(2,1:k));
        set(car2_traj, 'XData', x_mpc(6,1:k), 'YData', x_mpc(7,1:k));
        
        % Update the positions of the circles representing the vehicles
        set(car1_circle, 'XData', x_mpc(1,k) + x_circle, 'YData', x_mpc(2,k) + y_circle);
        set(car2_circle, 'XData', x_mpc(6,k) + x_circle, 'YData', x_mpc(7,k) + y_circle);

        % Pause to create animation effect
        pause(0.10); % Adjust the pause time to control the speed of the animation
    end
end

