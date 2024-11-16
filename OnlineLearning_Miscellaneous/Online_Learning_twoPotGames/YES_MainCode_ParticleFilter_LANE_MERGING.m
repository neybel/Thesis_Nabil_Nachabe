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
% W_xu_ranges = [
%     300, 6000;   % Range for x1
%     300, 6000;   % Range for y1
%     1e-5, 1e-1;  % Range for theta1
%     1e-5, 1e-1;  % Range for v1
%     1e-5, 1e-1;  % Range for s1
%     1e-4  1e-1;
%     1e-2  1  
% ];

W_xu_ranges = [
    0.1, 20;   % Range for x1
    0.1, 20;   % Range for y1
    1e-5, 1e-1;  % Range for theta1
    1e-5, 1e-1;  % Range for v1
    1e-5, 1e-1;  % Range for s1
    1e-3  5.0;
    1e-3  5.0  
];

% Initial state X0, observed trajectory X_sim, and fixed W_u
% W_x = [1e3;1e3;1e-2;1e-2;1e-5;1e3;1e3;1e-2;1e-2;1e-10];
% W_u=[1e-3;1e-1;1e-3;1e-1];
global car 
car = 1; % car = 1 means the ego is car 1 (starting from down) 
if car
% W_xu1 = [1e3;1e3;1e-2;1e-2;1e-5;1e-3;1e-1];
W_xu1 = [10;10;1e-2;1e-2;1e-10;1e-1;1e-1];
else
% W_xu1 = [1e3;1e3;1e-2;1e-2;1e-10;1e-3;1e-1];
W_xu1 = [10;10;1e-2;1e-2;1e-10;1e-1;1e-1];
end

% x0 = [110; -5; pi/2;10;0;115;0;pi;10;0]; For groudntruthtraj1
x0 = [110; -5; pi/2;8;0;115;0;pi;8;0];


% N is the prediction horizon
% load('ground_truth_trajectory1.mat');
% load('ground_truth_trajectory2.mat'); Not needed anymore


x_mpc=[x0];u_mpc=[];w_evol=[];
x_mpc2 = x0(6:end); x_mpc1=x0(1:5);
range_factor = 1;  % Start with a larger factor
num_iterations= 80;
weight_history = zeros(num_iterations, num_particles);

% initial_guess = [1500; 1500; 1e-2; 1e-2; 1e-5;0.1;0.05];
initial_guess = [1; 1; 1e-2; 1e-2; 1e-5;0.50;0.50];


ESS=[];
% std_dev = [500; 500; 1e-3; 1e-3; 1e-6; 5e-1;0.10];
std_dev = [5; 5; 1e-3; 1e-3; 1e-6; 0.50;0.50];
h = waitbar(0, sprintf('Processing iteration 1 of %d...', num_iterations));

%% Loop through each MPC iteration
for k=1:num_iterations
%% Potential game 2
%%% we need to run the potential game from the full information car
%%% perspective first:
[X_sim,U_sim,u01]=OCP_solve(ocp,x0,W_xu1,W_xu1,car); %%% here the full information car know all the weights; in this case they are the same for both agents
X_sim_next2 = Sim_model(sim,u01,x0);
if car
X_sim_next2 = X_sim_next2(6:end);
else
X_sim_next2 = X_sim_next2(1:5);
end
%%%%

% X_sim = x_star(:,:,k); %% State Observations 
% X_sim = X_sim(6:end,:);
% U_sim = u_star(:,:,k);
N = size(X_sim, 2) - 1; %% Number of shooting nodes from which we extracted from the observed state, this is also equal to the shootign nodes beign used in solving the OCPs in here!
%% Partile filter
%%% PArticle filter for the ego vehicle
[particles,estimated_W_xu,ess] = particleFilter(X_sim, U_sim ,x0, N, num_particles, W_xu_ranges,W_xu1,ocp,initial_guess,range_factor,car,std_dev);
ESS=[ESS ess];
%% Potential Game 1
%%% solving the potential game from the ego's perspective with the
%%% estimated weights
[traj,contr,u0] = OCP_solve(ocp,x0,estimated_W_xu.W_xu', W_xu1,car);
X_sim_next1=Sim_model(sim,u0,x0);
if car
X_sim_next1 = X_sim_next1(1:5);
else 
X_sim_next1=X_sim_next1(6:end);
end
if car
x0=[X_sim_next1;X_sim_next2]; %%% updating for the next mpc iteration
u0_mpc = [u0(1:2);u01(3:4)];
else
x0=[X_sim_next2;X_sim_next1];
u0_mpc = [u01(1:2);u0(3:4)];
end
x_mpc=[x_mpc x0];u_mpc=[u_mpc u0_mpc]; w_evol=[w_evol estimated_W_xu.W_xu'];
    weight_history(k, :) = particles.weights;
range_factor = max(0.05, range_factor * 0.95); %Reduction strategy
% W_xu_ranges = updateWxuRanges(estimated_W_xu.W_xu', range_factor); 
initial_guess=estimated_W_xu.W_xu';
waitbar(k / num_iterations, h, ...
        sprintf('Processing iteration %d of %d...', k, num_iterations));
end
close(h);

%% Figs
%% Fig x_mpc traj
  road_width = 5;road_length = 60;merging_lane_width = 5;merging_lane_position = 100;
rectangle('Position', [60, -2.5, road_length, road_width], 'FaceColor', [0, 1, 1, 0.5]);
rectangle('Position', [107.5 -15 7.5 12.5], 'FaceColor', [0, 1, 0, 0.5]);
    axis equal;
    xlabel('x [m]');ylabel('y [m]');title('Trajectory');grid on;hold on;
plot(x_mpc(1,:),x_mpc(2,:),'r',x_mpc(6,:),x_mpc(7,:),'b--')
grid on; grid minor;
legend({'agent 1','agent 2'});
hold on 

% % Plot merging lane dashed line (straight)
% y_merge_lane = -15:2.5:-2.5; % Adjust based on the merging lane position and length
% x_merge_lane = 111 * ones(size(y_merge_lane)); % Straight Y coordinates for merging dashed line
% plot(x_merge_lane, y_merge_lane, 'w--', 'LineWidth', 2); % White dashed line

% y_merge_lane1 = -15:2.5:-2.5; % Adjust based on the merging lane position and length
% x_merge_lane = 111 * ones(size(y_merge_lane)); % Straight Y coordinates for merging dashed line
% plot(x_merge_lane, y_merge_lane, 'w--', 'LineWidth', 2); % White dashed line
return
%% Fig X_mpc traj + x_sim (groundtruth) Comparasion
%%%% Comparaision with the initial observation state
% load('Sim_ground_truth1.mat');
load('Sim_ground_truth2.mat');
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

 % Initialization
w_mean = mean(w_evol, 2);   % Mean of weights at each iteration
w_std = std(w_evol, 0, 2);  % Standard deviation of weights at each iteration
figure()
 subplot(3,1,1)
 plot(1:num_iterations,w_evol(1,:),1:num_iterations,w_evol(2,:));
 xlabel('iteration');ylabel('Magnitude')
 hold on

% Shaded area for Wx1
fill([1:num_iterations fliplr(1:num_iterations)], ...
     [w_evol(1,:) + w_std(1,:) fliplr(w_evol(1,:) - w_std(1,:))], ...
     'b', 'FaceAlpha', 0.3, 'EdgeColor', 'none');
 grid on; grid minor;
 legend('Wx1','Wx2');
 title(['Evolution of Wx1 and Wx2 with Ground Truth Wx1 = ', num2str(W_xu1(1)), ', Wx2 = ', num2str(W_xu1(2))]);

 % Shaded area for Wu1
 subplot(3,1,2)
 plot(1:num_iterations,w_evol(6,:));
 xlabel('iteration');ylabel('Magnitude')
 hold on
 fill([1:num_iterations fliplr(1:num_iterations)], ...
     [w_evol(6,:) + w_std(6,:) fliplr(w_evol(6,:) - w_std(6,:))], ...
     'g', 'FaceAlpha', 0.3, 'EdgeColor', 'none');
 grid on; grid minor;
 legend('Wu1')
 title(['Evolution of Wu1 with Ground Truth Wu1 = ', num2str(W_xu1(6))]);

 % Shaded area for Wu2
 subplot(3,1,3)
 plot(1:num_iterations,w_evol(7,:));
 xlabel('iteration');ylabel('Magnitude')
 hold on
fill([1:num_iterations fliplr(1:num_iterations)], ...
     [w_evol(7,:) + w_std(7,:) fliplr(w_evol(7,:) - w_std(7,:))], ...
     'm', 'FaceAlpha', 0.3, 'EdgeColor', 'none');
 legend('Wu2');
 grid on; grid minor;
 title(['Evolution of Wu2 with Ground Truth Wu2 = ', num2str(W_xu1(7))]);

 saveas(gcf, 'W_evol.pdf','pdf');

  %% Figs u comparaision
load('Sim_ground_truth2.mat');
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

% %% A bar showing each particle (x axis) to its weight before resampling (y axis).!
% % Assuming 'particles.weights' contains the weights before resampling.
% 
% figure;
% bar(1:num_particles, particles.weights);
% title('Particle Weights Before Resampling');
% xlabel('Particle Index');
% ylabel('Weight');
% xlim([-10 num_particles]); % Limit x-axis to number of particles
% grid on
% ylim([0 1.1*max(particles.weights)]);
% %% A bar showing how many times each particle is chosen after the resampling procedure!
% % Count the number of times each particle was selected
% resample_counts = histcounts(resample_indices, 1:(num_particles+1));
% 
% % Plot the resampling counts
% figure;
% bar(1:num_particles, resample_counts);
% title('Resampling Counts for Each Particle');
% xlabel('Particle Index');
% ylabel('Number of Times Resampled');
% xlim([-10 num_particles]); % Limit x-axis to number of particles
% ylim([-2 1.2*max(resample_counts)])
% grid on
%% Animation 
if 1


% Main road
dsafe=sqrt(1.0);
    road_width = 5;
road_length = 60;
merging_lane_width = 5;
merging_lane_position = 100;

    figure(700);
rectangle('Position', [60, -2.5, road_length, road_width], 'FaceColor', [0, 1, 1, 0.5]);

% Merging lane
% rectangle('Position', [merging_lane_position - merging_lane_width/2, -road_width, merging_lane_width, road_width], 'FaceColor', 'r');
rectangle('Position', [107.5 -15 7.5 12.5], 'FaceColor', [0, 1, 0, 0.5]);
    axis equal;
    xlabel('x [m]');
    ylabel('y [m]');
    title('Trajectory');
    grid on;
    hold on;
    % Plot merging lane dashed line (straight)
y_merge_lane = -15:2.5:-2.5; % Adjust based on the merging lane position and length
x_merge_lane = 111 * ones(size(y_merge_lane)); % Straight Y coordinates for merging dashed line
plot(x_merge_lane, y_merge_lane, 'w--', 'LineWidth', 2); % White dashed line
hold on

    % Define the circle properties
    theta = linspace(0, 2*pi, 100);
    radius = sqrt(dsafe)/2; % radius is half of the diameter
    x_circle = radius * cos(theta);
    y_circle = radius * sin(theta);

    % Initialize the plot for the car trajectory
    car1_traj = plot(x_mpc(1,1), x_mpc(2,1), 'r');
    car2_traj = plot(x_mpc(6,1), x_mpc(7,1), 'b');
    
    % Initialize the plot for the circle representing each vehicle
    car1_circle = plot(x_mpc(1,1) + x_circle, x_mpc(2,1) + y_circle, 'r');
    car2_circle = plot(x_mpc(6,1) + x_circle, x_mpc(7,1) + y_circle, 'b');
    
    % Set up the axis limits based on the initial position and range of motion
    xlim([min(min(x_mpc(1,:)), min(x_mpc(6,:)))-5, max(max(x_mpc(1,:)), max(x_mpc(6,:)))+5 ]);
    ylim([min(min(x_mpc(2,:)), min(x_mpc(7,:)))-10 , max(max(x_mpc(2,:)), max(x_mpc(7,:)))+10 ]);
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
        pause(0.15); % Adjust the pause time to control the speed of the animation
    end
end

