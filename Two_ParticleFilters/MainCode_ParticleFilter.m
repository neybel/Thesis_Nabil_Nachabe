clc
clear all
close all
%% Initial state
x0 = [0;-45;pi/2;0;0;-30;0;0;15;0];
plotRoundaboutWithLanes();
D1=W2_new(x0);D2=W3_new(x0); %% referencing
%% Setting Up acados (OCP and Sim)
if 1
[ocp,sim]=SettingUp(x0);
end
%% Preliminary Parameters for the particle filter.
num_particles = 200;
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
end
% x0 = [0;-60;pi/2;0;0;-30;0;0;0;0];
% N is the prediction horizon
% load('ground_truth_trajectory1.mat');
% load('ground_truth_trajectory2.mat');


x_mpc=[x0];u_mpc=[];w_evol1=[];w_evol2=[];
x_mpc2 = x0(6:end); x_mpc1=x0(1:5);
range_factor = 1;  % Start with a larger factor
num_iterations= 150;
weight_history = zeros(num_iterations, num_particles);

initial_guess_belief_car2 = [12; 12; 2e-4; 2e-4; 2e-4;3e-2;0.50];
initial_guess_belief_car1 = [12; 12; 2e-4; 2e-4; 2e-4;3e-2;0.50];
% W_xu1 = [1e1;1e1;1e-4;1e-4;1e-4;1e-2;1];
ESS1=[]; ESS2=[];
std_dev1 = [7.5; 7.5; 1e-3; 1e-3; 1e-6; 3e-3;0.25];
std_dev2 = [7.5; 7.5; 1e-3; 1e-3; 1e-6; 3e-3;0.25];
std_dev=[10; 10; 1e-3; 1e-3; 1e-6; 3e-3;0.25];

ocp_N=20;
test_theta1=(pi/2)*ones(1,ocp_N+1);
test_theta2=(0)*ones(1, ocp_N + 1);
    testtheta1=test_theta1; testtheta2=test_theta2;
    h = waitbar(0, sprintf('Processing iteration 1 of %d...', num_iterations));


global std_state std_control
std_state = [0; 0; 0.;0.; 0.];  % For (x, y, theta, velocity, steering)
std_control = [0; 0];  % For (acceleration, steering rate)
% std_state = [0.5; 0.5; 0.01; 0.05; 0.02];  % For (x, y, theta, velocity, steering)
% std_control = [0.5; 0.5];  % For (acceleration, steering rate)
%% Loop through each MPC iteration
for k=1:num_iterations
    %% Potential game 1 with belief on 2 (from car 1 perspective)
[X_star1,U_star1,u01,test_theta_full1,test_theta_full2]=OCP_solve(ocp,x0,initial_guess_belief_car2,W_xu1,car,testtheta1,testtheta2,D1,D2); %%% here the full information car know all the weights; in this case they are the same for both agents
X_sim_next1 = Sim_model(sim,u01,x0);
X_sim_next1 = X_sim_next1(1:5); U_sim_next1 = u01(1:2);
%%%% Adding noise
% std_state = [0.1; 0.1; 0.01; 0.05; 0.02];  % For (x, y, theta, velocity, steering)
% std_control = [0.1; 0.01];  % For (acceleration, steering rate)
% Add Gaussian noise to the observed states and controls
X_noisy1 = X_sim_next1 + std_state .* randn(size(X_sim_next1));  % Noisy state observation
U_noisy1 = U_sim_next1 + std_control .* randn(size(U_sim_next1));  % Noisy control input
X_sim_next1=X_noisy1;U_sim_next1=U_noisy1;
%% Potential Game 2 with belief on 1 (from car 2 perspective)
%%% solving the potential game from the ego's perspective with the
%%% estimated weights
[X_star2,U_star2,u02,test_theta_ego1,test_theta_ego2] = OCP_solve(ocp,x0,W_xu1, initial_guess_belief_car1,car,testtheta1,testtheta2,D1,D2);
X_sim_next2=Sim_model(sim,u02,x0);
X_sim_next2=X_sim_next2(6:end);U_sim_next2=u02(3:4);
% Add Gaussian noise to the observed states and controls
X_noisy2 = X_sim_next2 + std_state .* randn(size(X_sim_next2));  % Noisy state observation
U_noisy2 = U_sim_next2 + std_control .* randn(size(U_sim_next2));  % Noisy control input
X_sim_next2=X_noisy2;U_sim_next2=U_noisy2;
%% Particle filter 1 (from car 1 perspective)
N = size(X_star1, 2) - 1; 
pf=1; %it means the pf belongs to car 1
[particles1,estimated_W_xuOfcar2,ess1,likelihoods] = particleFilter(X_sim_next2, U_sim_next2 ,x0, N, num_particles, W_xu_ranges,W_xu1,ocp,initial_guess_belief_car2,range_factor,car,std_dev,testtheta1,testtheta2,pf,D1,D2);
ESS1=[ESS1 ess1];

%% Particle filter 2 (from car 2 perspective)
N = size(X_star2, 2) - 1; 
pf=0; %it means the pf belongs to car 2
[particles2,estimated_W_xu_Ofcar1,ess2] = particleFilter(X_sim_next1, U_sim_next1 ,x0, N, num_particles,W_xu_ranges, W_xu1,ocp,initial_guess_belief_car1,range_factor,car,std_dev,testtheta1,testtheta2,pf,D1,D2);
ESS2=[ESS2 ess2];
%%

x0=[X_sim_next1;X_sim_next2]; %%% updating for the next mpc iteration
u0_mpc = [u01(1:2);u02(3:4)];
x_mpc=[x_mpc x0];u_mpc=[u_mpc u0_mpc]; w_evol2=[w_evol2 estimated_W_xuOfcar2.W_xu'];
w_evol1=[w_evol1 estimated_W_xu_Ofcar1.W_xu'];
%     weight_history(k, :) = particles.weights;
range_factor = max(0.05, range_factor * 0.995); %Reduction strategy
% W_xu_ranges = updateWxuRanges(estimated_W_xu.W_xu', range_factor); 
initial_guess_belief_car1=estimated_W_xu_Ofcar1.W_xu';
initial_guess_belief_car2=estimated_W_xuOfcar2.W_xu';

if car
    testtheta1=test_theta_full1; testtheta2=test_theta_ego2;
end
 waitbar(k / num_iterations, h, ...
        sprintf('Processing iteration %d of %d...', k, num_iterations));
end
close(h);
close all
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

 %% Min distance evolution
 plotMinDistance(x_mpc,num_iterations);
 %% Fig w_evol1
figure()
 % Initialization
w_mean1 = mean(w_evol1, 2);   % Mean of weights at each iteration
w_std1 = std(w_evol1, 0, 2);  % Standard deviation of weights at each iteration

 subplot(3,1,1)
 plot(1:num_iterations,w_evol1(1,:),1:num_iterations,w_evol1(2,:))  
 title('Evolution for Estimation for weights of car 1 ')
 hold on

% Shaded area for Wx1
fill([1:num_iterations fliplr(1:num_iterations)], ...
     [w_evol1(1,:) + w_std1(1,:) fliplr(w_evol1(1,:) - w_std1(1,:))], ...
     'b', 'FaceAlpha', 0.3, 'EdgeColor', 'none');
 grid on
 legend('Wx1','Wx2');
 % Shaded area for Wu1

 subplot(3,1,2)
 plot(1:num_iterations,w_evol1(6,:))
 hold on
 fill([1:num_iterations fliplr(1:num_iterations)], ...
     [w_evol1(6,:) + w_std1(6,:) fliplr(w_evol1(6,:) - w_std1(6,:))], ...
     'g', 'FaceAlpha', 0.3, 'EdgeColor', 'none');
 grid on
 legend('Wu1')
 subplot(3,1,3)
 plot(1:num_iterations,w_evol1(7,:))
 hold on
 % Shaded area for Wu2
fill([1:num_iterations fliplr(1:num_iterations)], ...
     [w_evol1(7,:) + w_std1(7,:) fliplr(w_evol1(7,:) - w_std1(7,:))], ...
     'm', 'FaceAlpha', 0.3, 'EdgeColor', 'none');
 legend('Wu2');
 grid on
  %% Fig w_evol 2
  figure()
 % Initialization
w_mean2 = mean(w_evol2, 2);   % Mean of weights at each iteration
w_std2 = std(w_evol2, 0, 2);  % Standard deviation of weights at each iteration

 subplot(3,1,1)
 plot(1:num_iterations,w_evol2(1,:),1:num_iterations,w_evol2(2,:))
  title('Evolution for Estimation for weights of car 2 ')
 hold on

% Shaded area for Wx1
fill([1:num_iterations fliplr(1:num_iterations)], ...
     [w_evol2(1,:) + w_std2(1,:) fliplr(w_evol2(1,:) - w_std2(1,:))], ...
     'b', 'FaceAlpha', 0.3, 'EdgeColor', 'none');
 grid on
 legend('Wx1','Wx2');
 % Shaded area for Wu1

 subplot(3,1,2)
 plot(1:num_iterations,w_evol2(6,:))
 hold on
 fill([1:num_iterations fliplr(1:num_iterations)], ...
     [w_evol2(6,:) + w_std2(6,:) fliplr(w_evol2(6,:) - w_std2(6,:))], ...
     'g', 'FaceAlpha', 0.3, 'EdgeColor', 'none');
 grid on
 legend('Wu1')
 subplot(3,1,3)
 plot(1:num_iterations,w_evol2(7,:))
 hold on
 % Shaded area for Wu2
fill([1:num_iterations fliplr(1:num_iterations)], ...
     [w_evol2(7,:) + w_std2(7,:) fliplr(w_evol2(7,:) - w_std2(7,:))], ...
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
        pause(0.05); % Adjust the pause time to control the speed of the animation
    end
end

