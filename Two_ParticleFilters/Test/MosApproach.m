clc
clear all
close all
addpath("C:\Users\user\acados\examples\acados_matlab_octave")
%% Initial state
x0 = [0;-45;pi/2;0;0;-30;0;0;10;0];
plotRoundaboutWithLanes();
D1=W2_newnew(x0);D2=W3_newnew(x0); %% referencing
%% Setting Up acados (OCP and Sim)
if 1
[ocp,sim]=customSettingUp(x0);
end
%% Preliminary Parameters for the particle filter.
num_particles = 250;
customnum_particles=250;
% Define the ranges for different components of W_x
W_xu_ranges = [
    0.1, 25;   % Range for x
    0.1, 25;   % Range for y
    1e-6, 1e-3;  % Range for theta
    1e-6, 1e-3;  % Range for v
    1e-6, 1e-3;  % Range for s
    1e-2  5;
    1e-2  5  
];
global car 
car = 1; % car = 1 means the ego is car 1 (starting from down) 
if car
% W_xu1 = [1e1;1e1;1e-4;1e-4;1e-4;1e-2;1e-1];

    W_xu11 = [10;10;1e-4;1e-4;1e-4;0.50;0.50]; %weights car 1
    W_xu12 = [10;10;1e-4;1e-4;1e-4;0.50;0.1]; %weights car 2
W_xu11_str = sprintf('%.1f, %.1f, %.1e, %.1e, %.1e, %.1f, %.1f', W_xu11);
W_xu12_str = sprintf('%.1f, %.1f, %.1e, %.1e, %.1e, %.1f, %.1f', W_xu12);
end
x_mpc=[x0];u_mpc=[];
x_mpc2 = x0(6:end); x_mpc1=x0(1:5);
range_factor = 1;  % Start with a larger factor
num_iterations= 150;
weight_history = zeros(num_iterations, num_particles);
% initial_guess_belief_car2 = [1; 1 ;1e-2; 1e-2; 1e-5;3e-2;0.5];
% initial_guess_belief_car1 = [1 ; 1; 1e-2; 1e-2; 1e-5;3e-2;0.5];
initial_guess_belief_car2 = [1; 1; 1e-2; 1e-2; 1e-5;3e-2;0.10];
initial_guess_belief_car1 = [1; 1; 1e-2; 1e-2; 1e-5;3e-2;0.50];
% initial_guess_belief_car2 = W_xu1;
% initial_guess_belief_car1 = W_xu1;
ESS1=[]; ESS2=[];
w_evol1=[initial_guess_belief_car1];w_evol2=[initial_guess_belief_car2];
% std_dev_custom=[5; 5; 1e-3; 1e-3; 1e-6; 0.1;0.1];
std_dev_custom=[5; 5; 1e-3; 1e-3; 1e-6; 0.2;0.2];
ocp_N=40;
test_theta1=(pi/2)*ones(1,ocp_N+1);
test_theta2=(0)*ones(1, ocp_N + 1);
    testtheta1=test_theta1; testtheta2=test_theta2;
    h = waitbar(0, sprintf('Processing iteration 1 of %d...', num_iterations));
global std_state std_control
std_state = [0; 0; 0;0; 0];  % For (x, y, theta, velocity, steering)
std_control = [0; 0];  % For (acceleration, steering rate)
% std_state = [0.5; 0.5; 0.01; 0.05; 0.02];  % For (x, y, theta, velocity, steering)
% std_control = [0.5; 0.5];  % For (acceleration, steering rate)
% std_state = [0.1; 0.1; 0.01; 0.05; 0.02];  % For (x, y, theta, velocity, steering)
% std_control = [0.001; 0.0001];  % For (acceleration, steering rate)
X_obs1=[x0(1:5)];X_obs2=[x0(6:10)];
U_obs1=[];U_obs2=[];
x01=x0;
X01=[x01]; U01=[];
K=12; %it was 20 before
ocp_N=40;

%% Loop through each MPC iteration
for k = 1:num_iterations
    %% Solve MPC for both cars
    [X_star1, U_star1, u01, test_theta_full1, test_theta_full2] = OCP_solve(ocp, x0, initial_guess_belief_car2, W_xu11, car, testtheta1, testtheta2, D1, D2);
    X_sim_next1 = Sim_model(sim, u01, x0);
    X_sim_next1 = X_sim_next1(1:5); U_sim_next1 = u01(1:2);
    X_sim_next1_not_noisy=X_sim_next1; U_sim_next1_not_noisy=U_sim_next1;


    % Add noise
    X_noisy1 = X_sim_next1 + std_state .* randn(size(X_sim_next1));
    U_noisy1 = U_sim_next1 + std_control .* randn(size(U_sim_next1));
    X_sim_next1 = X_noisy1; U_sim_next1 = U_noisy1;
    

    %% For car 2 (perspective of car 1)
    [X_star2, U_star2, u02, test_theta_ego1, test_theta_ego2] = OCP_solve(ocp, x0, W_xu12, initial_guess_belief_car1, car, testtheta1, testtheta2, D1, D2);
    X_sim_next2 = Sim_model(sim, u02, x0);
    X_sim_next2 = X_sim_next2(6:end); U_sim_next2 = u02(3:4);
    X_sim_next2_not_noisy=X_sim_next2; U_sim_next2_not_noisy=U_sim_next2;


    % Add noise
    X_noisy2 = X_sim_next2 + std_state .* randn(size(X_sim_next2));
    U_noisy2 = U_sim_next2 + std_control .* randn(size(U_sim_next2));
    X_sim_next2 = X_noisy2; U_sim_next2 = U_noisy2;

    %% Gather observations
    X_obs1 = [X_obs1, X_sim_next1];
    X_obs2 = [X_obs2, X_sim_next2];
    U_obs1 = [U_obs1, U_sim_next1];
    U_obs2 = [U_obs2, U_sim_next2];

    %% Start particle filtering after K iterations
    if k >= K
        pf = 1;  % Particle filter for car 1

        % Take last K observations and set initial condition to the first observation in the window
        X_obs1_recent = X_obs1(:, end-K:end);
        U_obs1_recent = U_obs1(:, end-K+1:end);
        X_obs2_recent = X_obs2(:, end-K:end); % end - k + 1
        U_obs2_recent = U_obs2(:, end-K+1:end); 
        
        % Set initial condition to the most recent state
        x01 = X_obs1_recent(:,1);  % Initial condition for particle filter based on the first observation of the recent window
        x02 = X_obs2_recent(:,1);  % Initial condition for particle filter based on the first observation of the recent window
        X01=[x01;x02];

        [particles1c, estimated_W_xuOfcar2, ess1, likelihoods1c] = customparticleFilter(X_obs2_recent, U_obs2_recent, X01, ocp_N, customnum_particles, W_xu_ranges, W_xu11, ocp, initial_guess_belief_car2, range_factor, car, std_dev_custom, testtheta1, testtheta2, pf, D1, D2, K);

        pf = 0;  % Particle filter for car 2
        [particles2c, estimated_W_xu_Ofcar1, ess2, likelihoods2c] = customparticleFilter(X_obs1_recent, U_obs1_recent, X01, ocp_N, customnum_particles, W_xu_ranges, W_xu12, ocp, initial_guess_belief_car1, range_factor, car, std_dev_custom, testtheta1, testtheta2, pf, D1, D2, K);

        %% Update weights and ranges for the next iteration
        w_evol2 = [w_evol2, estimated_W_xuOfcar2.W_xu'];
        w_evol1 = [w_evol1, estimated_W_xu_Ofcar1.W_xu'];
        range_factor = max(0.05, range_factor * 0.990);  % Reduction strategy
        initial_guess_belief_car1 = estimated_W_xu_Ofcar1.W_xu';
        initial_guess_belief_car2 = estimated_W_xuOfcar2.W_xu';
    end

    %% Update state for next MPC iteration
x0=[X_sim_next1_not_noisy;X_sim_next2_not_noisy]; %%% updating for the next mpc iteration
    x_mpc = [x_mpc, x0];
    u0_mpc = [u01(1:2); u02(3:4)];
    u_mpc = [u_mpc, u0_mpc];
if car
    testtheta1=test_theta_full1; testtheta2=test_theta_ego2;
end

    waitbar(k / num_iterations, h, sprintf('Processing iteration %d of %d...', k, num_iterations));
end
%%
close(h);
close all
%% Figs
%% Fig x_mpc traj
plotRoundaboutWithLanes();
hold on;
plot(x_mpc(1,:),x_mpc(2,:),'r--',x_mpc(6,:),x_mpc(7,:),'b--')
grid on


save('w_evol_new','w_evol1','w_evol2');

return
%% Fig X_mpc traj + x_sim (groundtruth) Comparasion
%%%% Comparaision with the initial observation state
load('Sim_traj.mat');
% hold on
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
plot(1:length(w_evol1), w_evol1(1,:), 1:length(w_evol1), w_evol1(2,:))  
title(['Evolution for Estimation for weights of car 1 (W_{xu11}: [', W_xu11_str, '])'])
hold on

% Shaded area for Wx1
fill([1:length(w_evol1) fliplr(1:length(w_evol1))], ...
     [w_evol1(1,:) + w_std1(1,:) fliplr(w_evol1(1,:) - w_std1(1,:))], ...
     'b', 'FaceAlpha', 0.3, 'EdgeColor', 'none');
grid on
legend('Wx1', 'Wx2');

% Shaded area for Wu1
subplot(3,1,2)
plot(1:length(w_evol1), w_evol1(6,:))
hold on
fill([1:length(w_evol1) fliplr(1:length(w_evol1))], ...
     [w_evol1(6,:) + w_std1(6,:) fliplr(w_evol1(6,:) - w_std1(6,:))], ...
     'g', 'FaceAlpha', 0.3, 'EdgeColor', 'none');
grid on
legend('Wu1')

subplot(3,1,3)
plot(1:length(w_evol1), w_evol1(7,:))
hold on
% Shaded area for Wu2
fill([1:length(w_evol1) fliplr(1:length(w_evol1))], ...
     [w_evol1(7,:) + w_std1(7,:) fliplr(w_evol1(7,:) - w_std1(7,:))], ...
     'm', 'FaceAlpha', 0.3, 'EdgeColor', 'none');
legend('Wu2');
grid on

%% Fig w_evol2
figure()
% Initialization
w_mean2 = mean(w_evol2, 2);   % Mean of weights at each iteration
w_std2 = std(w_evol2, 0, 2);  % Standard deviation of weights at each iteration

subplot(3,1,1)
plot(1:length(w_evol1), w_evol2(1,:), 1:length(w_evol1), w_evol2(2,:))
title(['Evolution for Estimation for weights of car 2 (W_{xu12}: [', W_xu12_str, '])'])
hold on

% Shaded area for Wx1
fill([1:length(w_evol1) fliplr(1:length(w_evol1))], ...
     [w_evol2(1,:) + w_std2(1,:) fliplr(w_evol2(1,:) - w_std2(1,:))], ...
     'b', 'FaceAlpha', 0.3, 'EdgeColor', 'none');
grid on
legend('Wx1', 'Wx2');

% Shaded area for Wu1
subplot(3,1,2)
plot(1:length(w_evol1), w_evol2(6,:))
hold on
fill([1:length(w_evol1) fliplr(1:length(w_evol1))], ...
     [w_evol2(6,:) + w_std2(6,:) fliplr(w_evol2(6,:) - w_std2(6,:))], ...
     'g', 'FaceAlpha', 0.3, 'EdgeColor', 'none');
grid on
legend('Wu1')

subplot(3,1,3)
plot(1:length(w_evol1), w_evol2(7,:))
hold on
% Shaded area for Wu2
fill([1:length(w_evol1) fliplr(1:length(w_evol1))], ...
     [w_evol2(7,:) + w_std2(7,:) fliplr(w_evol2(7,:) - w_std2(7,:))], ...
     'm', 'FaceAlpha', 0.3, 'EdgeColor', 'none');
legend('Wu2');
grid on


 %% Figs u comparaision
 load('Sim_traj.mat');
 subplot(2,2,1)
  plot(1:length(u_mpc),u_mpc(1,1:length(u_mpc)),'b')
 title('Acceleration 1')
 grid on 
  subplot(2,2,2)
  plot(1:length(u_mpc),u_mpc(2,1:length(u_mpc)),'b')
  title('Steering Rate 1')
  grid on
  subplot(2,2,3)
  plot(1:length(u_mpc),u_mpc(3,1:length(u_mpc)),'b')
  title('Acceleration 2')
  grid on
  subplot(2,2,4)
  plot(1:length(u_mpc),u_mpc(4,1:length(u_mpc)),'b')
   title('Steering Rate 2')
   grid on
   %% Figs velocities
    subplot(2,1,1)
  plot(1:length(x_mpc),x_mpc(4,1:length(x_mpc)),'b')
 title('v 1')
 grid on 
 subplot(2,1,2)
  plot(1:length(x_mpc),x_mpc(9,1:length(x_mpc)),'b')
 title('v 2')
 grid on 
 %% Animation 
if 1


dsafe=30.0;
plotRoundaboutWithLanes();
%     axis equal;
    xlabel('x [m]');
    ylabel('y [m]');
    title('Trajectory');
    grid on;
    hold on;

    % Define the circle properties
    theta = linspace(0, 2*pi, 100);
    radius = sqrt(dsafe)/2; % radius is half of the diameter
    x_circle = radius * cos(theta);
    y_circle = radius * sin(theta);
    car1_traj = plot(x_mpc(1,1), x_mpc(2,1), 'r');
    car2_traj = plot(x_mpc(6,1), x_mpc(7,1), 'b');
        car1_circle = plot(x_mpc(1,1) + x_circle, x_mpc(2,1) + y_circle, 'r');
    car2_circle = plot(x_mpc(6,1) + x_circle, x_mpc(7,1) + y_circle, 'b');  
    for k = 2:length(x_mpc)
        % Plot the trajectories of the cars
        set(car1_traj, 'XData', x_mpc(1,1:k), 'YData', x_mpc(2,1:k));
        set(car2_traj, 'XData', x_mpc(6,1:k), 'YData', x_mpc(7,1:k));
        % Update the positions of the circles representing the vehicles
        set(car1_circle, 'XData', x_mpc(1,k) + x_circle, 'YData', x_mpc(2,k) + y_circle);
        set(car2_circle, 'XData', x_mpc(6,k) + x_circle, 'YData', x_mpc(7,k) + y_circle);
        pause(0.05); % Adjust the pause time to control the speed of the animation
    end
end


clc
clear all
close all
addpath("C:\Users\user\acados\examples\acados_matlab_octave")
%% Initial state
x0 = [0;-45;pi/2;0;0;-30;0;0;10;0];
% plotRoundaboutWithLanes();
D1=W2_newnew(x0);D2=W3_newnew(x0); %% referencing
%% Setting Up acados (OCP and Sim)
if 1
[ocp,sim]=customSettingUp(x0);
end
%% Preliminary Parameters for the particle filter.
num_particles = 250;
customnum_particles=150;
% Define the ranges for different components of W_x
W_xu_ranges = [
    0.1, 30;   % Range for x1
    0.1, 30;   % Range for y1
    1e-6, 1e-3;  % Range for theta1
    1e-6, 1e-3;  % Range for v1
    1e-6, 1e-3;  % Range for s1
    1e-3  5;
    1e-2  5  
];
global car 
car = 1; % car = 1 means the ego is car 1 (starting from down) 
if car
%     W_xu11 = [10;10;1e-4;1e-4;1e-4;0.1;1]; %weights car 1
%     W_xu12 = [10;10;1e-4;1e-4;1e-4;1;0.1]; %weights car 2
    W_xu11 = [10;10;1e-4;1e-4;1e-4;0.50;0.50]; %weights car 1
    W_xu12 = [10;10;1e-4;1e-4;1e-4;0.50;0.1]; %weights car 2
    % Convert weight vectors to string format
W_xu11_str = sprintf('%.1f, %.1f, %.1e, %.1e, %.1e, %.1f, %.1f', W_xu11);
W_xu12_str = sprintf('%.1f, %.1f, %.1e, %.1e, %.1e, %.1f, %.1f', W_xu12);

end
x_mpc=[x0];u_mpc=[];
x_mpc2 = x0(6:end); x_mpc1=x0(1:5);
range_factor = 1;  % Start with a larger factor
num_iterations= 250;
weight_history = zeros(num_iterations, num_particles);
% initial_guess_belief_car2 = [1; 1; 1e-2; 1e-2; 1e-5;3e-2;0.50];
% initial_guess_belief_car1 = [1; 1; 1e-2; 1e-2; 1e-5;3e-2;0.50];
initial_guess_belief_car2 = [1; 1; 1e-2; 1e-2; 1e-5;3e-2;0.10];
initial_guess_belief_car1 = [1; 1; 1e-2; 1e-2; 1e-5;3e-2;0.50];
% initial_guess_belief_car2 = W_xu1;
% initial_guess_belief_car1 = W_xu1;
ESS1=[]; ESS2=[];
w_evol1=[initial_guess_belief_car1];w_evol2=[initial_guess_belief_car2];
std_dev_custom=[5; 5; 1e-3; 1e-3; 1e-6; 0.2;0.2];
% std_dev_custom=[5; 5; 1e-3; 1e-3; 1e-6; 0.50;0.35];
ocp_N=20;
test_theta1=(pi/2)*ones(1,ocp_N+1);
test_theta2=(0)*ones(1, ocp_N + 1);
    testtheta1=test_theta1; testtheta2=test_theta2;
    h = waitbar(0, sprintf('Processing iteration 1 of %d...', num_iterations));
global std_state std_control
std_state = [0; 0; 0;0; 0];  % For (x, y, theta, velocity, steering)
std_control = [0; 0];  % For (acceleration, steering rate)
% std_state = [0.5; 0.5; 0.01; 0.05; 0.02];  % For (x, y, theta, velocity, steering)
% std_control = [0.5; 0.5];  % For (acceleration, steering rate)

% std_state = [0.1; 0.1; 0.01; 0.05; 0.02];  % For (x, y, theta, velocity, steering)
% std_control = [0.001; 0.0001];  % For (acceleration, steering rate)
X_obs1=[x0(1:5)];X_obs2=[x0(6:10)];
U_obs1=[];U_obs2=[];
x01=x0;
X01=[x01]; U01=[];
K=8; %it was 20 before
ocp_N=20;

%% Loop through each MPC iteration
for k = 1:num_iterations
    %% Solve MPC for both cars
    [X_star1, U_star1, u01, test_theta_full1, test_theta_full2] = OCP_solve(ocp, x0, initial_guess_belief_car2, W_xu11, car, testtheta1, testtheta2, D1, D2);
    X_sim_next1 = Sim_model(sim, u01, x0);
    X_sim_next1 = X_sim_next1(1:5); U_sim_next1 = u01(1:2);
    X_sim_next1_not_noisy=X_sim_next1; U_sim_next1_not_noisy=U_sim_next1;


    % Add noise
    X_noisy1 = X_sim_next1 + std_state .* randn(size(X_sim_next1));
    U_noisy1 = U_sim_next1 + std_control .* randn(size(U_sim_next1));
    X_sim_next1 = X_noisy1; U_sim_next1 = U_noisy1;
    

    %% For car 2 (perspective of car 1)
    [X_star2, U_star2, u02, test_theta_ego1, test_theta_ego2] = OCP_solve(ocp, x0, W_xu12, initial_guess_belief_car1, car, testtheta1, testtheta2, D1, D2);
    X_sim_next2 = Sim_model(sim, u02, x0);
    X_sim_next2 = X_sim_next2(6:end); U_sim_next2 = u02(3:4);
    X_sim_next2_not_noisy=X_sim_next2; U_sim_next2_not_noisy=U_sim_next2;


    % Add noise
    X_noisy2 = X_sim_next2 + std_state .* randn(size(X_sim_next2));
    U_noisy2 = U_sim_next2 + std_control .* randn(size(U_sim_next2));
    X_sim_next2 = X_noisy2; U_sim_next2 = U_noisy2;

    %% Gather observations
    X_obs1 = [X_obs1, X_sim_next1];
    X_obs2 = [X_obs2, X_sim_next2];
    U_obs1 = [U_obs1, U_sim_next1];
    U_obs2 = [U_obs2, U_sim_next2];

    %% Start particle filtering after K iterations
    if k >= K

        % Take last K observations and set initial condition to the first observation in the window
        X_obs1_recent = X_obs1(:, end-K:end);
        U_obs1_recent = U_obs1(:, end-K+1:end);
        X_obs2_recent = X_obs2(:, end-K:end); % end - k + 1
        U_obs2_recent = U_obs2(:, end-K+1:end); 
        
        % Set initial condition to the most recent state
        x01 = X_obs1_recent(:,1);  % Initial condition for particle filter based on the first observation of the recent window
        x02 = X_obs2_recent(:,1);  % Initial condition for particle filter based on the first observation of the recent window
        X01=[x01;x02];
        pf = 1;  % Particle filter for car 1
        [particles1c, estimated_W_xuOfcar2, ess1, likelihoods1c] = customparticleFilter(X_obs2_recent, U_obs2_recent, X01, ocp_N, customnum_particles, W_xu_ranges, W_xu11, ocp, initial_guess_belief_car2, range_factor, car, std_dev_custom, testtheta1, testtheta2, pf, D1, D2, K,k);
        pf = 0;  % Particle filter for car 2
        [particles2c, estimated_W_xu_Ofcar1, ess2, likelihoods2c] = customparticleFilter(X_obs1_recent, U_obs1_recent, X01, ocp_N, customnum_particles, W_xu_ranges, W_xu12, ocp, initial_guess_belief_car1, range_factor, car, std_dev_custom, testtheta1, testtheta2, pf, D1, D2, K,k);

        %% Update weights and ranges for the next iteration
        w_evol2 = [w_evol2, estimated_W_xuOfcar2.W_xu'];
        w_evol1 = [w_evol1, estimated_W_xu_Ofcar1.W_xu'];
        range_factor = max(0.05, range_factor * 0.990);  % Reduction strategy
        initial_guess_belief_car1 = estimated_W_xu_Ofcar1.W_xu';
        initial_guess_belief_car2 = estimated_W_xuOfcar2.W_xu';
    end

    %% Update state for next MPC iteration
x0=[X_sim_next1_not_noisy;X_sim_next2_not_noisy]; %%% updating for the next mpc iteration
    x_mpc = [x_mpc, x0];
    u0_mpc = [u01(1:2); u02(3:4)];
    u_mpc = [u_mpc, u0_mpc];
if car
    testtheta1=test_theta_full1; testtheta2=test_theta_ego2;
end

    waitbar(k / num_iterations, h, sprintf('Processing iteration %d of %d...', k, num_iterations));
end
close(h);
close all
%% Figs
%% Fig x_mpc traj
plotRoundaboutWithLanes();
hold on;
plot(x_mpc(1,:),x_mpc(2,:),'r--',x_mpc(6,:),x_mpc(7,:),'b--')
grid on


save('w_evol_new','w_evol1','w_evol2');

return
%% Fig X_mpc traj + x_sim (groundtruth) Comparasion
%%%% Comparaision with the initial observation state
load('Sim_traj.mat');
% hold on
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
plot(1:length(w_evol1), w_evol1(1,:), 1:length(w_evol1), w_evol1(2,:))  
title(['Evolution for Estimation for weights of car 1 (W_{xu11}: [', W_xu11_str, '])'])
hold on

% Shaded area for Wx1
fill([1:length(w_evol1) fliplr(1:length(w_evol1))], ...
     [w_evol1(1,:) + w_std1(1,:) fliplr(w_evol1(1,:) - w_std1(1,:))], ...
     'b', 'FaceAlpha', 0.3, 'EdgeColor', 'none');
grid on
legend('Wx1', 'Wx2');

% Shaded area for Wu1
subplot(3,1,2)
plot(1:length(w_evol1), w_evol1(6,:))
hold on
fill([1:length(w_evol1) fliplr(1:length(w_evol1))], ...
     [w_evol1(6,:) + w_std1(6,:) fliplr(w_evol1(6,:) - w_std1(6,:))], ...
     'g', 'FaceAlpha', 0.3, 'EdgeColor', 'none');
grid on
legend('Wu1')

subplot(3,1,3)
plot(1:length(w_evol1), w_evol1(7,:))
hold on
% Shaded area for Wu2
fill([1:length(w_evol1) fliplr(1:length(w_evol1))], ...
     [w_evol1(7,:) + w_std1(7,:) fliplr(w_evol1(7,:) - w_std1(7,:))], ...
     'm', 'FaceAlpha', 0.3, 'EdgeColor', 'none');
legend('Wu2');
grid on

%% Fig w_evol2
figure()
% Initialization
w_mean2 = mean(w_evol2, 2);   % Mean of weights at each iteration
w_std2 = std(w_evol2, 0, 2);  % Standard deviation of weights at each iteration

subplot(3,1,1)
plot(1:length(w_evol1), w_evol2(1,:), 1:length(w_evol1), w_evol2(2,:))
title(['Evolution for Estimation for weights of car 2 (W_{xu12}: [', W_xu12_str, '])'])
hold on

% Shaded area for Wx1
fill([1:length(w_evol1) fliplr(1:length(w_evol1))], ...
     [w_evol2(1,:) + w_std2(1,:) fliplr(w_evol2(1,:) - w_std2(1,:))], ...
     'b', 'FaceAlpha', 0.3, 'EdgeColor', 'none');
grid on
legend('Wx1', 'Wx2');

% Shaded area for Wu1
subplot(3,1,2)
plot(1:length(w_evol1), w_evol2(6,:))
hold on
fill([1:length(w_evol1) fliplr(1:length(w_evol1))], ...
     [w_evol2(6,:) + w_std2(6,:) fliplr(w_evol2(6,:) - w_std2(6,:))], ...
     'g', 'FaceAlpha', 0.3, 'EdgeColor', 'none');
grid on
legend('Wu1')

subplot(3,1,3)
plot(1:length(w_evol1), w_evol2(7,:))
hold on
% Shaded area for Wu2
fill([1:length(w_evol1) fliplr(1:length(w_evol1))], ...
     [w_evol2(7,:) + w_std2(7,:) fliplr(w_evol2(7,:) - w_std2(7,:))], ...
     'm', 'FaceAlpha', 0.3, 'EdgeColor', 'none');
legend('Wu2');
grid on


 %% Figs u comparaision
 load('Sim_traj.mat');
 subplot(2,2,1)
  plot(1:length(u_mpc),u_mpc(1,1:length(u_mpc)),'b')
 title('Acceleration 1')
 grid on 
  subplot(2,2,2)
  plot(1:length(u_mpc),u_mpc(2,1:length(u_mpc)),'b')
  title('Steering Rate 1')
  grid on
  subplot(2,2,3)
  plot(1:length(u_mpc),u_mpc(3,1:length(u_mpc)),'b')
  title('Acceleration 2')
  grid on
  subplot(2,2,4)
  plot(1:length(u_mpc),u_mpc(4,1:length(u_mpc)),'b')
   title('Steering Rate 2')
   grid on
   %% Figs velocities
    subplot(2,1,1)
  plot(1:length(x_mpc),x_mpc(4,1:length(x_mpc)),'b')
 title('v 1')
 grid on 
 subplot(2,1,2)
  plot(1:length(x_mpc),x_mpc(9,1:length(x_mpc)),'b')
 title('v 2')
 grid on 
 %% Animation 
if 1


dsafe=2.0;
plotRoundaboutWithLanes();
%     axis equal;
    xlabel('x [m]');
    ylabel('y [m]');
    title('Trajectory');
    grid on;
    hold on;

    % Define the circle properties
    theta = linspace(0, 2*pi, 100);
    radius = sqrt(dsafe)/2; % radius is half of the diameter
    x_circle = radius * cos(theta);
    y_circle = radius * sin(theta);
    car1_traj = plot(x_mpc(1,1), x_mpc(2,1), 'r');
    car2_traj = plot(x_mpc(6,1), x_mpc(7,1), 'b');
        car1_circle = plot(x_mpc(1,1) + x_circle, x_mpc(2,1) + y_circle, 'r');
    car2_circle = plot(x_mpc(6,1) + x_circle, x_mpc(7,1) + y_circle, 'b');  
    for k = 2:length(x_mpc)
        % Plot the trajectories of the cars
        set(car1_traj, 'XData', x_mpc(1,1:k), 'YData', x_mpc(2,1:k));
        set(car2_traj, 'XData', x_mpc(6,1:k), 'YData', x_mpc(7,1:k));
        % Update the positions of the circles representing the vehicles
        set(car1_circle, 'XData', x_mpc(1,k) + x_circle, 'YData', x_mpc(2,k) + y_circle);
        set(car2_circle, 'XData', x_mpc(6,k) + x_circle, 'YData', x_mpc(7,k) + y_circle);
        pause(0.05); % Adjust the pause time to control the speed of the animation
    end
end


