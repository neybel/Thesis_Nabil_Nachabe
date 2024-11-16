clc
clear all
% clear mex
close all
%% Initial state
x0 = [0;-45;pi/2;0;0;-30;0;0;10;0];
x01=x0(1:5);x02=x0(6:10);
D1=W2_newnew(x0);D2=W3_newnew(x0); %% referencing
%% Setting Up acados (OCP and Sim)
% if 1
% [ocp,sim]=customSettingUp(x0);
% end
%%
%% Preliminary Parameters for the particle filter.
num_particles = 75;
W_xu_ranges = [
    0.1, 200;   % Range for x1
    0.1, 200;   % Range for y1
    1e-6, 1e-3;  % Range for theta1
    1e-6, 1e-3;  % Range for v1
    1e-6, 1e-3;  % Range for s1
    1e-3  10;
    1e-2  10  
];
% initial_guess_belief_car2 = [1; 1; 1e-2; 1e-2; 1e-5;0.15;0.01];
% initial_guess_belief_car1 = [1; 1; 1e-2; 1e-2; 1e-5;0.15;0.1];
initial_guess_belief_car2 = [1; 1; 1e-2; 1e-2; 1e-5;0.15;0.01];
initial_guess_belief_car1 = [2.5; 2.5; 1e-2; 1e-2; 1e-5;0.15;0.1];
% W_xu1 = [1e1;1e1;1e-4;1e-4;1e-4;1e-2;1]; Ground truth
std_dev=[2.5; 2.5; 1e-3; 1e-3; 1e-6; 0.1;0.1];
% std_dev1=std_dev;std_dev2=std_dev;
std_dev1=std_dev;
std_dev2=[1.5; 1.5; 1e-3; 1e-3; 1e-6; 0.2;0.2];
W_xucar1 = [10;10;1e-4;1e-4;1e-4;0.50;0.1]; % Weights of car 1
W_xucar2 = [10;10;1e-4;1e-4;1e-4;0.50;0.1]; % Weights of car 2
return
%% Compile S functions of acados (ocp and sim)
% cd c_generated_code
if 0
make_sfun_sim; % integrator
make_sfun; % ocp solver

%%% Trying with this
make_sfun_sim_sim_model;

%% Copy Simulink example block into c_generated_code
source_folder = fullfile(pwd, '..');
target_folder = pwd;
copyfile( fullfile(source_folder, 'Simulink_test_SFunction.slx'), target_folder );
%% Open Simulink example block
open_system(fullfile(target_folder, 'Simulink_test_SFunction.slx'))
end
%% Run the Simulink model
try
    tic
    out = sim('PF_Simulink_TEST.slx');
    toc
%     cd ..
catch
    cd ..
    error('Simulink advanced closed loop example failed')
end
%% Plotting after running
X_sim1=out.x1.signals.values(:,1);Y_sim1=out.x1.signals.values(:,2);
X_sim2=out.x2.signals.values(:,1);Y_sim2=out.x2.signals.values(:,2);
plotRoundaboutWithLanes();
hold on 
plot(X_sim1,Y_sim1,'r*',X_sim2,Y_sim2,'bo');
%% Evolution from car 1 perspective
figure(1)
W_xy1=out.Estimation_car2(:,1); iterations=size(W_xy1,1);
W_a=out.Estimation_car2(:,6);W_s=out.Estimation_car2(:,7);
subplot(3,1,1)
plot(1:iterations,W_xy1);
xlabel('# iteration');ylabel('Weight magnitude');legend('W_x');
title('Evolution of weights estimation from car 1 perspective')
grid on;grid minor;
subplot(3,1,2)
plot(1:iterations,W_a);
xlabel('# iteration');ylabel('Weight magnitude');legend('W_a')
grid on; grid minor;
subplot(3,1,3)
plot(1:iterations,W_s)
xlabel('# iteration');ylabel('Weight magnitude');legend('W_s')
grid on; grid minor;
saveas(gcf, 'EvolCar1.pdf','pdf');
%% Evolution from car 2 perspective
figure (2)
W_xy2=out.Estimation_car1(:,1); iterations=size(W_xy2,1);
W_a=out.Estimation_car1(:,6);W_s=out.Estimation_car1(:,7);
subplot(3,1,1)
plot(1:iterations,W_xy2);
xlabel('# iteration');ylabel('Weight magnitude');legend('W_x');
title('Evolution of weights estimation from car 2 perspective')
grid on;grid minor;
subplot(3,1,2)
plot(1:iterations,W_a);
xlabel('# iteration');ylabel('Weight magnitude');legend('W_a')
grid on; grid minor;
subplot(3,1,3)
plot(1:iterations,W_s)
xlabel('# iteration');ylabel('Weight magnitude');legend('W_s')
grid on; grid minor;
saveas(gcf, 'EvolCar2.pdf','pdf');
%% Animation
if 1
dsafe=12.50;
plotRoundaboutWithLanes();
%     axis equal;
    xlabel('x [m]');
    ylabel('y [m]');
    title('Trajectory');
    grid on;
    grid minor
    hold on;

    % Define the circle properties
    theta = linspace(0, 2*pi, 100);
    radius = sqrt(dsafe/2); % radius is half of the diameter
    x_circle = radius * cos(theta);
    y_circle = radius * sin(theta);

    % Initialize the plot for the car trajectory
    car1_traj = plot(X_sim1(1), Y_sim1(1), 'r');
    car2_traj = plot(X_sim2(1), Y_sim2(1), 'b');
    
    % Initialize the plot for the circle representing each vehicle
    car1_circle = plot(X_sim1(1) + x_circle, Y_sim1(1) + y_circle, 'r');
    car2_circle = plot(X_sim2(1) + x_circle, Y_sim2(1) + y_circle, 'b');
    
    % Set up the axis limits based on the initial position and range of motion
%     xlim([min(min(x_mpc(1,:)), min(x_mpc(6,:)))-5, max(max(x_mpc(1,:)), max(x_mpc(6,:)))+5 ]);
%     ylim([min(min(x_mpc(2,:)), min(x_mpc(7,:)))-10 , max(max(x_mpc(2,:)), max(x_mpc(7,:)))+10 ]);
%     axis equal;
%     
    % Animation loop
    for k = 2:length(X_sim1)
        % Plot the trajectories of the cars
        set(car1_traj, 'XData', X_sim1(1:k), 'YData', Y_sim1(1:k));
        set(car2_traj, 'XData', X_sim2(1:k), 'YData', Y_sim2(1:k));
        
        % Update the positions of the circles representing the vehicles
        set(car1_circle, 'XData', X_sim1(k) + x_circle, 'YData', Y_sim1(k) + y_circle);
        set(car2_circle, 'XData', X_sim2(k) + x_circle, 'YData', Y_sim2(k) + y_circle);

        % Pause to create animation effect
        pause(0.05); % Adjust the pause time to control the speed of the animation
    end
end




