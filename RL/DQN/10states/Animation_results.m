% Define start and goal points for car 1 and car 2
start_state1 = [0; 0; 0; 0; 0]; % Starting state for car 1 [x, y, theta, velocity, steering]
goal_point1 = [10, 10]; % Goal position for car 1

start_state2 = [20; 0; 0; 0; 0]; % Starting state for car 2 [x, y, theta, velocity, steering]
goal_point2 = [10, 20]; % Goal position for car 2

% Parameters for the animation
animation_steps = max_steps; % Use the same number of steps as in the training
dt = 0.1; % Time step for animation

% Create the figure for animation
figure;
hold on;

% Plot start and goal points
plot(start_state1(1), start_state1(2), 'go', 'MarkerSize', 10, 'DisplayName', 'Start Point 1');
plot(goal_point1(1), goal_point1(2), 'r*', 'MarkerSize', 10, 'DisplayName', 'Goal Point 1');

plot(start_state2(1), start_state2(2), 'bo', 'MarkerSize', 10, 'DisplayName', 'Start Point 2');
plot(goal_point2(1), goal_point2(2), 'm*', 'MarkerSize', 10, 'DisplayName', 'Goal Point 2');

% Create line objects for cars
car1_line = plot(nan, nan, 'r-', 'LineWidth', 2, 'DisplayName', 'Car 1');
car2_line = plot(nan, nan, 'b-', 'LineWidth', 2, 'DisplayName', 'Car 2');

legend('Location', 'Best');
xlim([-10, 30]); % Set x-axis limits
ylim([-10, 30]); % Set y-axis limits
xlabel('X Position');
ylabel('Y Position');
title('Vehicle Animation');

% Initialize the state for both cars
state1 = start_state1;
state2 = start_state2;

% Store previous positions to manage overlap
car1_positions = [];
car2_positions = [];

for step = 1:animation_steps
    % Select actions for both cars using the trained policy
    action1 = epsilon_greedy_policy(state1, 0, q_network, acc_actions, steer_actions);
    action2 = epsilon_greedy_policy(state2, 0, q_network, acc_actions, steer_actions);
    
    % Convert action indices to actual actions
    [acc_idx1, steer_idx1] = ind2sub([length(acc_actions), length(steer_actions)], action1);
    actual_action1 = [acc_actions(acc_idx1), steer_actions(steer_idx1)];

    [acc_idx2, steer_idx2] = ind2sub([length(acc_actions), length(steer_actions)], action2);
    actual_action2 = [acc_actions(acc_idx2), steer_actions(steer_idx2)];

    % Apply actions and get next states
    [next_state1, ~] = environment_step(state1, actual_action1, car_length, dt, [10; 0; 0; 0; 0], state2, state_dim);
    [next_state2, ~] = environment_step(state2, actual_action2, car_length, dt, [0; 10; 0; 0; 0], state1, state_dim);

    % Append new positions
    car1_positions = [car1_positions; state1(1:2)'];
    car2_positions = [car2_positions; state2(1:2)'];
    
    % Update car positions in the animation
    set(car1_line, 'XData', car1_positions(:,1));
    set(car1_line, 'YData', car1_positions(:,2));
    
    set(car2_line, 'XData', car2_positions(:,1));
    set(car2_line, 'YData', car2_positions(:,2));
    
    % Pause to create animation effect
    pause(dt);

    % Update states
    state1 = next_state1;
    state2 = next_state2;
end

hold off;
