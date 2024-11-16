% Initialize a new figure for animation
close all
clc
figure;
hold on;
axis equal;
grid on;
xlim([-10 30]);
ylim([-10 30]);

% Plot the reference goal points for both cars
plot(ref_state1(1), ref_state1(2), 'ro', 'MarkerSize', 10, 'LineWidth', 2); % Goal for car 1
plot(ref_state2(1), ref_state2(2), 'bo', 'MarkerSize', 10, 'LineWidth', 2); % Goal for car 2

% Simulation parameters
state1 = [0; 0; 0; 0; 0]; % Start position for car 1
state2 = [20; 0; 0; 0; 0]; % Start position for car 2
car_length = 2.5; % For plotting the car

% Number of time steps to simulate
num_steps = 500;

for step = 1:num_steps
    % Discretize the current states to find the best actions from Q-table
    [x_idx1, y_idx1, theta_idx1, v_idx1, delta_idx1] = discretize_state(state1, num_x_states, num_y_states, num_theta_states, num_v_states, num_delta_states);
    [x_idx2, y_idx2, theta_idx2, v_idx2, delta_idx2] = discretize_state(state2, num_x_states, num_y_states, num_theta_states, num_v_states, num_delta_states);
    
    % Get the best action (exploitation) for car 1 from the learned Q-table
    q_values1 = reshape(Q_table(x_idx1, y_idx1, theta_idx1, v_idx1, delta_idx1, :, :), [length(acc_actions), length(steer_actions)]);
    [~, max_idx1] = max(q_values1(:)); % Get the index of the maximum Q-value
    [acc_idx1, steer_idx1] = ind2sub(size(q_values1), max_idx1); % Convert linear index to 2D subscripts
    u1 = [acc_actions(acc_idx1); steer_actions(steer_idx1)]; % Chosen action (acceleration, steering rate) for car 1

    % Get the best action (exploitation) for car 2 from the learned Q-table
    q_values2 = reshape(Q_table(x_idx2, y_idx2, theta_idx2, v_idx2, delta_idx2, :, :), [length(acc_actions), length(steer_actions)]);
    [~, max_idx2] = max(q_values2(:)); % Get the index of the maximum Q-value
    [acc_idx2, steer_idx2] = ind2sub(size(q_values2), max_idx2); % Convert linear index to 2D subscripts
    u2 = [acc_actions(acc_idx2); steer_actions(steer_idx2)]; % Chosen action (acceleration, steering rate) for car 2

    % Simulate the next states using the kinematic model
    next_state1 = kinematic_model(state1, u1, L, dt);
    next_state2 = kinematic_model(state2, u2, L, dt);
    
    % Clear the previous cars' drawings
    clf;
    hold on;
    axis equal;
    grid on;
    xlim([-10 30]);
    ylim([-10 30]);

    % Replot the reference goal points for both cars
    plot(ref_state1(1), ref_state1(2), 'bo', 'MarkerSize', 10, 'LineWidth', 2); % Goal for car 1
    plot(ref_state2(1), ref_state2(2), 'ro', 'MarkerSize', 10, 'LineWidth', 2); % Goal for car 2
    
    % Plot the car at the current state for both cars
    plot_car(state1, car_length,1); % Car 1 in blue
    plot_car(state2, car_length,0); % Car 2 in red

    % Update the states
    state1 = next_state1;
    state2 = next_state2;
    
    % Pause to create animation effect
    pause(0.1); % Adjust pause time for desired speed of animation
step
    % Check if either car has reached its goal
    if check_goal(state1, ref_state1, 0.5)
        disp('Car 1 reached its goal!');
        break;
    end
    if check_goal(state2, ref_state2, 0.5)
        disp('Car 2 reached its goal!');
        break;
    end
end
