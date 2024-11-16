% Initialize a new figure for animation
close all
clc
figure;
hold on;
axis equal;
grid on; grid minor;
xlim([-10 20]);
ylim([-10 20]);

% Plot the reference goal point
plot(ref_state(1), ref_state(2), 'ro', 'MarkerSize', 10, 'LineWidth', 2); % Goal (ref_state)

% Simulation parameters
state = [0; 0; 0; 0; 0]; % Start at the initial state
car_length = 2.5; % For plotting the car

% Number of time steps to simulate
num_steps = 250;

for step = 1:num_steps
    % Discretize the current state to find the best action from Q-table
    [x_idx, y_idx, theta_idx, v_idx, delta_idx] = discretize_state(state, num_x_states, num_y_states, num_theta_states, num_v_states, num_delta_states);
    
    % Get the best action (exploitation) from the learned Q-table
    q_values = reshape(Q_table(x_idx, y_idx, theta_idx, v_idx, delta_idx, :, :), [length(acc_actions), length(steer_actions)]);
    [~, max_idx] = max(q_values(:)); % Get the index of the maximum Q-value
    [acc_idx, steer_idx] = ind2sub(size(q_values), max_idx); % Convert linear index to 2D subscripts
    u = [acc_actions(acc_idx); steer_actions(steer_idx)]; % Chosen action (acceleration, steering rate)
    
    % Simulate the next state using the kinematic model
    next_state = kinematic_model(state, u, L, dt);
    
    % Plot the car at the current state
    plot_car(state, car_length); % Custom function to draw the car

    % Update the state
    state = next_state;
    
    % Pause to create animation effect
    pause(0.1); % Adjust pause time for desired speed of animation
end