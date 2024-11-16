% Parameters for the vehicles
clc
clearvars
close all
L = 2.5;  % Wheelbase (m)
dt = 0.1; % Time step (s)
Q_xy = 1e3; Q_theta = 1e-5; Q_v = 1e0; R_a = 1e-5; R_delta = 1e-5;
collision_threshold = 1.0;  % Minimum distance between cars to avoid collision
collision_penalty = 1e1;    % High penalty for collision

% Discretization parameters
num_x_states = 80;
num_y_states = 80;
num_theta_states = 15;
num_v_states = 15;
num_delta_states = 3;

% Action space (discrete values for acceleration and steering rate)
acc_actions = linspace(-11.5, +11.5, 20);    % 25 possible actions for acceleration
steer_actions = linspace(-1, 1, 20); % 25 possible actions for steering rate

% Initialize the Q-table
Q_table = rand(num_x_states, num_y_states, num_theta_states, num_v_states, num_delta_states, ...
               length(acc_actions), length(steer_actions));

% Q_table = sparse(num_x_states * num_y_states * num_theta_states * num_v_states * num_delta_states, length(acc_actions) * length(steer_actions));


% Learning parameters
alpha = 0.25;  % Learning rate
gamma = 0.90;  % Discount factor
epsilon = 0.1; % Exploration rate

% Reference states for both cars (goals)
ref_state1 = [10; 10; pi/2; 0; 0]; % [x, y, theta, v]
ref_state2 = [0; 20; pi/2; 0; 0];  % Different goal for car 2

% Total episodes for training
total_episodes = 9e4;
hWaitBar = waitbar(0, 'Training Progress', 'Name', 'Q-learning Training');

for episode = 1:total_episodes
    % Initialize both cars' states and integral error
    state1 = [0; 0; 0; 0; 0]; % Start position for car 1
    state2 = [20; 0; 0; 0; 0]; % Start position for car 2
    for step = 1:500
        % Discretize the current states of both cars
        [x_idx1, y_idx1, theta_idx1, v_idx1, delta_idx1] = discretize_state(state1, num_x_states, num_y_states, num_theta_states, num_v_states, num_delta_states);
        [x_idx2, y_idx2, theta_idx2, v_idx2, delta_idx2] = discretize_state(state2, num_x_states, num_y_states, num_theta_states, num_v_states, num_delta_states);

        % Epsilon-greedy action selection for car 1
        if rand < epsilon
            % Exploration: select random actions
            acc_idx1 = randi(length(acc_actions)); % Random index for acceleration
            steer_idx1 = randi(length(steer_actions)); % Random index for steering rate
        else
            % Exploitation: select best actions based on Q-table
            q_values1 = reshape(Q_table(x_idx1, y_idx1, theta_idx1, v_idx1, delta_idx1, :, :), [length(acc_actions), length(steer_actions)]);
            [~, max_idx1] = max(q_values1(:)); % Get the index of the maximum Q-value
            [acc_idx1, steer_idx1] = ind2sub(size(q_values1), max_idx1); % Convert linear index to 2D subscripts
        end

        % Epsilon-greedy action selection for car 2
        if rand < epsilon
            % Exploration: select random actions
            acc_idx2 = randi(length(acc_actions)); % Random index for acceleration
            steer_idx2 = randi(length(steer_actions)); % Random index for steering rate
        else
            % Exploitation: select best actions based on Q-table
            q_values2 = reshape(Q_table(x_idx2, y_idx2, theta_idx2, v_idx2, delta_idx2, :, :), [length(acc_actions), length(steer_actions)]);
            [~, max_idx2] = max(q_values2(:)); % Get the index of the maximum Q-value
            [acc_idx2, steer_idx2] = ind2sub(size(q_values2), max_idx2); % Convert linear index to 2D subscripts
        end

        % Get the chosen actions (acceleration and steering rate) for both cars
        u1 = [acc_actions(acc_idx1); steer_actions(steer_idx1)];
        u2 = [acc_actions(acc_idx2); steer_actions(steer_idx2)];

        % Apply the action to get the next states using the kinematic model
        next_state1 = kinematic_model(state1, u1, L, dt);
        next_state2 = kinematic_model(state2, u2, L, dt);

        % Calculate the reward (negative cost) including the collision penalty
        reward = -cost_function(next_state1, next_state2, ref_state1, ref_state2, u1, u2, Q_xy, Q_theta, Q_v, R_a, R_delta, collision_threshold, collision_penalty);

        % Discretize next states
        [next_x_idx1, next_y_idx1, next_theta_idx1, next_v_idx1, next_delta_idx1] = discretize_state(next_state1, num_x_states, num_y_states, num_theta_states, num_v_states, num_delta_states);
        [next_x_idx2, next_y_idx2, next_theta_idx2, next_v_idx2, next_delta_idx2] = discretize_state(next_state2, num_x_states, num_y_states, num_theta_states, num_v_states, num_delta_states);

        % Q-learning update for car 1
        best_future_q1 = max(max(Q_table(next_x_idx1, next_y_idx1, next_theta_idx1, next_v_idx1, next_delta_idx1, :, :), [], 'all'));
        current_q1 = Q_table(x_idx1, y_idx1, theta_idx1, v_idx1, delta_idx1, acc_idx1, steer_idx1);
        Q_table(x_idx1, y_idx1, theta_idx1, v_idx1, delta_idx1, acc_idx1, steer_idx1) = current_q1 + alpha * (reward + gamma * best_future_q1 - current_q1);

        % Q-learning update for car 2
        best_future_q2 = max(max(Q_table(next_x_idx2, next_y_idx2, next_theta_idx2, next_v_idx2, next_delta_idx2, :, :), [], 'all'));
        current_q2 = Q_table(x_idx2, y_idx2, theta_idx2, v_idx2, delta_idx2, acc_idx2, steer_idx2);
        Q_table(x_idx2, y_idx2, theta_idx2, v_idx2, delta_idx2, acc_idx2, steer_idx2) = current_q2 + alpha * (reward + gamma * best_future_q2 - current_q2);

        % Update the states for the next step
        state1 = next_state1;
        state2 = next_state2;

        % Check if either car reached its goal
        if check_goal(state1, ref_state1, 0.5)
            state1(4) = 0; % Stop car 1 by setting velocity to 0
        end
        if check_goal(state2, ref_state2, 0.5)
            state2(4) = 0; % Stop car 2 by setting velocity to 0
        end
    end

    % Decay exploration rate epsilon
    epsilon = epsilon * 0.99;

    % Update the progress bar
    progress = episode / total_episodes;
    waitbar(progress, hWaitBar, sprintf('Episode %d of %d (%.2f%% complete)', episode, total_episodes, progress * 100));
end

% Close the progress bar
close(hWaitBar);
