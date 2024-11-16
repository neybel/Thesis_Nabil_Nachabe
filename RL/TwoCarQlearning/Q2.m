% Parameters for the vehicles
clc
clearvars
close all
L = 2.5;  % Wheelbase (m)
dt = 0.1; % Time step (s)
Q_xy = 1e3; Q_theta = 1e-5; Q_v = 1e0; R_a = 1e-5; R_delta = 1e-5;
collision_threshold = 0.50;  % Minimum distance between cars to avoid collision 1.0
collision_penalty = 1e1;    % High penalty for collision 134

% Discretization parameters
num_x_states = 80;
num_y_states = 80;
num_theta_states = 30;
num_v_states = 30;
num_delta_states = 3;

% Action space (discrete values for acceleration and steering rate)
acc_actions = linspace(-11.5, +11.5, 10);    % 30 possible actions for acceleration 30.0
steer_actions = linspace(-1, 1, 10); % 30 possible actions for steering rate 30.0F

% Initialize the Q-table as a sparse matrix
Q_table = sparse(num_x_states * num_y_states * num_theta_states * num_v_states * num_delta_states, length(acc_actions) * length(steer_actions));

% Learning parameters
alpha = 0.1;  % Learning rate 0.25
gamma = 0.90;  % Discount factor
epsilon = 0.3; % Exploration rate 0.1

% Reference states for both cars (goals)
ref_state1 = [10; 10; pi/2; 0; 0]; % [x, y, theta, v]
ref_state2 = [0; 20; pi/2; 0; 0];  % Different goal for car 2

% Total episodes for training
total_episodes = 5e2;
hWaitBar = waitbar(0, 'Training Progress', 'Name', 'Q-learning Training');

for episode = 1:total_episodes
    % Initialize both cars' states and integral error
    state1 = [0; 0; 0; 0; 0]; % Start position for car 1
    state2 = [20; 0; 0; 0; 0]; % Start position for car 2
    for step = 1:100
        % Discretize the current states of both cars
        [x_idx1, y_idx1, theta_idx1, v_idx1, delta_idx1] = discretize_state(state1, num_x_states, num_y_states, num_theta_states, num_v_states, num_delta_states);
        [x_idx2, y_idx2, theta_idx2, v_idx2, delta_idx2] = discretize_state(state2, num_x_states, num_y_states, num_theta_states, num_v_states, num_delta_states);

        % Convert state indices into a single linear index
        state_idx1 = sub2ind([num_x_states, num_y_states, num_theta_states, num_v_states, num_delta_states], x_idx1, y_idx1, theta_idx1, v_idx1, delta_idx1);
        state_idx2 = sub2ind([num_x_states, num_y_states, num_theta_states, num_v_states, num_delta_states], x_idx2, y_idx2, theta_idx2, v_idx2, delta_idx2);

        % Epsilon-greedy action selection for car 1
        if rand < epsilon
            % Exploration: select random actions
            acc_idx1 = randi(length(acc_actions)); % Random index for acceleration
            steer_idx1 = randi(length(steer_actions)); % Random index for steering rate
        else
            % Exploitation: select best actions based on Q-table
            q_values1 = Q_table(state_idx1, :);
            q_values1 = reshape(q_values1, [length(acc_actions), length(steer_actions)]); % Reshape to 2D matrix for actions
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
            q_values2 = Q_table(state_idx2, :);
            q_values2 = reshape(q_values2, [length(acc_actions), length(steer_actions)]); % Reshape to 2D matrix for actions
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

        % Convert next state indices into a single linear index
        next_state_idx1 = sub2ind([num_x_states, num_y_states, num_theta_states, num_v_states, num_delta_states], next_x_idx1, next_y_idx1, next_theta_idx1, next_v_idx1, next_delta_idx1);
        next_state_idx2 = sub2ind([num_x_states, num_y_states, num_theta_states, num_v_states, num_delta_states], next_x_idx2, next_y_idx2, next_theta_idx2, next_v_idx2, next_delta_idx2);

        % Q-learning update for car 1
        best_future_q1 = max(Q_table(next_state_idx1, :), [], 'all');
        current_q1 = Q_table(state_idx1, sub2ind([length(acc_actions), length(steer_actions)], acc_idx1, steer_idx1));
        Q_table(state_idx1, sub2ind([length(acc_actions), length(steer_actions)], acc_idx1, steer_idx1)) = current_q1 + alpha * (reward + gamma * best_future_q1 - current_q1);

        % Q-learning update for car 2
        best_future_q2 = max(Q_table(next_state_idx2, :), [], 'all');
        current_q2 = Q_table(state_idx2, sub2ind([length(acc_actions), length(steer_actions)], acc_idx2, steer_idx2));
        Q_table(state_idx2, sub2ind([length(acc_actions), length(steer_actions)], acc_idx2, steer_idx2)) = current_q2 + alpha * (reward + gamma * best_future_q2 - current_q2);

        % Update the states for the next step
        state1 = next_state1;
        state2 = next_state2;

        % Check if either car reached its goal
        if check_goal(state1, ref_state1, 0.5)
            reward=reward+1000; % new line
            state1(4) = 0; % Stop car 1 by setting velocity to 0
        end
        if check_goal(state2, ref_state2, 0.5)
            reward=reward+1000; % new line
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
