% Parameters for the vehicle
clc
clearvars
close all
L = 2.5;  % Wheelbase (m)
dt = 0.1; % Time step (s)
Q_xy=1e2; Q_theta = 1e-5; Q_v = 1e0; R_a = 1e-5; R_delta = 1e-5;

% Discretization parameters
num_x_states = 50;
num_y_states = 50;
num_theta_states = 15;
num_v_states = 15;
num_delta_states = 5;
% num_actions = 3; % 3 actions for each control input (acceleration and steering rate)
% Action space (discrete values for acceleration and steering rate)
acc_actions = linspace(-1, 1, 25); % 5 possible actions for acceleration
steer_actions = linspace(-0.01, 0.2, 25); % 5 possible actions for steering rate


% Initialize the Q-table
% Initialize the Q-table
Q_table = rand(num_x_states, num_y_states, num_theta_states, num_v_states, num_delta_states, length(acc_actions), length(steer_actions));

% Learning parameters
alpha = 0.10;  % Learning rate: Increase α: If the learning rate is too low, the Q-values may change too slowly, resulting in slow convergence. Increasing α can help the algorithm learn faster.
% Decrease α: If α is too high, the Q-values might oscillate or become
% unstable. Decrease α if the learning process appears erratic or unstable. Typical Values: 0.01 to 0.5.

gamma = 0.95; % Discount factor: Definition: The discount factor γ determines how much future rewards are considered in the Q-value updates. It balances immediate versus future rewards.
% Typical Values: 0.9 to 0.99. Increase γ: A higher γ values future rewards more, which can be beneficial if future rewards are more important. However, very high values can make learning slow and overly optimistic.
% Decrease γ: A lower γ values immediate rewards more heavily. This might be useful if immediate rewards are more critical than future rewards.
epsilon = 0.2; % Exploration rate: Definition: The exploration rate ε determines the probability of choosing a random action instead of the best-known action. It ensures that the agent explores the state-action space and does not always exploit the current knowledge.
% Typical Values: 0.1 to 0.5.
% Increase ε: More exploration can help the agent discover better strategies and avoid getting stuck in local optima. If the agent is not exploring enough, increase ε.
% Decrease ε: Lower values of ε make the agent exploit its current knowledge more and explore less. If the agent is exploring too much and not converging, decrease ε.

% Reference state to track
ref_state = [7.5; 7.5; pi/2; 0;0]; % [x, y, theta, v]
%%
% Action space (discrete values for acceleration and steering rate)
% actions = [-1, 0, 1]; % 3 possible actions for both acceleration and steering rate
total_episodes = 9e4;
hWaitBar = waitbar(0, 'Training Progress', 'Name', 'Q-learning Training');
for episode = 1:total_episodes
    % Initialize integral error
    integral_error = 0;
    state = [0; 0; 0; 0; 0]; % Start at initial state
    for step = 1:250
        % Discretize the current state
        [x_idx, y_idx, theta_idx, v_idx, delta_idx] = discretize_state(state, num_x_states, num_y_states, num_theta_states, num_v_states, num_delta_states);
        
        % Epsilon-greedy action selection
        if rand < epsilon
            % Exploration: select random actions
            acc_idx = randi(length(acc_actions)); % Random index for acceleration
            steer_idx = randi(length(steer_actions)); % Random index for steering rate
        else
            % Exploitation: select best actions based on Q-table
            q_values = reshape(Q_table(x_idx, y_idx, theta_idx, v_idx, delta_idx, :, :), [length(acc_actions), length(steer_actions)]);
            [~, max_idx] = max(q_values(:)); % Get the index of the maximum Q-value
            [acc_idx, steer_idx] = ind2sub(size(q_values), max_idx); % Convert linear index to 2D subscripts
        end
        
        % Get the chosen action (accelerclcation and steering rate)
        u = [acc_actions(acc_idx); steer_actions(steer_idx)]; % Map the indices to actions
        
        % Apply the action to get next state using the kinematic model
        next_state = kinematic_model(state, u, L, dt);
        
        % Calculate reward (negative cost)
            reward = -cost_function(next_state, ref_state, u, Q_xy, Q_theta, Q_v, R_a, R_delta);
%           reward = -cost_function(next_state, ref_state, u, integral_error, Q_xy, Q_theta, Q_v, R_a, R_delta, dt);

        
        % Discretize next state
        [next_x_idx, next_y_idx, next_theta_idx, next_v_idx, next_delta_idx] = discretize_state(next_state, num_x_states, num_y_states, num_theta_states, num_v_states, num_delta_states);
        
        % Q-learning update
        best_future_q = max(max(Q_table(next_x_idx, next_y_idx, next_theta_idx, next_v_idx, next_delta_idx, :, :), [], 'all'));
        current_q = Q_table(x_idx, y_idx, theta_idx, v_idx, delta_idx, acc_idx, steer_idx);
        Q_table(x_idx, y_idx, theta_idx, v_idx, delta_idx, acc_idx, steer_idx) = current_q + alpha * (reward + gamma * best_future_q - current_q);
        
        % Update the state for the next step
        state = next_state;
    end
    
    % Decay exploration rate epsilon
    epsilon = epsilon * 0.99;
    % Update the progress bar
progress = episode / total_episodes;
waitbar(progress, hWaitBar, sprintf('Episode %d of %d (%.2f%% complete)', episode, total_episodes, progress * 100));
end
% Close the progress bar
close(hWaitBar);
% Display the learned Q-table
% disp(Q_table);
