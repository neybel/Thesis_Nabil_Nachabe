clc
clearvars

% Parameters
dt = 0.1;
car_length = 2.5;

% Define possible actions for acceleration and steering rate
acc_actions = linspace(-5, 5, 30);  % 30 actions between -5 (full brake) and 5 (full throttle)
steer_actions = linspace(-1, 1, 30);  % 30 actions between -1 (left) and 1 (right)
state_dim = 10;  % Now includes both cars: 5 states each, so 10 states total
action_dim = length(acc_actions) * length(steer_actions);  % Number of actions

% Define Q-network for combined states
layers = [
    featureInputLayer(state_dim, 'Normalization', 'none', 'Name', 'input') % Now we input both cars' states (10 states total)
    fullyConnectedLayer(128, 'Name', 'fc1')
    reluLayer('Name', 'relu1')
    fullyConnectedLayer(128, 'Name', 'fc2')
    reluLayer('Name', 'relu2')
    fullyConnectedLayer(action_dim, 'Name', 'fc3') % Output layer for Q-values
];

q_network = dlnetwork(layers);  % Convert layers to a deep learning network

% Training parameters
buffer_capacity = 10000; % Size of the replay buffer
batch_size = 64;         % Batch size for training
epsilon = 1.0;           % Initial exploration rate
epsilon_decay = 0.995;   % Decay factor for epsilon
gamma = 0.95;            % Discount factor for future rewards
total_episodes = 1000;   % Total number of episodes
max_steps = 100;         % Maximum steps per episode
lr = 1e-3;               % Learning rate for gradient descent

% Initialize replay buffer
experience_buffer = [];

for episode = 1:total_episodes
    % Initialize states for both cars
    state1 = [0; 0; 0; 0; 0];  % Starting state for car 1
    state2 = [20; 0; 0; 0; 0]; % Starting state for car 2
    
    for step = 1:max_steps
        % Combine the states into one vector
        combined_state = [state1; state2];  % 10-element vector

        % Select actions for both cars using epsilon-greedy policy
        [action1, action2] = epsilon_greedy_policy(combined_state, epsilon, q_network, acc_actions, steer_actions);
        
        % Convert action indices to actual actions
        [acc_idx1, steer_idx1] = ind2sub([length(acc_actions), length(steer_actions)], action1);
        actual_action1 = [acc_actions(acc_idx1), steer_actions(steer_idx1)];

        [acc_idx2, steer_idx2] = ind2sub([length(acc_actions), length(steer_actions)], action2);
        actual_action2 = [acc_actions(acc_idx2), steer_actions(steer_idx2)];
        
        % Apply actions and get next states and rewards
        [next_state1, next_state2, reward1, reward2] = environment_step(state1, actual_action1, state2, actual_action2, car_length, dt, [10; 0; 0; 0; 0], [0; 10; 0; 0; 0]);

        % Combine the next states as well
        combined_next_state = [next_state1; next_state2];
        
        % Store experiences in the replay buffer
        experience_buffer = [experience_buffer; [combined_state', action1, action2, reward1 + reward2, combined_next_state']];
        
        % If buffer is large enough, train the Q-network
        if size(experience_buffer, 1) >= batch_size
            train_q_network(q_network, experience_buffer, batch_size, gamma, lr, state_dim, action_dim);
        end
        
        % Update states
        state1 = next_state1;
        state2 = next_state2;
        
        % Decay exploration rate
        epsilon = max(epsilon * epsilon_decay, 0.01);
    end
end
