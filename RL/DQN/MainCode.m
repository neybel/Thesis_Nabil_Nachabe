clc
clearvars

% Parameters
dt = 0.1;
car_length = 2.5;

% Define possible actions for acceleration and steering rate
acc_actions = linspace(-11.5, 11.5, 15);  %  30
steer_actions = linspace(-1, 1, 15);  % 30
state_dim = 5;  % e.g., [x, y, theta, velocity, steering]
action_dim = length(acc_actions) * length(steer_actions);  % Number of actions

% Define Q-network
layers = [
    featureInputLayer(5, 'Normalization', 'none', 'Name', 'input') % Ensure this matches state_dim
    fullyConnectedLayer(32, 'Name', 'fc1') %% 128
    reluLayer('Name', 'relu1')
    fullyConnectedLayer(32, 'Name', 'fc2') %% 128 beofe
    reluLayer('Name', 'relu2')
    fullyConnectedLayer(action_dim, 'Name', 'fc3') % Output layer for Q-values
];

q_network = dlnetwork(layers);  % Convert layers to a deep learning network
% q_network = dlnetwork(layers, 'ExecutionEnvironment', 'gpu');
if canUseGPU
    q_network.Learnables.Value = cellfun(@gpuArray, q_network.Learnables.Value, 'UniformOutput', false);
end

% Training parameters
buffer_capacity = 10000; % Size of the replay buffer
batch_size = 16;         % Batch size for training 64
epsilon = 1.0;           % Initial exploration rate
epsilon_decay = 0.999;   % Decay factor for epsilon 0.995
gamma = 0.95;            % Discount factor for future rewards
total_episodes = 100;   % Total number of episodes
max_steps = 100;         % Maximum steps per episode
lr = 1e-2;               % Learning rate for gradient descent 1e-3

% Initialize replay buffer
experience_buffer = [];

% Initialize progress bar
h = waitbar(0, 'Starting Training...');

for episode = 1:total_episodes
    % Initialize states for both cars
    state1 = [0; 0; 0; 0; 0];  % Starting state for car 1
    state2 = [20; 0; 0; 0; 0]; % Starting state for car 2
    
    for step = 1:max_steps
        % Select action for both cars using epsilon-greedy policy
        [action1] = epsilon_greedy_policy(state1, epsilon, q_network, acc_actions, steer_actions);
        [action2] = epsilon_greedy_policy(state2, epsilon, q_network, acc_actions, steer_actions);

        % Convert action indices to actual actions
        [acc_idx1, steer_idx1] = ind2sub([length(acc_actions), length(steer_actions)], action1);
        actual_action1 = [acc_actions(acc_idx1), steer_actions(steer_idx1)];

        [acc_idx2, steer_idx2] = ind2sub([length(acc_actions), length(steer_actions)], action2);
        actual_action2 = [acc_actions(acc_idx2), steer_actions(steer_idx2)];

        % Apply actions and get next states and rewards
        [next_state1, reward1] = environment_step(state1, actual_action1, car_length, dt, [10; 0; 0; 0; 0], state2, state_dim);
        [next_state2, reward2] = environment_step(state2, actual_action2, car_length, dt, [0; 10; 0; 0; 0], state1, state_dim);

        % Store experiences in the replay buffer
        experience_buffer = [experience_buffer; [state1', action1, reward1, next_state1']];
        experience_buffer = [experience_buffer; [state2', action2, reward2, next_state2']];

        % If buffer is large enough, train the Q-network
%         if size(experience_buffer, 1) >= batch_size
%             train_q_network(q_network, experience_buffer, batch_size, gamma, lr, state_dim, action_dim);
%         end
        if mod(step, 4) == 0 && size(experience_buffer, 1) >= batch_size
            train_q_network(q_network, experience_buffer, batch_size, gamma, lr, state_dim, action_dim);
        end

        % Update states
        state1 = next_state1;
        state2 = next_state2;

        % Decay exploration rate
        epsilon = max(epsilon * epsilon_decay, 0.01);
    end
    
    % Update the progress bar
waitbar(episode / total_episodes, h, sprintf('Episode %d of %d (%.1f%%)', episode, total_episodes, 100 * episode / total_episodes));
end

% Close progress bar
close(h);
