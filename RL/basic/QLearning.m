% Q-Learning Parameters
alpha = 0.1; % Learning rate
gamma = 0.9; % Discount factor
epsilon = 1.0; % Exploration probability
epsilon_decay = 0.995; % Decay rate for epsilon
max_episodes = 1000; % Number of episodes to train
max_steps = 100; % Max steps per episode

% Define state space (9 states) and action space (3 actions)
num_states = 9; % 9 states (3 lane positions x 3 distances to other vehicles)
num_actions = 3; % steer left, go straight, steer right

% Initialize Q-table
Q_table = zeros(num_states, num_actions);

% Define a reward matrix that matches the number of states and actions
reward_matrix = [-1, -1, -1;    % Steer left, Go straight, Steer right for state 1
                 +10, +10, +10; % State 2
                 -50, -50, -50; % State 3 (Collision state, large negative reward)
                 +1, +10, +1;   % State 4
                 +1, +10, +1;   % State 5 (etc.)
                 -50, -50, -50; % State 6
                 +1, +10, +1;   % State 7
                 +1, +10, +1;   % State 8
                 -50, -50, -50];% State 9 (Collision state, large negative reward)

% Q-Learning loop
for episode = 1:max_episodes
    % Initialize the state (start at a random state)
    current_state = randi(num_states); % Random start state
    done = false;

    for step = 1:max_steps
        % Choose action using epsilon-greedy policy
        if rand() < epsilon
            action = randi([1, num_actions]); % Random action (exploration)
        else
            [~, action] = max(Q_table(current_state, :)); % Best action (exploitation)
        end
        
        % Take action, get next state and reward
        [next_state, reward] = take_action(current_state, action, reward_matrix);

        % Q-learning update rule
        Q_table(current_state, action) = Q_table(current_state, action) + ...
            alpha * (reward + gamma * max(Q_table(next_state, :)) - Q_table(current_state, action));
        
        % Move to the next state
        current_state = next_state;

        % End episode if in terminal state (collision or success)
        if is_terminal_state(current_state)
            done = true;
            break;
        end
    end
    
    % Decay exploration probability (epsilon)
    epsilon = max(0.01, epsilon * epsilon_decay); % Prevent epsilon from going below 0.01
end

% Display the learned Q-table
disp('Learned Q-Table:');
disp(Q_table);