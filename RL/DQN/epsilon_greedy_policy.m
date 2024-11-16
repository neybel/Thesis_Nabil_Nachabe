function action_idx = epsilon_greedy_policy(state, epsilon, q_network, acc_actions, steer_actions)
    state = state(:);  % Ensure state is a column vector
    state_dl = dlarray(state, 'CB');  % Convert to dlarray

    if rand < epsilon
        % Random action (exploration)
        acc_idx = randi([1, length(acc_actions)]);
        steer_idx = randi([1, length(steer_actions)]);
    else
        % Use Q-network to predict Q-values and pick the best action
        q_values = predict(q_network, state_dl);
        q_values = extractdata(q_values);  % Convert dlarray to numeric array
        [~, action_idx] = max(q_values);  % Get the index of the best action
        return;
    end

    % Return combined index for acceleration and steering rate
    action_idx = sub2ind([length(acc_actions), length(steer_actions)], acc_idx, steer_idx);
end
