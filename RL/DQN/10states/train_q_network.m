function train_q_network(q_network, experience_buffer, batch_size, gamma, lr, state_dim, action_dim)
    % Sample a batch from the replay buffer
    batch = datasample(experience_buffer, batch_size, 'Replace', false);

    % Extract components from the batch
    combined_states = batch(:, 1:state_dim); % Combined states of both cars
    actions1 = batch(:, state_dim + 1);
    actions2 = batch(:, state_dim + 2);
    rewards = batch(:, state_dim + 3);
    combined_next_states = batch(:, state_dim + 4:end);

    % Convert data to dlarray and ensure correct dimensions
    combined_states_dl = dlarray(combined_states', 'CB'); % Transpose to [state_dim, batch_size]
    combined_next_states_dl = dlarray(combined_next_states', 'CB'); % Transpose to [state_dim, batch_size]

    % Predict Q-values for the current states
    q_values = predict(q_network, combined_states_dl);

    % Predict target Q-values for the next states
    target_q_values = predict(q_network, combined_next_states_dl);

    % Compute the Q-learning targets
    targets = q_values;
    for i = 1:batch_size
        action1_idx = actions1(i);
        action2_idx = actions2(i);
        target_value = rewards(i) + gamma * max(target_q_values(:, i));
        targets(action1_idx, i) = target_value; % Update target for action1
        targets(action2_idx, i) = target_value; % Update target for action2
    end

    % Compute loss
    loss = mse(q_values, targets);

    % Update the Q-network weights using backpropagation
    gradients = dlgradient(loss, q_network.Learnables);
    q_network = dlupdate(@(w, g) w - lr * g, q_network, gradients);
end
