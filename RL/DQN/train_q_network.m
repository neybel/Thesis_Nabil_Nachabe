function train_q_network(q_network, experience_buffer, batch_size, gamma, lr, state_dim, action_dim)
    % Sample a batch from the replay buffer
    batch = datasample(experience_buffer, batch_size, 'Replace', false);

    % Extract components from the batch
    states = batch(:, 1:state_dim);
    actions = batch(:, state_dim + 1); % Now this is a single column of action indices
    rewards = batch(:, state_dim + 2);
    next_states = batch(:, state_dim + 3:end);

    if canUseGPU
    states_dl = dlarray(gpuArray(states'), 'CB');
    next_states_dl = dlarray(gpuArray(next_states'), 'CB');
    else
    states_dl = dlarray(states', 'CB');
    next_states_dl = dlarray(next_states', 'CB');
    end


%     % Convert data to dlarray and ensure correct dimensions
%     states_dl = dlarray(states', 'CB'); % Transpose to [state_dim, batch_size]
%     next_states_dl = dlarray(next_states', 'CB'); % Transpose to [state_dim, batch_size]

    % Use dlfeval to compute loss and gradients
    [loss, gradients] = dlfeval(@compute_loss_and_gradients, q_network, states_dl, next_states_dl, ...
                                actions, rewards, gamma, batch_size);

    % Update the Q-network weights using backpropagation
    q_network = dlupdate(@(w, g) w - lr * g, q_network, gradients);
end