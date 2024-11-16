% Define the function to compute loss and gradients
function [loss, gradients] = compute_loss_and_gradients(q_network, states_dl, next_states_dl, actions, rewards, gamma, batch_size)
    % Predict Q-values for the current state
    q_values = predict(q_network, states_dl);

    % Predict target Q-values for the next state
    target_q_values = predict(q_network, next_states_dl);

    % Compute the Q-learning targets
    targets = q_values;
    for i = 1:batch_size
        action_idx = actions(i);
        target_value = rewards(i) + gamma * max(target_q_values(:, i));
        targets(action_idx, i) = target_value;
    end

    % Compute loss (mean squared error)
    loss = mse(q_values, targets);

    % Compute gradients
    gradients = dlgradient(loss, q_network.Learnables);
end