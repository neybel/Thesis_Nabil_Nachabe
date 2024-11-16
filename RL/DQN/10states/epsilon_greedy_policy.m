function [action1, action2] = epsilon_greedy_policy(combined_state, epsilon, q_network, acc_actions, steer_actions)
    % Determine the number of actions
    num_acc_actions = length(acc_actions);
    num_steer_actions = length(steer_actions);
    
    % Total number of actions
    num_actions = num_acc_actions * num_steer_actions;

    % Epsilon-greedy policy
    if rand < epsilon
        % Random action
        action1 = randi(num_actions);
        action2 = randi(num_actions);
    else
        % Get Q-values for the current state
        combined_state_dl = dlarray(combined_state(:), 'CB'); % Convert to dlarray
        
        q_values = predict(q_network, combined_state_dl);
        
        % Extract the Q-values for both actions
        [~, best_action_idx] = max(q_values, [], 'all', 'linear');
        
        % Convert linear index to subscript indices
        [action1_idx, action2_idx] = ind2sub([num_acc_actions, num_steer_actions], best_action_idx);
        
        % Convert subscript indices to action indices
        action1 = sub2ind([num_acc_actions, num_steer_actions], action1_idx, action2_idx);
        action2 = sub2ind([num_acc_actions, num_steer_actions], action2_idx, action1_idx);
    end
end
