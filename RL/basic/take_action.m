%% Helper function: Take action and return next state and reward
function [next_state, reward] = take_action(current_state, action, reward_matrix)
    % Transition logic for state and action
    % Action: 1 = steer left, 2 = go straight, 3 = steer right

    switch current_state
        case {1, 4, 7} % In left lane
            if action == 1
                next_state = current_state; % Stay in left lane (safe)
            elseif action == 2
                next_state = current_state + 1; % Move to center lane
            else
                next_state = current_state + 2; % Move to right lane
            end
        case {2, 5, 8} % In center lane
            if action == 1
                next_state = current_state - 1; % Move to left lane
            elseif action == 2
                next_state = current_state; % Stay in center lane
            else
                next_state = current_state + 1; % Move to right lane
            end
        case {3, 6, 9} % In right lane
            if action == 1
                next_state = current_state - 2; % Move to center lane
            elseif action == 2
                next_state = current_state; % Stay in right lane
            else
                next_state = current_state; % Stay in right lane
            end
    end
    
    % Get reward for this state-action pair
    reward = reward_matrix(current_state, action);
end
