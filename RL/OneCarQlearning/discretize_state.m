% Discretization function
function [x_idx, y_idx, theta_idx, v_idx, delta_idx] = discretize_state(state, num_x, num_y, num_theta, num_v, num_delta)
    % Discretize state space for Q-table indexing
    x_idx = min(max(1, round(state(1))), num_x);
    y_idx = min(max(1, round(state(2))), num_y);
    theta_idx = min(max(1, round(state(3))), num_theta);
    v_idx = min(max(1, round(state(4))), num_v);
    delta_idx = min(max(1, round(state(5))), num_delta);
end