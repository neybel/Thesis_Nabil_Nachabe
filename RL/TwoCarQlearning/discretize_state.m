function [x_idx, y_idx, theta_idx, v_idx, delta_idx] = discretize_state(state, num_x_states, num_y_states, num_theta_states, num_v_states, num_delta_states)
    % Discretize the state for both agents
    x_idx = min(max(1, round(state(1))), num_x_states);
    y_idx = min(max(1, round(state(2))), num_y_states);
    theta_idx = min(max(1, round(state(3) * num_theta_states / (2*pi))), num_theta_states);
    v_idx = min(max(1, round(state(4))), num_v_states);
    delta_idx = min(max(1, round(state(5))), num_delta_states);
end
