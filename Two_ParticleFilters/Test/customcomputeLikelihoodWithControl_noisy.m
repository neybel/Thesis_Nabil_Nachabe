function [likelihood, total_error, scaling_factor] = customcomputeLikelihoodWithControl_noisy(X_sim, predicted_trajectory, U_star, predicted_control)
    % Parameters
    nx = size(X_sim, 1);  % Number of state variables
    nu = size(U_star, 1); % Number of control inputs
    N = size(X_sim, 2);   % Number of time steps


    std_state = [0.1; 0.1; 0.01; 0.05; 0.02];  % For (x, y, theta, velocity, steering)
std_control = [0.1; 0.01];  % For (acceleration, steering rate)
% std_state = [0.05; 0.05; 0.005; 0.025; 0.01];  % For (x, y, theta, velocity, steering)
% std_control = [0.05; 0.005];  % For (acceleration, steering rate)

    % Compute the deviation between predicted and observed state trajectory
    state_deviation = X_sim - predicted_trajectory;

    % Compute the deviation between predicted and observed control inputs
    control_deviation = U_star - predicted_control;

    % Assuming state variables are weighted (if needed)
    importance_state_weights = [1, 1, 1e-5, 1e-5, 1e-5]; % Importance-based weights for state variables
    importance_control_weights = [1, 1];                 % Importance-based weights for control inputs
    
    % Noise-based weights (derived from standard deviation of noise)
%     noise_state_weights = 1 ./ (2 * std_state.^2);  % Noise-based weights for state
%     noise_control_weights = 2 * std_control.^2;  % Direct relation with variance (not inverse)
noise_state_weights = 0;noise_control_weights=0;

    combined_state_weights = importance_state_weights ./ (1 + noise_state_weights);
    normalized_weights_state = combined_state_weights / sum(combined_state_weights);

    combined_control_weights = importance_control_weights .* (1 ./ (1 + noise_control_weights));
    normalized_weights_control = combined_control_weights / sum(combined_control_weights);



%     noise_state_weights = 1 ;  % Noise-based weights for state
%     noise_control_weights = 1;  % Noise-based weights for control
    % Combine both importance and noise-based weights
%     combined_state_weights = importance_state_weights .* noise_state_weights';
%     combined_control_weights = importance_control_weights .* noise_control_weights';

    % Compute weighted state and control errors
    weighted_state_error = sum((combined_state_weights* state_deviation).^2, 1);
    weighted_control_error = sum((combined_control_weights* control_deviation).^2, 1);

    % Combine state and control errors
    total_error = sum(weighted_state_error) + sum(weighted_control_error);
% 
% 
% norm_state_error = state_deviation ./ (std(state_deviation(:)) + 1e-5);
% norm_control_error = control_deviation ./ (std(control_deviation(:)) + 1e-5);
% total_error = sum(combined_state_weights' .*norm_state_error) + sum(combined_control_weights' .*norm_control_error);
% scaling_factor = 10 * 1 / max((max(std(norm_state_error), std(norm_control_error))) + 1e-5);
% Likelihood calculation using an adaptive scaling factor
    std_error1 = std(weighted_state_error);
    std_error2 = std(weighted_control_error);
    dynamic_constant = 2.5 * max(std_error2, std_error1);
    scaling_factor = 1 / (dynamic_constant + 1e-5); % Ensure non-zero scaling factor
    % Compute likelihood
    likelihood = exp(-0.5 * scaling_factor * total_error);

    % Ensure likelihood is within a reasonable range
    likelihood = max(likelihood, 1e-30);
end
