function likelihood = computeLikelihoodTest(X_sim, predicted_trajectory)
    % Parameters
    nx = size(X_sim, 1);  % Number of state variables
    N = size(X_sim, 2);   % Number of time steps
    
    % Compute the deviation between predicted trajectory and observed trajectory
    deviation = X_sim - predicted_trajectory;

    % Normalize the deviation
    max_deviation = max(abs(deviation), [], 2);  % Maximum deviation for each state variable
    normalized_deviation = deviation ./ (max_deviation + 1e-6);  % Add small value to avoid division by zero

    % State weights, potentially increased for more significant states
    state_weights = [500.0, 500.0, 0.01, 0.1, 1e-5, 500.0, 500.0, 0.01, 0.1, 1e-5];
    
    % Apply state weights to normalized deviation
    weighted_deviation = state_weights' .* normalized_deviation;
    
    % Compute the squared deviation (error) for each time step
    error = sum(weighted_deviation.^2, 1); % Summing squared error over state dimensions
    
    % Total error (sum of all time steps)
    total_error = sum(error);
    
    % Convert the total error into likelihood (using a Gaussian-like function)
    scaling_factor = 1e-6;  % Decrease scaling factor for more sensitivity
    tolerance = 1e-9;  % To avoid zero likelihood

    % Compute the likelihood
    likelihood = exp(-0.5 * scaling_factor * total_error);
    
    % Ensure the likelihood is within a reasonable range
    likelihood = max(likelihood, tolerance);
end
