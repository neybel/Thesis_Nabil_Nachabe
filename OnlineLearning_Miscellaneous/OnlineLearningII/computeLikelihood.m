 % Define computeLikelihood Function
function likelihood = computeLikelihood(X_sim,W_x, W_u)

%This function computes the deviation between the observed trajectory and the predicted trajectory obtained from ACADOS.
%It then calculates the total error and converts it into a likelihood value.
    % Parameters
    %%% Note that X_sim in the input of this function represent the
    %%% observation which is loaded from the .mat file
    nx = size(X_sim, 1);
%     nu = size(U_sim, 1);
    N = size(X_sim, 2);
    
    % Generate the predicted trajectory using ACADOS
    predicted_trajectory = OCP_solve(ocp, x0, W_x, W_u);

    % Compute the deviation between predicted trajectory and observed trajectory
    deviation = X_sim - predicted_trajectory;

    % Assuming state variables are ordered as [x1, y1, theta1, v1, steering1, x2, y2, theta2, v2, steering2]
state_weights = [1.0, 1.0, 0.1, 0.1, 0.1, 1.0, 1.0, 0.1, 0.1, 0.1]; % Example weights

% Element-wise multiplication of deviation with weights
weighted_deviation = state_weights' .* deviation;
    
    % Compute the squared deviation (error) for each time step
    error = sum(weighted_deviation.^2, 1); % Summing squared error over state dimensions
    
    % Total error (sum of all time steps)
    total_error = sum(error);
    
    % Convert the total error into likelihood (using a Gaussian-like function)
    scaling_factor = 1e-6; % Adjust as needed
    tolerance = 1e-9; % Adjust as needed
    
    % Compute the likelihood (smaller error means higher likelihood)
    likelihood = exp(-0.5 * scaling_factor * total_error);
    
    % Ensure the likelihood is within a reasonable range
    likelihood = max(likelihood, tolerance);
end

