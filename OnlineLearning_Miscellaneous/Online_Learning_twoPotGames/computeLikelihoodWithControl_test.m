function [likelihood,total_error,scaling_factor] = computeLikelihoodWithControl_test(X_sim, predicted_trajectory, U_star, predicted_control)
    % Parameters
    nx = size(X_sim, 1);  % Number of state variables
    nu = size(U_star, 1); % Number of control inputs
    N = size(X_sim, 2);   % Number of time steps
    % Assuming state variables are weighted (if needed)
    state_weights = [1, 1, 1e-5, 1e-2, 1e-5, 1, 1, 1e-5, 1e-2, 1e-5];
    control_weights = [1, 1, 1, 1]; % Example weights for control inputs
    % Compute the deviation between predicted and observed state trajectory
    state_deviation = X_sim - predicted_trajectory;

    % Compute the deviation between predicted and observed control inputs
    control_deviation = U_star - predicted_control;
    %%
%     delta=0.1;
% state_error = abs(state_deviation);
% control_error = abs(control_deviation);
% state_loss = (state_error <= delta) .* (state_error.^2) + (state_error > delta) .* (2*delta*state_error - delta^2);
% control_loss = (control_error <= delta) .* (control_error.^2) + (control_error > delta) .* (2*delta*control_error - delta^2);
% 
% total_error = sum(sum(state_loss .* state_weights', 1)) + sum(sum(control_loss .* control_weights', 1));



state_cov = eye(nx); % Covariance matrix for state deviations
control_cov = eye(nu); % Covariance matrix for control deviations

state_mahalanobis = sqrt(sum((state_deviation' / state_cov) .* state_deviation', 2));
control_mahalanobis = sqrt(sum((control_deviation' / control_cov) .* control_deviation', 2));

total_error = sum(state_mahalanobis) + sum(control_mahalanobis);
%%
% Compute scaling factor
% This example assumes dynamic scaling based on total_error statistics
dynamic_constant = max(total_error) + 1e-5;
scaling_factor = 1 / (dynamic_constant); 

scaling_factor = 1 / (mean(total_error) + 1e-5); % Example adjustment


% Compute likelihood
likelihood = exp(-0.5 * scaling_factor * total_error);

% Ensure likelihood is within a reasonable range
likelihood = max(likelihood, 1e-15);
end
