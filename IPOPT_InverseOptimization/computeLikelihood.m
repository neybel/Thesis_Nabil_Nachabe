%% Define computeLikelihood Function
function likelihood = computeLikelihood(X_sim, U_sim, W_x, W_u,state_ref)
    % Compute the cost using the runMPC function with the given weights
    cost = runMPC(X_sim, U_sim, W_x, W_u);
    
    % Compute the observed cost
    observed_cost = computeObservedCost1(X_sim, U_sim,state_ref);
    % Scaling factor and tolerance
%     scaling_factor = 1e-10; % Adjust as needed
    scaling_factor = 1e-6; % Adjust as needed

%     tolerance = 1e-4; % Adjust as needed
    tolerance = 1e-9; % Adjust as needed

    
    % Compute scaled difference
    scaled_diff = scaling_factor * (cost - observed_cost);
    
    % Apply softmax function to the scaled difference
    likelihood = exp(-0.5 * (scaled_diff)^2);
    
    % Ensure the likelihood is within a reasonable range
    likelihood = max(likelihood, tolerance);
    
%     normalized_cost = cost / max(cost, observed_cost);
%     normalized_observed_cost = observed_cost / max(cost, observed_cost);
%     % Define the likelihood (negative log-likelihood for simplicity)
%         likelihood = exp(-0.5 * (normalized_cost - normalized_observed_cost)^2);
% %     likelihood = exp(-0.5 * (cost - observed_cost)^2);
end