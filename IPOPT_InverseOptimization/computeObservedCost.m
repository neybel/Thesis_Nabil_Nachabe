%% Define computeObservedCost Function
function observed_cost = computeObservedCost(X_sim, U_sim)
    % Placeholder for the observed cost calculation
    % You should define how you compute the observed cost based on your simulation data
    
    % Example: Compute the observed cost using the same cost function but with fixed weights
    % Define fixed weights
    W_x_fixed = diag([1e1, 1e3, 1e-2, 1e-1, 1e-5, 1e3, 1e1, 1e-2, 1e-1, 1e-10]);
    W_u_fixed = diag([1e-3, 1e2, 1e-3, 1e3]);
    
    % Calculate the cost with fixed weights
    observed_cost = runMPC(X_sim, U_sim, W_x_fixed, W_u_fixed);
end