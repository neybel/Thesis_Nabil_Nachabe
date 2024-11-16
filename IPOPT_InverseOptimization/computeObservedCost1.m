% Define computeObservedCost Function
function observed_cost = computeObservedCost1(X_sim, U_sim, state_ref)
    % Parameters
    nx = size(X_sim, 1);
    nu = size(U_sim, 1);
    N = size(X_sim, 2);

    % Example proxy weights (identity matrices)
    W_x_proxy = 1e-2*eye(nx);
    W_x_proxy(1,1) = 10;W_x_proxy(2,2)=10;W_x_proxy(6,6)=10;W_x_proxy(7,7)=10;
    W_u_proxy = 1e-1*eye(nu);
    
    % Initialize observed cost
    observed_cost = 0;
    
    % Loop through each timestep
    for k = 1:N
        % State and control deviation from reference
        state_deviation = X_sim(:, k) - state_ref(:, k);
        control_effort = U_sim(:, k);
        
        % Quadratic cost contributions
        observed_cost = observed_cost + state_deviation' * W_x_proxy * state_deviation + control_effort' * W_u_proxy * control_effort;
    end
    
    % Final state deviation cost
    final_state_deviation = X_sim(:, end) - state_ref(:, end);
    observed_cost = observed_cost + final_state_deviation' * W_x_proxy * final_state_deviation;
end