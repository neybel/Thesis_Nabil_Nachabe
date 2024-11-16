function likelihood = computeLikelihood_Euclidean(X_sim, predicted_trajectory)
    % Compute Euclidean distance between observed and predicted trajectories
    distance = sqrt(sum((X_sim(1:2, :) - predicted_trajectory(1:2, :)).^2, 1)); % considering x and y positions
    
    % Total error (sum of distances across all time steps)
    total_distance = sum(distance);

    std_distance=std(distance);
    
    % Convert total distance into a likelihood
    scaling_factor = 1; % Adjust this scaling factor as needed
% scaling_factor = 1 / (std_distance + 1e-5); % 1e-5 as epsilon
    likelihood = exp(-0.5 * scaling_factor * total_distance);
    
    % Ensure likelihood is within a reasonable range
    tolerance = 1e-15;
    likelihood = max(likelihood, tolerance);
end
