% Define computeLikelihood Function
function likelihood = computeLikelihood1(X_sim, predicted_trajectory)
    % This function computes the deviation between the observed trajectory
    % and the predicted trajectory obtained from ACADOS, and then calculates
    % the total error and converts it into a likelihood value.

    % Parameters
    nx = size(X_sim, 1);  % Number of state variables
    N = size(X_sim, 2);   % Number of time steps
    
    % Compute the deviation between predicted trajectory and observed trajectory
    deviation = X_sim - predicted_trajectory;

    % Assuming state variables are ordered as [x1, y1, theta1, v1, steering1, x2, y2, theta2, v2, steering2]
    state_weights = [10, 10, 0.01, 0.01, 1e-5, 10, 10, 0.01, 0.01, 1e-5]; % Example weights
%     state_weights = [200.0, 200.0, 0.01, 0.1, 1e-5]; % Example weights
%     state_weights = [1, 1, 1e-5, 1e-2, 1e-5, 1, 1, 1e-5, 1e-2, 1e-5]; % Example weights
%     state_weights = [200, 200, 0.01, 0.01, 1e-5, 200, 200, 0.01, 0.01, 1e-5]; % Example weights




    % Element-wise multiplication of deviation with weights
    weighted_deviation = state_weights' .* deviation;
    
%     Compute the squared deviation (error) for each time step
    error = sum(weighted_deviation.^2, 1); % Summing squared error over state dimensions

% % Example using Manhattan distance
% error = sum(abs(weighted_deviation), 1);
%     
    % Total error (sum of all time steps)
    total_error = sum(error);
%     transformed_error = log(1 + total_error);  % Log transformation

    
    % Convert the total error into likelihood (using a Gaussian-like function)
%     scaling_factor = 1e-6; % Adjust as needed
%         scaling_factor = 1 / (1e-5 + var(error));  % Adjust as needed

        scaling_factor = 1 / (max(error) - min(error));

%         normalized_error = total_error / (1e-5 + max(error));  % Avoid division by zero


% scaling_factor = 1e-5;
    tolerance = 1e-10; % Adjust as needed
    
    % Compute the likelihood (smaller error means higher likelihood)
%     likelihood = exp(-0.5 * scaling_factor * total_error); %% inspired form the gaussian distribution
likelihood = exp(-0.5 * scaling_factor * total_error); %% manhattan stuff

%     likelihood = exp(-0.5 * scaling_factor * normalized_error); %% inspired form the gaussian distribution
%     likelihood = exp(-0.5 * scaling_factor * transformed_error); %% inspired form the gaussian distribution

    %% Softmax
%     % Compute raw likelihoods
% raw_likelihoods = exp(-0.5 * scaling_factor * total_error);
% 
% % Apply softmax function
% likelihood = exp(raw_likelihoods) / sum(exp(raw_likelihoods));
    %% Student T distribution
% nu = 2; % degrees of freedom for Student's t-distribution, adjust as needed
% likelihood = (1 + (total_error / nu))^(-(nu + 1) / 2);
%%
likelihood = max(likelihood, tolerance);
end
