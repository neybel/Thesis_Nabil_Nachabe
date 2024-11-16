function [likelihood,total_error,scaling_factor] = computeLikelihoodWithControl(X_sim, predicted_trajectory, U_star, predicted_control)
    % Parameters
    nx = size(X_sim, 1);  % Number of state variables
    nu = size(U_star, 1); % Number of control inputs
    N = size(X_sim, 2);   % Number of time steps

    % Compute the deviation between predicted and observed state trajectory
    state_deviation = X_sim - predicted_trajectory;

    % Compute the deviation between predicted and observed control inputs
    control_deviation = U_star - predicted_control;
    %% Normalization of errors (might not be needed)
%     state_deviation = state_deviation / (std(state_deviation(:)) + 1e-5);
%     control_deviation = control_deviation / (std(control_deviation(:)) + 1e-5);

    %%

    % Assuming state variables are weighted (if needed)
    state_weights = [1, 1, 1e-5, 1e-5, 1e-5, 1, 1, 1e-5, 1e-5, 1e-5];
    control_weights = [1, 1, 1, 1]; % Example weights for control inputs

    % Compute weighted state and control errors
    weighted_state_error = sum((state_weights' .* state_deviation).^2, 1);
    weighted_control_error = sum((control_weights' .* control_deviation).^2, 1);

    % Combine state and control errors
    total_error = sum(weighted_state_error) + sum(weighted_control_error);

    % Likelihood calculation using an adaptive scaling factor
    std_error1 = std(weighted_state_error);
    std_error2 = std(weighted_control_error);
    %% IQR
%     iqr_state_error = iqr(weighted_state_error);
% iqr_control_error = iqr(weighted_control_error);
% mean_state_error = mean(weighted_state_error);
% mean_control_error = mean(weighted_control_error);
% 
% % Alternative 1: Combine mean and IQR for the scaling factor
% scaling_factor1 = 1 / (mean_state_error + iqr_state_error + 1e-5);
% scaling_factor2 = 1 / (mean_control_error + iqr_control_error + 1e-5);
% scaling_factor = min(scaling_factor1, scaling_factor2);

% Alternative 2: Combine mean and variance for the scaling factor
% variance_state_error = var(weighted_state_error);
% variance_control_error = var(weighted_control_error);
% scaling_factor1 = 1 / (mean_state_error + variance_state_error + 1e-5);
% scaling_factor2 = 1 / (mean_control_error + variance_control_error + 1e-5);
% scaling_factor = min(scaling_factor1, scaling_factor2);




%% Scaling Factor
% scaling_factor=mean([std_error2,std_error1]);
% scaling_factor=min(std_error2,std_error1);
% scaling_factor = 1e-3; 
% scaling_factor = 1 / (max(std_error2, std_error1) + 1e-3);

dynamic_constant = 2.5 * max(std_error2, std_error1);
scaling_factor = 1 / (dynamic_constant + 1e-5); % Ensure non-zero scaling factor

% scaling_factor1 = abs(1 / (max( weighted_state_error) - min(weighted_state_error)));
% scaling_factor2 =abs(1 / (max( weighted_control_error) - min(weighted_control_error)));
% scaling_factor = min(scaling_factor2,scaling_factor1);

%% 1. Adaptive Scaling Factor Based on Error Statistics
% mean_state_error = mean(weighted_state_error);
% mean_control_error = mean(weighted_control_error);
% std_state_error = std(weighted_state_error);
% std_control_error = std(weighted_control_error);
% norm_state_error = (weighted_state_error - mean_state_error) / (std_state_error + 1e-5);
% norm_control_error = (weighted_control_error - mean_control_error) / (std_control_error + 1e-5);
% 
% total_error = sum(norm_state_error) + sum(norm_control_error);
% % scaling_factor = 1;  % Or a very small value if needed
% scaling_factor1 = abs(1 / (max(norm_state_error) - min(norm_state_error) + 1e-5));
% scaling_factor2 = abs(1 / (max(norm_control_error) - min(norm_control_error) + 1e-5));
% 
% % Increase these factors to ensure differences in likelihood
% scaling_factor1 = scaling_factor1 * 100;  % Experiment with multiplying by 10 or 100
% scaling_factor2 = scaling_factor2 * 100;
% 
% scaling_factor = min(scaling_factor2, scaling_factor1);

%     scaling_factor = 1 / (std_error + 1e-5);
    likelihood = exp(-0.5 * scaling_factor * total_error);
%     smooth_likelihood = 1 / (1 + exp(0.5 * scaling_factor * total_error));
%     likelihood = smooth_likelihood;


    % Ensure likelihood is within a reasonable range
    likelihood = max(likelihood, 1e-30);
end
