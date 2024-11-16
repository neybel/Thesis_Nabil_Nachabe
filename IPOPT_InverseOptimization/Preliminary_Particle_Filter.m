
num_particles = 1000;
process_noise_std = 0.1;
measurement_noise_std = 0.1;

%% Comments
% Particle filters are a form of sequential Monte Carlo methods used for state estimation in non-linear and non-Gaussian processes. They are particularly useful when dealing with complex HMMs where the state and observation models are non-linear.
%% Particle Filter tinkering to update the 'gamma' parameter iteratively
%%%The particle filter uses Bayes' theorem iteratively:
% The prior is the distribution of particles at the current time step.
% The likelihood is computed based on the observation.
% The posterior is formed by updating particle weights using the likelihood and then normalizing them.
% By repeating these steps, the particle filter provides an online, sequential estimation of the state (parameter 
% 𝛾) distribution, effectively implementing Bayesian inference.
for t = 1:N
    % Extract p and p_ref for the current time step
    p_val = [X_star(:, t); U_star(:, t)];
    if t + horizon_length <= size(References, 2)
        p_ref_val = References(:, t:(t + horizon_length - 1));
    else
        p_ref_val = References(:, t:end);
    end
    
    % Flatten p_ref to a column vector
    p_ref_val = p_ref_val(:);

    % Define p_next (the next state in the horizon)
    if t + 1 <= N
        p_next_val = X_star(1:5, t + 1);
    else
        p_next_val = X_star(1:5, t); % Use the last state if at the end
    end

    % Create the parameters vector
    P_val = [p_val; p_ref_val; p_next_val];

    % Update gamma using Particle filter
    for obs = observations
        % Prediction step
        particles = particles + normrnd(0, process_noise_std, [1, num_particles]);

        % Update step
        likelihoods = normpdf(obs, particles, measurement_noise_std);
        weights = weights .* likelihoods;
        weights = weights / sum(weights);  % Normalize weights

        % Resample step
        indices = randsample(1:num_particles, num_particles, true, weights);
        particles = particles(indices);
        weights = ones(1, num_particles) / num_particles;  % Reset weights
    end

    % The updated gamma parameter
    gamma_updated = mean(particles);

    % Define the entropic risk-aware cost function with updated gamma
    risk_aware_cost = (1/gamma_updated) * log(exp(gamma_updated * original_cost));

    % Define the NLP problem
    nlp = struct('x', W, 'f', risk_aware_cost, 'p', vertcat(p, p_ref, p_next));

    % Set options for the IPOPT solver
    opts = struct('ipopt', struct('print_level', 5, 'tol', 1e-6, 'max_iter', 500));

    % Create the solver
    solver = nlpsol('solver', 'ipopt', nlp, opts);

    % Solve the problem
    sol = solver('x0', W0, 'p', P_val);

    % Extract the optimal solution
    W_opt = full(sol.x);
    W_opt_array(:, t) = W_opt;
    W0 = W_opt;
end
