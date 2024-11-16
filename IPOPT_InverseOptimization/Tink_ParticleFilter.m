%% Bayesian Inference with Particle Filter
%% Referencing
waypoints = Way_IPOPT();
waypoint_count = size(waypoints, 2);
T = 0.02;  % Time step


ref1 = zeros(5, waypoint_count);
ref2 = zeros(5, waypoint_count);

ref1(1, :) = waypoints(1, :);
ref1(2, :) = waypoints(2, :);
ref1(3, :) = (pi/2) * ones(1, waypoint_count);
ref1(4, :) = 2 * ones(1, waypoint_count);
ref1(5, :) = zeros(1, waypoint_count);

ref2(1, :) = linspace(125, 125 - 4 * waypoint_count * T, waypoint_count);
ref2(2, :) = 0.5 * ones(1, waypoint_count);
ref2(3, :) = pi * ones(1, waypoint_count);
ref2(4, :) = 2 * ones(1, waypoint_count);
ref2(5, :) = zeros(1, waypoint_count);


%%
% Load observations
load('observations.mat', 'X_sim', 'U_sim'); % Load your X_sim and U_sim

l=size(X_sim,2);
ref1=ref1(:,1:l);
ref2=ref2(:,1:l);

state_ref = [ref1;ref2]; % Example: use the first column of ref1


% Define particle filter parameters
num_particles = 30; % Number of particles
num_iterations = 1; % Number of iterations

% Define prior distribution for weights
% W_x_prior_mean = diag([1e1, 1e3, 1e-2, 1e-1, 1e-5, 1e3, 1e1, 1e-2, 1e-1, 1e-10]);
% W_u_prior_mean = diag([1e-3, 1e2, 1e-3, 1e3]);
W_x_prior_mean = diag([1, 1, 1e-2, 1e-2, 1e-2, 10, 10, 1e-2, 1e-2, 1e-2]);
W_u_prior_mean = diag([1e-1, 200, 1e-1, 200]);


%%
% initial_noise_level_x = 1.0;  % Adjust as needed
% initial_noise_level_u = 1.0;  % Adjust as needed
% prior_range_factor = 10;  % Adjust as needed
% 
% W_x_prior_mean_broad = diag([1e1 * prior_range_factor, 1e3 * prior_range_factor, 1e-2 * prior_range_factor, 1e-1 * prior_range_factor, 1e-5 * prior_range_factor, 1e3 * prior_range_factor, 1e1 * prior_range_factor, 1e-2 * prior_range_factor, 1e-1 * prior_range_factor, 1e-10 * prior_range_factor]);
% W_u_prior_mean_broad = diag([1e-3 * prior_range_factor, 1e2 * prior_range_factor, 1e-3 * prior_range_factor, 1e3 * prior_range_factor]);
% 
% particles_W_x = repmat(W_x_prior_mean_broad(:), 1, num_particles) + randn(size(W_x_prior_mean_broad(:), 1), num_particles) * initial_noise_level_x;
% particles_W_u = repmat(W_u_prior_mean_broad(:), 1, num_particles) + randn(size(W_u_prior_mean_broad(:), 1), num_particles) * initial_noise_level_u;
%%
% Initialize particles
% particles_W_x = repmat(W_x_prior_mean(:), 1, num_particles) + randn(size(W_x_prior_mean(:), 1), num_particles) * 100; %it was *0.1 before
% particles_W_u = repmat(W_u_prior_mean(:), 1, num_particles) + randn(size(W_u_prior_mean(:), 1), num_particles) * 100;

%%
% % Define higher and lower noise levels
% higher_noise_level = 100;
% lower_noise_level = 0.1;
% 
% % Initialize particles for W_x
% particles_W_x = repmat(W_x_prior_mean(:), 1, num_particles);
% particles_W_u = repmat(W_u_prior_mean(:), 1, num_particles);
% 
% % Apply higher noise to specific indices in W_x
% indices_higher_noise = [1, 2, 6, 7];
% particles_W_x(indices_higher_noise, :) = particles_W_x(indices_higher_noise, :) + randn(length(indices_higher_noise), num_particles) * higher_noise_level;
% 
% % Apply lower noise to the rest of the indices in W_x
% indices_lower_noise = setdiff(1:numel(W_x_prior_mean), indices_higher_noise);
% particles_W_x(indices_lower_noise, :) = particles_W_x(indices_lower_noise, :) + randn(length(indices_lower_noise), num_particles) * lower_noise_level;
% 
% % Apply lower noise to all indices in W_u
% particles_W_u = particles_W_u + randn(size(particles_W_u)) * lower_noise_level;
% Define higher and lower noise levels
higher_noise_level = 100;
lower_noise_level = 0.1;

% Initialize particles for W_x
particles_W_x = repmat(W_x_prior_mean(:), 1, num_particles);
particles_W_u = repmat(W_u_prior_mean(:), 1, num_particles);

% Indices for the higher noise elements in the diagonal of the 10x10 matrix
higher_noise_indices = [1, 12, 61, 72];  % Linear indices for (1,1), (2,2), (6,6), and (7,7) in 10x10 matrix

% Apply higher noise to specific diagonal elements in W_x for all particles
% for i = 1:num_particles
%     for idx = higher_noise_indices
%         particles_W_x(idx, i) = W_x_prior_mean(idx) + randn() * higher_noise_level;
%     end
% end
% Assuming higher_noise_indices are indices where we want higher noise
for i = 1:num_particles
    for idx = higher_noise_indices
        particles_W_x(idx, i) = W_x_prior_mean(idx) + randn() * higher_noise_level;
        particles_W_x(idx, i) = max(particles_W_x(idx, i), 0.1); % Ensure non-negative
    end
end

% Apply lower noise to the rest of the diagonal elements in W_x for all particles
all_diagonal_indices = 1:11:100;  % Indices of all diagonal elements in the 10x10 matrix
lower_noise_indices = setdiff(all_diagonal_indices, higher_noise_indices);
for i = 1:num_particles
    for idx = lower_noise_indices
        particles_W_x(idx, i) = W_x_prior_mean(idx) + randn() * lower_noise_level;
        particles_W_x(idx,i) = max(particles_W_x(idx,i),1e-2);
    end
end

% Apply lower noise to all elements in W_u for all particles
% particles_W_u = repmat(W_u_prior_mean(:), 1, num_particles) + randn(size(particles_W_u)) * 0.1;
%%
% Define indices where you want higher noise
standard_noise_level = 0.5;
higher_noise_level = 50.0;
higher_noise_indices_u = [6,16]; % Corresponds to indices in the 4x4 matrix (1-based indexing)

% Initialize particles with prior means and noise
particles_W_u = repmat(W_u_prior_mean(:), 1, num_particles);

% Add noise to the particles
for i = 1:num_particles
    % Create a noise vector for the current particle
    noise_vector = randn(size(particles_W_u(:, i))) * standard_noise_level;
    
    % Apply higher noise to specific indices
    noise_vector(higher_noise_indices_u) = randn(length(higher_noise_indices_u), 1) * higher_noise_level;
    
    % Add noise and ensure non-negativity
    particles_W_u(:, i) = max(particles_W_u(:, i) + noise_vector, 1e-2);
end
%%


%%
weights = ones(num_particles, 1) / num_particles;

% % Define likelihood function
% function likelihood = computeLikelihood(X_sim, U_sim, W_x, W_u)
%     % Run MPC with given W_x and W_u and compute cost
%     % Define your MPC setup here using CasADi
%     % ...
% 
%     % Assume you have a function `runMPC` that returns the cost given weights
%     cost = runMPC(X_sim, U_sim, W_x, W_u);
%     
%     % Define the likelihood (negative log-likelihood for simplicity)
%     % Assume some observed cost `observed_cost` from your MPC runs
%     observed_cost = computeObservedCost(X_sim, U_sim);
%     
%     likelihood = exp(-0.5 * (cost - observed_cost)^2);
% end


% Initialize the Parallel Pool
if isempty(gcp('nocreate'))
%     parpool;
    parpool('local', 4); % Set the number of workers to the number of particles
end

% Particle filter iterations
for iter = 1:num_iterations
    % Compute weights for each particle
%     for i = 1:num_particles
%         W_x = reshape(particles_W_x(:, i), size(W_x_prior_mean));
%         W_u = reshape(particles_W_u(:, i), size(W_u_prior_mean));
%         likelihoods(i) = computeLikelihood(X_sim, U_sim, W_x, W_u,state_ref);
%     end
 parfor i = 1:num_particles
        W_x = reshape(particles_W_x(:, i), size(W_x_prior_mean));
        W_u = reshape(particles_W_u(:, i), size(W_u_prior_mean));
        likelihoods(i) = computeLikelihood(X_sim, U_sim, W_x, W_u, state_ref);
%         fprintf('Particle %d computed by worker %d\n', i, getCurrentWorker().ID);
    end
    
    % Normalize weights
    weights = likelihoods / sum(likelihoods);
    
    % Resample particles based on weights
%     indices = randsample(1:num_particles, num_particles, true, weights);
    indices = systematicResample(weights);
    particles_W_x = particles_W_x(:, indices);
    particles_W_u = particles_W_u(:, indices);
    
    % Add some noise to the particles (optional)
    particles_W_x = particles_W_x + randn(size(particles_W_x)) * 0.01;
    particles_W_u = particles_W_u + randn(size(particles_W_u)) * 0.01;
end

% Compute the estimated weights
estimated_W_x = mean(particles_W_x, 2);
estimated_W_u = mean(particles_W_u, 2);

% Reshape to original form
estimated_W_x = reshape(estimated_W_x, size(W_x_prior_mean));
estimated_W_u = reshape(estimated_W_u, size(W_u_prior_mean));

% Display estimated weights
disp('Estimated W_x:');
disp(estimated_W_x);
disp('Estimated W_u:');
disp(estimated_W_u);
