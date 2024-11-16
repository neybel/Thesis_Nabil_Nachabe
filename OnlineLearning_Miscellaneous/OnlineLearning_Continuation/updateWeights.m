function particles = updateWeights(particles, X_sim, predicted_trajectories)
    num_particles = size(particles.W_x, 1);
    likelihoods = zeros(num_particles, 1);
    
    for i = 1:num_particles
        % Compute likelihood of observed trajectory given the predicted trajectory
        likelihoods(i) = computeLikelihood1(X_sim, predicted_trajectories{i});
%           likelihoods(i) = computeLikelihoodTest(X_sim, predicted_trajectories{i});
    end
    
    % Update particle weights
    particles.weights = particles.weights .* likelihoods;
    
    % Normalize weights
    particles.weights = particles.weights / sum(particles.weights);
end
