% Main Particle Filter Function
function estimated_weights = particleFilter(X_sim,x0, N, num_particles, W_x_ranges, W_u,ocp)
    % Step 1: Initialize particles
    particles = initializeParticles(num_particles, W_x_ranges);
    
    % Step 2: Prediction
    predicted_trajectories = predictTrajectories(particles, x0, N, W_u,ocp);
    
    % Step 3: Update
    particles = updateWeights(particles, X_sim, predicted_trajectories);
    
    % Step 4: Resampling
    particles = resampleParticles(particles);
    
    % Step 5: Estimation
    estimated_weights = estimateWeights(particles);
end
