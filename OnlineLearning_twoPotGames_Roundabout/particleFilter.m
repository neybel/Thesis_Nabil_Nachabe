% Main Particle Filter Function
function [particles,estimated_weights,ESS] = particleFilter(X_sim,U_sim,x0, N, num_particles, W_xu_ranges, W_xu1,ocp,initial_guess,range_factor,car,std_dev,testtheta1,testtheta2,D1,D2)
    % Step 1: Initialize particles
    particles = initializeParticles1(num_particles, W_xu_ranges,initial_guess,range_factor,std_dev);
    
    % Step 2: Prediction
    [predicted_trajectories,predicted_controls] = predictTrajectories(particles, x0, N, W_xu1,ocp,car,testtheta1,testtheta2,D1,D2);
    
    % Step 3: Update
    [particles,ESS] = updateWeights(particles, X_sim, predicted_trajectories,U_sim,predicted_controls);
    
    % Step 4: Resampling
  [particles,pre_resample_weights,resampled_weights] = resampleParticles(particles);
%   particles = systematicResample(particles);
%   particles = residualResample(particles); %combines systematic and multi
%     particles = stratifiedResample(particles);
    
    % Step 5: Estimation
    estimated_weights = estimateWeights(particles);
end
