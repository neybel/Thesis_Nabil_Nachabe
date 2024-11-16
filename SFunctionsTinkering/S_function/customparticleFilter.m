% Main Particle Filter Function
function [particles,estimated_weights,ESS,likelihoods] = customparticleFilter(X_sim,U_sim,x0, N, num_particles, W_xu_ranges, W_xu1,ocp,initial_guess,range_factor,car,std_dev,testtheta1,testtheta2,pf,D1,D2,K,k)
    % Step 1: Initialize particles
    particles = initializeParticles1(num_particles, W_xu_ranges,initial_guess,range_factor,std_dev);
    
    % Step 2: Prediction
    [predicted_trajectories,predicted_controls] = custompredictTrajectories(particles, x0, N, W_xu1,ocp,car,testtheta1,testtheta2,pf,D1,D2,K);
    
    % Step 3: Update
    [particles,ESS,likelihoods] = customupdateWeights(particles, X_sim, predicted_trajectories,U_sim,predicted_controls);
         
    
    
    
    %%%% Plottinf for motinoring purposes
  if k==150
        plotRoundaboutWithLanes()
    hold on
    for u=1:num_particles
    plot(predicted_trajectories{u}(1,:),predicted_trajectories{u}(2,:));
    hold on
    end
    hold on
    plot(X_sim(1,:),X_sim(2,:),'yo')
%     keyboard
  end
    %%%%%
    % Step 4: Resampling
  [particles,pre_resample_weights,resampled_weights] = resampleParticles(particles);
%   particles = systematicResample(particles);
%   particles = residualResample(particles); %combines systematic and multi
%     particles = stratifiedResample(particles);
    
    % Step 5: Estimation
    estimated_weights = estimateWeights(particles);
end
