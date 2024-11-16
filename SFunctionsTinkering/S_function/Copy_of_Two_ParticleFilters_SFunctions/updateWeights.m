function [particles,ESS,likelihoods] = updateWeights(particles, X_sim, predicted_trajectories,U_sim,predicted_controls)
    num_particles = size(particles.W_xu, 1);
    likelihoods = zeros(num_particles, 1);
    total_error=zeros(num_particles,1);
    scaling_factor=zeros(num_particles,1);
    for i = 1:num_particles
        % Compute likelihood of observed trajectory given the predicted trajectory
%         likelihoods(i) = computeLikelihood1(X_sim, predicted_trajectories{i});
%         likelihoods(i) = computeLikelihood_Euclidean(X_sim, predicted_trajectories{i});
%           likelihoods(i) = computeLikelihoodTest(X_sim, predicted_trajectories{i});


%         [likelihoods(i),total_error(i),scaling_factor(i)] = computeLikelihoodWithControl(X_sim, predicted_trajectories{i},U_sim,predicted_controls{i});
                [likelihoods(i),total_error(i),scaling_factor(i)] = computeLikelihoodWithControl_noisy(X_sim, predicted_trajectories{i},U_sim,predicted_controls{i});



    end
    
    % Update particle weights
    particles.weights = particles.weights .* likelihoods;
    
    % Normalize weights
    particles.weights = particles.weights / sum(particles.weights);
    ESS=1/sum(particles.weights.^2); %% Effetive Sample Size
end
