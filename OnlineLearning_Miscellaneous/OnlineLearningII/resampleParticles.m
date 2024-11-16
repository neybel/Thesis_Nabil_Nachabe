function [particles,pre_resample_weights,resampled_weights] = resampleParticles(particles)
%%%% multinomial resampling technique here. Consider checking the
%%%% systematic resampling which could be more efficeint and more stable!
    pre_resample_weights = particles.weights;

%%% Resamplign is important since: address the issue of particle degeneracy, where most particles may end up with very low weights and only a few have significant weight.
    num_particles = size(particles.W_xu, 1);
    
    % Resample indices according to weights
    resample_indices = randsample(1:num_particles, num_particles, true, particles.weights);
    
    % Resample particles
    particles.W_xu = particles.W_xu(resample_indices, :);
    particles.weights = ones(num_particles, 1) / num_particles; % Reset weights

     % Return resampled weights for visualization
    resampled_weights = particles.weights;
end