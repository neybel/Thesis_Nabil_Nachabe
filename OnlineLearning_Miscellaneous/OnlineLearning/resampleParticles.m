function particles = resampleParticles(particles)
    num_particles = size(particles.W_x, 1);
    
    % Resample indices according to weights
    resample_indices = randsample(1:num_particles, num_particles, true, particles.weights);
    
    % Resample particles
    particles.W_x = particles.W_x(resample_indices, :);
    particles.weights = ones(num_particles, 1) / num_particles; % Reset weights
end