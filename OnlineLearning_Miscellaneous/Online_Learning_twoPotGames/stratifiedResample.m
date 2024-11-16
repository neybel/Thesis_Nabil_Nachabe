function particles = stratifiedResample(particles)
    num_particles = size(particles.W_xu, 1);
    cumulative_weights = cumsum(particles.weights);
    stratified_samples = (rand(num_particles, 1) + (0:num_particles-1)') / num_particles;
    
    indices = zeros(num_particles, 1);
    j = 1;
    
    for i = 1:num_particles
        while stratified_samples(i) > cumulative_weights(j)
            j = j + 1;
        end
        indices(i) = j;
    end
    
    particles.W_xu = particles.W_xu(indices, :);
    particles.weights = ones(num_particles, 1) / num_particles; % Reset weights
end
