function particles = systematicResample(particles)
    num_particles = size(particles.W_xu, 1);
    
    % Step 1: Compute the cumulative distribution
    cumulative_sum = cumsum(particles.weights);
    cumulative_sum(end) = 1;  % Ensure the last value is exactly 1
    
    % Step 2: Determine the step size and the random start point
    step = 1 / num_particles;
    start = rand() * step;
    
    % Step 3: Systematically sample the particles
    resample_indices = zeros(num_particles, 1);
    i = 1;
    for j = 1:num_particles
        u = start + (j - 1) * step;
        while u > cumulative_sum(i)
            i = i + 1;
        end
        resample_indices(j) = i;
    end
    
    % Step 4: Resample the particles
    particles.W_xu = particles.W_xu(resample_indices, :);
    particles.weights = ones(num_particles, 1) / num_particles;  % Reset weights
end