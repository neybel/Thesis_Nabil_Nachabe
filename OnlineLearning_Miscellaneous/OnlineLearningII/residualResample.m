function particles = residualResample(particles)
    num_particles = size(particles.W_xu, 1);
    weights = particles.weights;
    num_residuals = floor(weights * num_particles);
    residual_particles = mod(weights * num_particles, 1);

    % Initialize arrays for resampling
    resample_indices = zeros(num_particles, 1);

    % Add integer number of particles
    idx = 1;
    for i = 1:num_particles
        resample_indices(idx:idx + num_residuals(i) - 1) = i;
        idx = idx + num_residuals(i);
    end

    % Resample remaining particles
    residual_indices = randsample(num_particles, num_particles - sum(num_residuals), true, residual_particles);
    resample_indices(idx:end) = residual_indices;

    % Update particles
    particles.W_xu = particles.W_xu(resample_indices, :);
    particles.weights = ones(num_particles, 1) / num_particles; % Reset weights
end
