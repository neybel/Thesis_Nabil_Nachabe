function particles = initializeParticles(num_particles, W_x_ranges)
    % Initialize particles randomly within the given ranges for W_x and W_u
%     W_x_particles = rand(num_particles, 1) * (W_x_range(2) - W_x_range(1)) + W_x_range(1);
%     W_u_particles = rand(num_particles, 1) * (W_u_range(2) - W_u_range(1)) + W_u_range(1);
% Initialize particles randomly within the given ranges for W_x components
    num_ranges = size(W_x_ranges, 1);  % Number of parameters
    W_x_particles = zeros(num_particles, size(W_x_ranges,1));

     for i = 1:num_ranges
        % Generate random particles within the specified range
        W_x_particles(:, i) = rand(num_particles, 1) * ...
                              (W_x_ranges(i, 2) - W_x_ranges(i, 1)) + W_x_ranges(i, 1);
    end
    
    % Initialize weights uniformly
    weights = ones(num_particles, 1) / num_particles;
    
    particles.W_x = W_x_particles;
%     particles.W_u = W_u_particles;
    particles.weights = weights;
end
