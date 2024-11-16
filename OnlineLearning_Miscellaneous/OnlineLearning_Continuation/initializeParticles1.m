function particles = initializeParticles1(num_particles, W_x_ranges)
    num_ranges = size(W_x_ranges, 1);  % Number of parameters
    W_x_particles = zeros(num_particles, num_ranges);

    % mean and standard deviation for each parameter, the mean could be
    % iteratively updated with the previous estimate, and the std_dev can
    % be itratively intentionally made smaller. 
    initial_guess = [3000; 3000; 1e-2; 1e-2; 1e-5; 3000; 3000; 1e-2; 1e-2; 1e-10];
    std_dev = [1e3; 1e3; 1e-3; 1e-3; 1e-6; 1e3; 1e3; 1e-3; 1e-3; 1e-7];
    
    for i = 1:num_ranges
        % Define the mean and standard deviation for the Gaussian distribution for parameter i
        mean_value = initial_guess(i);       % Mean for the Gaussian distribution
        std_deviation = std_dev(i);          % Standard deviation for the Gaussian distribution

        % Generate random particles from a Gaussian distribution
        W_x_particles(:, i) = mean_value + std_deviation * randn(num_particles, 1);
        
        % Clip values to be within the specified range
        W_x_particles(:, i) = max(W_x_ranges(i, 1), min(W_x_ranges(i, 2), W_x_particles(:, i)));
    end
    
    % Initialize weights uniformly
    weights = ones(num_particles, 1) / num_particles;
    
    particles.W_x = W_x_particles;
    particles.weights = weights;
end
