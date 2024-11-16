function particles = initializeParticles1(num_particles, W_xu_ranges,initial_guess,range_factor,std_dev)
    num_ranges = size(W_xu_ranges, 1);  % Number of parameters
    W_xu_particles = zeros(num_particles, num_ranges);

    % mean and standard deviation for each parameter, the mean could be
    % iteratively updated with the previous estimate, and the std_dev can
    % be itratively intentionally made smaller. 
%     initial_guess = [3000; 3000; 1e-2; 1e-2; 1e-5;0.005;0.5];
%     std_dev = [1e3; 1e3; 1e-3; 1e-3; 1e-6; 1e-2;0.25];

%     std_dev = [2e3; 2e3; 1e-3; 1e-3; 1e-6; 1e-1;0.25];
%     std_dev = [9e3; 9e3; 1e-3; 1e-3; 1e-6; 1;1];

    std_dev=std_dev*range_factor;

    
    for i = 1:num_ranges
        % Define the mean and standard deviation for the Gaussian distribution for parameter i
        mean_value = initial_guess(i)    % Mean for the Gaussian distribution
        std_deviation = std_dev(i);          % Standard deviation for the Gaussian distribution

        % Generate random particles from a Gaussian distribution
        W_xu_particles(:, i) = mean_value + std_deviation * randn(num_particles, 1)
        
        % Clip values to be within the specified range
        W_xu_particles(:, i) = max(W_xu_ranges(i, 1), min(W_xu_ranges(i, 2), W_xu_particles(:, i)));
    end
    %%% make the x and y the same since we have that knowledge (testing this)
    W_xu_particles(:,2)=W_xu_particles(:,1);
    %%%
    
    % Initialize weights uniformly
    weights = ones(num_particles, 1) / num_particles;
    
    particles.W_xu = W_xu_particles;
    particles.weights = weights;
end
