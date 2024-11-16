function predicted_trajectories = predictTrajectories(particles, x0, N, W_u,ocp)
num_particles = size(particles.W_x, 1);
    predicted_trajectories = cell(num_particles, 1);
    
    for i = 1:num_particles
        W_x = particles.W_x(i, :);  % Current particle's W_x
        W_x=W_x';
        
        % Solve OCP for the current particle's W_x and fixed W_u
        predicted_trajectories{i} = OCP_solve(ocp, x0, W_x, W_u);
    end
end