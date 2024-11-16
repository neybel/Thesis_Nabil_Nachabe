function estimated_weights = estimateWeights(particles)
    estimated_W_x = sum(particles.W_x .* particles.weights,1);
%     estimated_W_u = sum(particles.W_u .* particles.weights);
    
    estimated_weights.W_x = estimated_W_x;
%     estimated_weights.W_u = estimated_W_u;
end