function estimated_weights = estimateWeights(particles)
    estimated_W_xu = sum(particles.W_xu .* particles.weights,1);
%     estimated_W_u = sum(particles.W_u .* particles.weights);
    
    estimated_weights.W_xu = estimated_W_xu;
%     estimated_weights.W_u = estimated_W_u;
end
