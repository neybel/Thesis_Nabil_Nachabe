function plotMinDistance(x_mpc, numIterations)
    % Initialize distance array
    distances = zeros(1, numIterations);
    
    for i = 1:numIterations
        % Extract positions for both agents at iteration i
        x1 = x_mpc(1, i); % x position of agent 1
        y1 = x_mpc(2, i); % y position of agent 1
        x2 = x_mpc(6, i); % x position of agent 2
        y2 = x_mpc(7, i); % y position of agent 2
        
        % Calculate the Euclidean distance between the two agents
        distances(i) = sqrt((x2 - x1)^2 + (y2 - y1)^2);
    end

    % Find the minimum distance and the iteration at which it occurs
    [min_distance, min_index] = min(distances);
    
    % Plot the distance over iterations
    figure;
    plot(1:numIterations, distances, 'LineWidth', 2);
    hold on;
    
    % Mark the minimum distance point
    plot(min_index, min_distance, 'ro', 'MarkerSize', 8, 'LineWidth', 2);
    
    % Display the value of the minimum distance on the plot
    text(min_index, min_distance, ['Min: ', num2str(min_distance, '%.2f'), ' m'], ...
         'VerticalAlignment', 'top', 'HorizontalAlignment', 'left', 'FontSize', 12);
    
    % Label the plot
    xlabel('Iteration');
    ylabel('Distance between Agents (meters)');
    title('Distance between Two Agents over Iterations');
    grid on;
    grid minor;
    hold off;
end
