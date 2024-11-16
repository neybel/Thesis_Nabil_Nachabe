function plotMinDistance(x_mpc, numIterations)
    % Initialize distance arrays for 6 pairs of agents
    distances = zeros(6, numIterations);
    
    % Loop through iterations to calculate distances for all agent pairs
    for i = 1:numIterations
        % Extract positions for all 4 agents at iteration i
        x1 = x_mpc(1, i); y1 = x_mpc(2, i);  % Agent 1
        x2 = x_mpc(6, i); y2 = x_mpc(7, i);  % Agent 2
        x3 = x_mpc(11, i); y3 = x_mpc(12, i); % Agent 3
        x4 = x_mpc(16, i); y4 = x_mpc(17, i); % Agent 4
        
        % Calculate distances between all pairs of agents
        distances(1, i) = sqrt((x2 - x1)^2 + (y2 - y1)^2);  % Agent 1 - Agent 2
        distances(2, i) = sqrt((x3 - x1)^2 + (y3 - y1)^2);  % Agent 1 - Agent 3
        distances(3, i) = sqrt((x4 - x1)^2 + (y4 - y1)^2);  % Agent 1 - Agent 4
        distances(4, i) = sqrt((x3 - x2)^2 + (y3 - y2)^2);  % Agent 2 - Agent 3
        distances(5, i) = sqrt((x4 - x2)^2 + (y4 - y2)^2);  % Agent 2 - Agent 4
        distances(6, i) = sqrt((x4 - x3)^2 + (y4 - y3)^2);  % Agent 3 - Agent 4
    end

    % Create a figure
    figure;
    
    % Define colors for each plot
    colors = {'b', 'r', 'g', 'c', 'm', 'k'};
    
    % Plot each distance
    hold on;
    plot(1:numIterations, distances(1, :), 'LineWidth', 2, 'Color', colors{1}); % Agent 1 - 2
    plot(1:numIterations, distances(2, :), 'LineWidth', 2, 'Color', colors{2}); % Agent 1 - 3
    plot(1:numIterations, distances(3, :), 'LineWidth', 2, 'Color', colors{3}); % Agent 1 - 4
    plot(1:numIterations, distances(4, :), 'LineWidth', 2, 'Color', colors{4}); % Agent 2 - 3
    plot(1:numIterations, distances(5, :), 'LineWidth', 2, 'Color', colors{5}); % Agent 2 - 4
    plot(1:numIterations, distances(6, :), 'LineWidth', 2, 'Color', colors{6}); % Agent 3 - 4

    % Annotate with arrows for identical graphs
    text(numIterations * 0.9, max(distances(1, :)) * 0.9, 'D_{1,2}', 'Color', colors{1}, 'FontSize', 12, 'HorizontalAlignment', 'center');
    text(numIterations * 0.9, max(distances(2, :)) * 0.8, 'D_{1,3}', 'Color', colors{2}, 'FontSize', 12, 'HorizontalAlignment', 'center');
    text(numIterations * 0.9, max(distances(3, :)) * 0.7, 'D_{1,4}', 'Color', colors{3}, 'FontSize', 12, 'HorizontalAlignment', 'center');
    text(numIterations * 0.9, max(distances(4, :)) * 0.6, 'D_{2,3}', 'Color', colors{4}, 'FontSize', 12, 'HorizontalAlignment', 'center');
    text(numIterations * 0.9, max(distances(5, :)) * 0.5, 'D_{2,4}', 'Color', colors{5}, 'FontSize', 12, 'HorizontalAlignment', 'center');
    text(numIterations * 0.9, max(distances(6, :)) * 0.4, 'D_{3,4}', 'Color', colors{6}, 'FontSize', 12, 'HorizontalAlignment', 'center');

    % Label the plot
    xlabel('Iteration');
    ylabel('Distance between Agents (meters)');
    title('Distances between Agents over Iterations');
    grid on;
    grid minor;
    hold off;
end
