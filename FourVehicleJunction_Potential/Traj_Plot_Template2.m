figure(100)
road_width = 40;
road_length = 200;
merging_lane_width = 40;
merging_lane_position = 100;

% Main road
rectangle('Position', [0, -road_width/2, road_length, road_width], 'FaceColor', [0, 1, 1, 0.5]);
hold on;

% Merging lane (optional, can uncomment if needed)
% rectangle('Position', [merging_lane_position - merging_lane_width/2, -road_width, merging_lane_width, road_width], 'FaceColor', 'r');
rectangle('Position', [85 -80 40 140], 'FaceColor', [0, 1, 0, 0.5]);

% Plot trajectories for four agents
plot(x_sim(1,:), x_sim(2,:), 'r*', 'LineWidth', 2);  % Agent 1
plot(x_sim(6,:), x_sim(7,:), 'g*', 'LineWidth', 2);  % Agent 2
plot(x_sim(11,:), x_sim(12,:), 'b*', 'LineWidth', 2); % Agent 3
plot(x_sim(16,:), x_sim(17,:), 'm*', 'LineWidth', 2); % Agent 4

% Define more frequent time steps (5 times more)
time_steps = round(linspace(1, length(x_sim(1,:)), 6)); % More frequent sampling

% Represent cars as rectangles at specific time steps
car_length = 2; % Length of the car representation
car_width = 1;  % Width of the car representation

for i = time_steps
    % Draw Agent 1
    rectangle('Position', [x_sim(1,i)-car_length/2, x_sim(2,i)-car_width/2, car_length, car_width], ...
              'FaceColor', 'r', 'EdgeColor', 'k', 'LineWidth', 1.5);
    
    % Draw Agent 2
    rectangle('Position', [x_sim(6,i)-car_length/2, x_sim(7,i)-car_width/2, car_length, car_width], ...
              'FaceColor', 'g', 'EdgeColor', 'k', 'LineWidth', 1.5);
    
    % Draw Agent 3
    rectangle('Position', [x_sim(11,i)-car_length/2, x_sim(12,i)-car_width/2, car_length, car_width], ...
              'FaceColor', 'b', 'EdgeColor', 'k', 'LineWidth', 1.5);
    
    % Draw Agent 4
    rectangle('Position', [x_sim(16,i)-car_length/2, x_sim(17,i)-car_width/2, car_length, car_width], ...
              'FaceColor', 'm', 'EdgeColor', 'k', 'LineWidth', 1.5);
    
    % Annotate timestep numbers at each car position
    text(x_sim(1,i), x_sim(2,i), num2str(i), 'Color', 'r', 'FontSize', 10, 'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'center');
    text(x_sim(6,i), x_sim(7,i), num2str(i), 'Color', 'g', 'FontSize', 10, 'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'center');
    text(x_sim(11,i), x_sim(12,i), num2str(i), 'Color', 'b', 'FontSize', 10, 'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'center');
    text(x_sim(16,i), x_sim(17,i), num2str(i), 'Color', 'm', 'FontSize', 10, 'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'center');
end

% Set axes properties
% axis equal;
% xlim([min(x_sim(1,:)) - 10, max(x_sim(1,:)) + 10]);
% ylim([min(x_sim(2,:)) - 10, max(x_sim(2,:)) + 10]);
xlabel('x [m]');
ylabel('y [m]');
title('Car Trajectories');
grid on;
