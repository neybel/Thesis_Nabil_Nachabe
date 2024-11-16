clc
close all
figure(100)
% Plot the starting positions of agents
start_pos_agent1 = [x_sim(1,1), x_sim(2,1)];  % Initial position of Agent 1
start_pos_agent2 = [x_sim(6,1), x_sim(7,1)];  % Initial position of Agent 2
   road_width = 20;
road_length = 150;
merging_lane_width = 10;
merging_lane_position = 100;


% Main road
rectangle('Position', [0, -road_width/2, road_length, road_width], 'FaceColor', [0, 1, 1, 0.5],'HandleVisibility', 'off');

% Merging lane
% rectangle('Position', [merging_lane_position - merging_lane_width/2, -road_width, merging_lane_width, road_width], 'FaceColor', 'r');
rectangle('Position', [100 -50 20 40], 'FaceColor', [0, 1, 0, 0.5],'HandleVisibility', 'off');
hold on 

% Plot lane markings at the center, top, and bottom of the road
% plot([0 road_length], [0 0], 'w--', 'LineWidth', 2);            % Center dashed line
% plot([0 road_length], [-road_width/3, -road_width/3], 'w--', 'LineWidth', 2);  % Bottom lane marking
plot([0 road_length], [road_width/20, road_width/20], 'w--', 'LineWidth', 2);    % Top lane marking

% Plot agent trajectories with transparency (alpha)

h1=plot(x_sim(1,:), x_sim(2,:), 'r-', 'LineWidth', 6, 'Color', [1, 0, 0, 0.6]);  % Agent 1 (Red, 30% opacity)
h2=plot(x_sim(6,:), x_sim(7,:), 'g-', 'LineWidth', 6, 'Color', [0, 1, 0, 0.6]);  % Agent 2 (Green, 30% opacity)
% legend({'Agent 1 (Red)', 'Agent 2 (Green)'}, 'Location', 'Best');  % Improved legend
legend([h1, h2], {'Agent 1', 'Agent 2'},'Location','best');


% Setting axis properties
axis equal;
xlim([min([x_sim(1,:), x_sim(6,:)]) - 10, max([x_sim(1,:), x_sim(6,:)]) + 10]);
ylim([-road_width, road_width] * 1.5);  % Adjust based on road width for margin

% Labels and title
xlabel('x [m]', 'FontSize', 12);
ylabel('y [m]', 'FontSize', 12);
title('Car Trajectories with Road and Lane Markings', 'FontSize', 14);

% Grid and legend
grid on;
grid minor;

% Car dimensions
car_length = 4;  % Length of the car representation
car_width = 2;   % Width of the car representation

ttt=5;
% Define more frequent time steps
time_steps = round(linspace(1, length(x_sim(1,:)), ttt));  % More frequent sampling

% Offset to avoid cars overlapping when they are close
offset_y_agent2 = 0.1;  % Slight vertical offset for Agent 2
for i = time_steps
    % Car for Agent 1
    theta = x_sim(3,i);  % Orientation angle for Agent 1
    rect_x = [-car_length/2, car_length/2, car_length/2, -car_length/2];
    rect_y = [-car_width/2, -car_width/2, car_width/2, car_width/2];
    
    % Rotation and translation for Agent 1
    rotation_matrix = [cos(theta), -sin(theta); sin(theta), cos(theta)];
    rotated_corners_agent1 = rotation_matrix * [rect_x; rect_y];
    translated_corners_agent1 = rotated_corners_agent1 + [x_sim(1,i); x_sim(2,i)];
    
    fill(translated_corners_agent1(1,:), translated_corners_agent1(2,:), 'r', 'EdgeColor', 'k', 'LineWidth', 1.5,'HandleVisibility', 'off');
    
    % Add quiver for Agent 1
    quiver(x_sim(1,i), x_sim(2,i), cos(theta) * car_length, sin(theta) * car_length, ...
           'Color', 'k', 'LineWidth', 1.5, 'MaxHeadSize', 1,'HandleVisibility', 'off');
    
    % Add time step for Agent 1 (e.g., slightly above the car)
    text(x_sim(1,i), x_sim(2,i) + 2.50, sprintf('t=%d', i), 'Color', 'r', 'FontSize', 10, 'HorizontalAlignment', 'center');

    % Car for Agent 2
    theta_agent2 = x_sim(8,i);  % Use theta_agent2 directly
    rotation_matrix_agent2 = [cos(theta_agent2), -sin(theta_agent2); sin(theta_agent2), cos(theta_agent2)];
    rotated_corners_agent2 = rotation_matrix_agent2 * [rect_x; rect_y];
    translated_corners_agent2 = rotated_corners_agent2 + [x_sim(6,i); x_sim(7,i) + offset_y_agent2];  % Apply offset

    % Fill and quiver for Agent 2
    fill(translated_corners_agent2(1,:), translated_corners_agent2(2,:), 'g', 'EdgeColor', 'k', 'LineWidth', 1.5,'HandleVisibility', 'off');
    quiver(x_sim(6,i), x_sim(7,i) + offset_y_agent2, cos(theta_agent2) * car_length, sin(theta_agent2) * car_length, ...
           'Color', 'k', 'LineWidth', 1.5, 'MaxHeadSize', 1,'HandleVisibility', 'off');

    % Add time step for Agent 2 (e.g., slightly below the car)
    text(x_sim(6,i), x_sim(7,i) + offset_y_agent2 - 2.0, sprintf('t=%d', i), 'Color', 'g', 'FontSize', 10, 'HorizontalAlignment', 'center');
end


%%
% return
% Adding text labels instead of legend
% text(road_length + 5, road_width / 4, 'Agent 1 (Red)', 'Color', 'r', 'FontSize', 10, 'HorizontalAlignment', 'left');
% text(road_length + 5, -road_width / 4, 'Agent 2 (Green)', 'Color', 'g', 'FontSize', 10, 'HorizontalAlignment', 'left');

hold off;
% legend({'Agent 1 (Red)', 'Agent 2 (Green)'}, 'Location', 'Best');  % Improved legend

plotMinDistance(x_sim,N_sim);

