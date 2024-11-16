clc
close all
figure(100)
% Plot the starting positions of agents
start_pos_agent1 = [x_sim(1,1), x_sim(2,1)];  % Initial position of Agent 1
start_pos_agent2 = [x_sim(6,1), x_sim(7,1)];  % Initial position of Agent 2

% % Define road properties
road_length = 250;  % Length of the road
road_width = 40;    % Width of the road
lane_marking_offset = 5;  % Distance of lane markings from the center

% % Plot the road as a rectangle (centered at y=0)
% rectangle('Position', [-10, -road_width/2, road_length, road_width], 'FaceColor', [0.7 0.7 0.7], 'EdgeColor', 'none');
% hold on;

% % Plot lane markings at the center, top, and bottom of the road
% % plot([0 road_length], [0 0], 'w--', 'LineWidth', 2);            % Center dashed line
% plot([0 road_length], [-road_width/4, -road_width/4], 'w--', 'LineWidth', 2);  % Bottom lane marking
% plot([0 road_length], [road_width/4, road_width/4], 'w--', 'LineWidth', 2);    % Top lane marking

% Plot agent trajectories with transparency (alpha)

plot(x_sim(1,:), x_sim(2,:), 'r-', 'LineWidth', 6, 'Color', [1, 0, 0, 0.6]);  % Agent 1 (Red, 30% opacity)
plot(x_sim(6,:), x_sim(7,:), 'g-', 'LineWidth', 6, 'Color', [0, 1, 0, 0.6]);  % Agent 2 (Green, 30% opacity)



% Setting axis properties
axis equal;
% xlim([min([x_sim(1,:), x_sim(6,:)]) - 10, max([x_sim(1,:), x_sim(6,:)]) + 10]);
% ylim([-road_width, road_width] * 1.5);  % Adjust based on road width for margin

% Labels and title
xlabel('x [m]', 'FontSize', 12);
ylabel('y [m]', 'FontSize', 12);
title('Car Trajectories with Road and Lane Markings', 'FontSize', 14);

% Grid and legend
grid on;
grid minor;

% Car dimensions
car_length = 0.50;  % Length of the car representation
car_width = 0.50;   % Width of the car representation

ttt=3;
% Define more frequent time steps
time_steps = round(linspace(1, length(x_sim(1,:)), ttt));  % More frequent sampling

% Offset to avoid cars overlapping when they are close
offset_y_agent2 = 0.1;  % Slight vertical offset for Agent 2

for i = time_steps
    % Car for Agent 1
    theta = x_sim(3,i);  % Orientation angle for Agent 1
    rect_x = [-car_length/2, car_length/2, car_length/2, -car_length/2];
    rect_y = [-car_width/2, -car_width/2, car_width/2, car_width/2];
    rotation_matrix = [cos(theta), -sin(theta); sin(theta), cos(theta)];
    rotated_corners = rotation_matrix * [rect_x; rect_y];
    translated_corners = rotated_corners + [x_sim(1,i); x_sim(2,i)];
    fill(translated_corners(1,:), translated_corners(2,:), 'r', 'EdgeColor', 'k', 'LineWidth', 1.5);
    quiver(x_sim(1,i), x_sim(2,i), cos(theta) * car_length, sin(theta) * car_length, ...
           'Color', 'k', 'LineWidth', 1.5, 'MaxHeadSize', 1);

    % Car for Agent 2 (with 180-degree flip and offset)
% Car for Agent 2 (without 180-degree flip)
% Car for Agent 2 with 180-degree rotation
theta_agent2 = x_sim(8,i);  % The heading angle of Agent 2
rotation_matrix_agent2 = [cos(theta_agent2), -sin(theta_agent2); sin(theta_agent2), cos(theta_agent2)];
rotated_corners_agent2 = rotation_matrix_agent2 * [rect_x; rect_y];
translated_corners_agent2 = rotated_corners_agent2 + [x_sim(6,i); x_sim(7,i) + offset_y_agent2];  % Apply offset

% Draw the car
fill(translated_corners_agent2(1,:), translated_corners_agent2(2,:), 'g', 'EdgeColor', 'k', 'LineWidth', 1.5);

% Quiver for Agent 2 (correct arrow direction)
quiver(x_sim(6,i), x_sim(7,i) + offset_y_agent2, ...
       -cos(theta_agent2) * car_length, -sin(theta_agent2) * car_length, ...  % Negating to flip direction
       'Color', 'k', 'LineWidth', 1.5, 'MaxHeadSize', 1);



    % Add step labels for both agents
    text(x_sim(1,i), x_sim(2,i) + 2, num2str(i), 'Color', 'r', 'FontSize', 10, 'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'center');
    text(x_sim(6,i), x_sim(7,i) + offset_y_agent2 + 2, num2str(i), 'Color', 'g', 'FontSize', 10, 'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'center');
end
clc
close all
figure(100)

% Define road properties
road_length = 150;  % Length of the road
road_width = 40;    % Width of the road
lane_marking_offset = 5;  % Distance of lane markings from the center

% Plot the road as a rectangle (centered at y=0)
% rectangle('Position', [-10, -road_width/2, road_length, road_width], 'FaceColor', [0.7 0.7 0.7], 'EdgeColor', 'none');
hold on;

% % Plot lane markings at the center, top, and bottom of the road
% % plot([0 road_length], [0 0], 'w--', 'LineWidth', 2);            % Center dashed line
% plot([-10 road_length], [-road_width/4, -road_width/4], 'w--', 'LineWidth', 2);  % Bottom lane marking
% plot([-10 road_length], [road_width/4, road_width/4], 'w--', 'LineWidth', 2);    % Top lane marking

% Plot agent trajectories with transparency (alpha)
plot(x_sim(1,:), x_sim(2,:), 'r-', 'LineWidth', 6, 'Color', [1, 0, 0, 0.6]);  % Agent 1 (Red, 30% opacity)
plot(x_sim(6,:), x_sim(7,:), 'g-', 'LineWidth', 6, 'Color', [0, 1, 0, 0.6]);  % Agent 2 (Green, 30% opacity)

% Setting axis properties
axis equal;
% xlim([min([x_sim(1,:), x_sim(6,:)]) - 10, max([x_sim(1,:), x_sim(6,:)]) + 10]);
% ylim([-road_width, road_width] * 1.5);  % Adjust based on road width for margin

% Labels and title
xlabel('x [m]', 'FontSize', 12,'Interpreter','latex');
ylabel('y [m]', 'FontSize', 12,'Interpreter','latex');
title('Car Trajectories with Road and Lane Markings', 'FontSize', 14,'Interpreter','latex');

% Grid and legend
grid on;
grid minor;

% Car dimensions
% car_length = 4;  % Length of the car representation
% car_width = 2;   % Width of the car representation

% Define more frequent time steps
time_steps = round(linspace(1, length(x_sim(1,:)), ttt));  % More frequent sampling

% Offset to avoid cars overlapping when they are close
offset_y_agent2 = 0.1;  % Slight vertical offset for Agent 2

for i = time_steps
    % Car for Agent 1
    theta = x_sim(3,i);  % Orientation angle for Agent 1
    rect_x = [-car_length/2, car_length/2, car_length/2, -car_length/2];
    rect_y = [-car_width/2, -car_width/2, car_width/2, car_width/2];
    rotation_matrix = [cos(theta), -sin(theta); sin(theta), cos(theta)];
    rotated_corners = rotation_matrix * [rect_x; rect_y];
    translated_corners = rotated_corners + [x_sim(1,i); x_sim(2,i)];
    fill(translated_corners(1,:), translated_corners(2,:), 'r', 'EdgeColor', 'k', 'LineWidth', 1.5);
    quiver(x_sim(1,i), x_sim(2,i), cos(theta) * car_length, sin(theta) * car_length, ...
           'Color', 'k', 'LineWidth', 1.5, 'MaxHeadSize', 1);

    % Car for Agent 2 (with 180-degree flip and offset)
    theta_agent2 = x_sim(8,i) + pi;
    rotation_matrix_agent2 = [cos(theta_agent2), -sin(theta_agent2); sin(theta_agent2), cos(theta_agent2)];
    rotated_corners_agent2 = rotation_matrix_agent2 * [rect_x; rect_y];
    translated_corners_agent2 = rotated_corners_agent2 + [x_sim(6,i); x_sim(7,i) + offset_y_agent2];  % Apply offset
    
    fill(translated_corners_agent2(1,:), translated_corners_agent2(2,:), 'g', 'EdgeColor', 'k', 'LineWidth', 1.5);
    quiver(x_sim(6,i), x_sim(7,i) + offset_y_agent2, ...
       cos(theta_agent2 + pi) * car_length, sin(theta_agent2 + pi) * car_length, ...  % Flipping by +pi for reverse direction
       'Color', 'k', 'LineWidth', 1.5, 'MaxHeadSize', 1);
    % Add step labels for both agents
    text(x_sim(1,i), x_sim(2,i) + 2, num2str(i), 'Color', 'r', 'FontSize', 10, 'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'center');
    text(x_sim(6,i), x_sim(7,i) + offset_y_agent2 + 2, num2str(i), 'Color', 'g', 'FontSize', 10, 'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'center');
end
legend({'Agent 1 (Red)', 'Agent 2 (Green)'}, 'Location', 'northeast', 'FontSize', 12);
plotMinDistance(x_sim,N_sim);
