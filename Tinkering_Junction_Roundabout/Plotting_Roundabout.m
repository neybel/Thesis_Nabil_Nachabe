clc
close all
figure(100)
road_width = 40;
road_length = 200;
merging_lane_width = 40;
merging_lane_position = 100;

plotRoundaboutWithLanes();
hold on
 % Plot merging lane dashed line (straight)
x_merge_lane = 0:10:200; % Adjust based on the merging lane position and length
y_merge_lane = 0 * ones(size(x_merge_lane)); % Straight Y coordinates for merging dashed line
plot(x_merge_lane, y_merge_lane, 'w--', 'LineWidth', 2,'HandleVisibility', 'off'); % White dashed line
hold on 

y_merge_lane2 = -80:10:60; % Adjust based on the merging lane position and length
x_merge_lane2 = 105 * ones(size(y_merge_lane2)); % Straight Y coordinates for merging dashed line
plot(x_merge_lane2, y_merge_lane2, 'w--', 'LineWidth', 2,'HandleVisibility', 'off'); % White dashed line


% Plot trajectories for four agents
h1=plot(x_sim(1,:), x_sim(2,:), 'r', 'LineWidth', 2);  % Agent 1
h2=plot(x_sim(6,:), x_sim(7,:), 'g', 'LineWidth', 2);  % Agent 2
h3=plot(x_sim(11,:), x_sim(12,:), 'b', 'LineWidth', 2); % Agent 3
h4=plot(x_sim(16,:), x_sim(17,:), 'm', 'LineWidth', 2); % Agent 4

% Car dimensions
car_length = 2; % Length of the car representation
car_width = 2;  % Width of the car representation

% Define more frequent time steps (5 times more)
time_steps = round(linspace(1, length(x_sim(1,:)), 5)); % More frequent sampling

for i = time_steps
    % Calculate rectangle corners based on orientation
    theta = x_sim(3,i); % Orientation angle
    % Define the rectangle corners based on fixed car length and width
    rect_x = [-car_length/2, car_length/2, car_length/2, -car_length/2]; 
    rect_y = [-car_width/2, -car_width/2, car_width/2, car_width/2];

    % Rotation matrix for the angle theta
    rotation_matrix = [cos(theta), -sin(theta); sin(theta), cos(theta)];

    % Rotate rectangle corners
    rotated_corners = rotation_matrix * [rect_x; rect_y];
    % Translate to the current position
    translated_corners = rotated_corners + [x_sim(1,i); x_sim(2,i)];

    % Fill the rectangle (car) at the new position and orientation
    fill(translated_corners(1,:), translated_corners(2,:), 'r', 'EdgeColor', 'k', 'LineWidth', 1.5,'HandleVisibility', 'off');
    quiver(x_sim(1,i), x_sim(2,i), cos(theta) * car_length, sin(theta) * car_length, ...
           'Color', 'k', 'LineWidth', 1, 'MaxHeadSize', 1,'HandleVisibility', 'off');

    % Repeat for Agent 2
    theta = x_sim(8,i);
    rotated_corners = rotation_matrix * [rect_x; rect_y];
    translated_corners = rotated_corners + [x_sim(6,i); x_sim(7,i)];
    fill(translated_corners(1,:), translated_corners(2,:), 'g', 'EdgeColor', 'k', 'LineWidth', 1.5,'HandleVisibility', 'off');
    quiver(x_sim(6,i), x_sim(7,i), cos(theta) * car_length, sin(theta) * car_length, ...
           'Color', 'k', 'LineWidth', 1, 'MaxHeadSize', 1,'HandleVisibility', 'off');
    
    % Repeat for Agent 3
    theta = x_sim(13,i);
    rotated_corners = rotation_matrix * [rect_x; rect_y];
    translated_corners = rotated_corners + [x_sim(11,i); x_sim(12,i)];
    fill(translated_corners(1,:), translated_corners(2,:), 'b', 'EdgeColor', 'k', 'LineWidth', 1.5,'HandleVisibility', 'off');
    quiver(x_sim(11,i), x_sim(12,i), cos(theta) * car_length, sin(theta) * car_length, ...
           'Color', 'k', 'LineWidth', 1, 'MaxHeadSize', 1,'HandleVisibility', 'off');

    % Repeat for Agent 4
    theta = x_sim(18,i);
    rotated_corners = rotation_matrix * [rect_x; rect_y];
    translated_corners = rotated_corners + [x_sim(16,i); x_sim(17,i)];
    fill(translated_corners(1,:), translated_corners(2,:), 'm', 'EdgeColor', 'k', 'LineWidth', 1.5,'HandleVisibility', 'off');
    quiver(x_sim(16,i), x_sim(17,i), cos(theta) * car_length, sin(theta) * car_length, ...
           'Color', 'k', 'LineWidth', 1, 'MaxHeadSize', 1,'HandleVisibility', 'off');

%     % Annotate timestep numbers at each car position
%     text(x_sim(1,i), x_sim(2,i), num2str(i), 'Color', 'r', 'FontSize', 10, 'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'center');
%     text(x_sim(6,i), x_sim(7,i), num2str(i), 'Color', 'g', 'FontSize', 10, 'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'center');
%     text(x_sim(11,i), x_sim(12,i), num2str(i), 'Color', 'b', 'FontSize', 10, 'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'center');
%     text(x_sim(16,i), x_sim(17,i), num2str(i), 'Color', 'm', 'FontSize', 10, 'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'center');
% Annotate timestep numbers at each car position with fixed font size
text(x_sim(1,i), x_sim(2,i) -5 , num2str(i), 'Color', 'r', 'FontSize', 10, 'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'center');
text(x_sim(6,i)-2.5, x_sim(7,i) + 1.0 , num2str(i), 'Color', 'g', 'FontSize', 10, 'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'center');
text(x_sim(11,i)+2, x_sim(12,i) + 1, num2str(i), 'Color', 'b', 'FontSize', 10, 'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'center');
text(x_sim(16,i) -2.50, x_sim(17,i) - 5, num2str(i), 'Color', 'm', 'FontSize', 10, 'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'center');
end

% Set axes properties
% axis equal;
% xlim([min(x_sim(1,:)) - 10, max(x_sim(1,:)) + 10]);
% ylim([min(x_sim(2,:)) - 10, max(x_sim(2,:)) + 10]);
xlabel('x [m]');
ylabel('y [m]');
title('Car Trajectories with Orientation');
grid on;
grid minor;
hold off
% legend({'Agent 1 (red)','Agent 2 (Green)','Agent 3 (Blue)','Agent 4 (Pink)'})
% Add a text box to describe the agents
% annotation('textbox', [0.15, 0.85, 0.2, 0.1], 'String', {'Agent 1 (red)', 'Agent 2 (green)', 'Agent 3 (blue)', 'Agent 4 (pink)'}, ...
%            'EdgeColor', 'none', 'FontSize', 12, 'Color', 'k');
legend([h1, h2,h3,h4], {'Agent 1', 'Agent 2','Agent 3','Agent 4'},'Location','best');

return
plotMinDistance(x_sim,N_sim);
