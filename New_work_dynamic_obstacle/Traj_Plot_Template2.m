figure(100)
road_width = 20;
road_length = 150;

% Main road
rectangle('Position', [0, -road_width/2, road_length, road_width], 'FaceColor', [0, 1, 1, 0.5]);
hold on;

% Plot trajectories
plot(x_sim(1,:), x_sim(2,:), 'r*-', 'LineWidth', 2); % Agent 1 trajectory
plot(x_sim(6,:), x_sim(7,:), 'bo', 'MarkerSize', 8); % Agent 2 trajectory

% Represent cars as rectangles at more frequent time steps
car_length = 1; % Length of the car representation
car_width = 1;  % Width of the car representation

% Define more frequent time steps (5 times more)
time_steps = round(linspace(1, length(x_sim(1,:)), 8)); % More frequent sampling

for i = time_steps
    % Draw Agent 1
    rectangle('Position', [x_sim(1,i)-car_length/2, x_sim(2,i)-car_width/2, car_length, car_width], ...
              'FaceColor', 'r', 'EdgeColor', 'k', 'LineWidth', 1.5);
    
    % Draw Agent 2
    rectangle('Position', [x_sim(6,i)-car_length/2, x_sim(7,i)-car_width/2, car_length, car_width], ...
              'FaceColor', 'b', 'EdgeColor', 'k', 'LineWidth', 1.5);
    
    % Annotate timestep
    text(x_sim(1,i), x_sim(2,i), num2str(i), 'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'right', 'Color', 'k', 'FontSize', 8);
    text(x_sim(6,i), x_sim(7,i), num2str(i), 'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'left', 'Color', 'k', 'FontSize', 8);
end

% Add arrows to indicate direction
for i = 1:length(x_sim(1,:))-1
    quiver(x_sim(1,i), x_sim(2,i), x_sim(1,i+1)-x_sim(1,i), x_sim(2,i+1)-x_sim(2,i), 0, ...
           'color','k','MaxHeadSize',1,'AutoScale','off','LineWidth',1.5);
end

% Mark start and end points
plot(x_sim(1,1), x_sim(2,1), 'go', 'MarkerSize', 10); % Start point of Agent 1
plot(x_sim(1,end), x_sim(2,end), 'r*', 'MarkerSize', 10); % End point of Agent 1
plot(x_sim(6,1), x_sim(7,1), 'mo', 'MarkerSize', 10); % Start point of Agent 2
plot(x_sim(6,end), x_sim(7,end), 'b*', 'MarkerSize', 10); % End point of Agent 2

% Annotations for key points
text(x_sim(1,1), x_sim(2,1), ' Start A1', 'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'right');
text(x_sim(1,end), x_sim(2,end), ' End A1', 'VerticalAlignment', 'top', 'HorizontalAlignment', 'left');
text(x_sim(6,1), x_sim(7,1), ' Start A2', 'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'right');
text(x_sim(6,end), x_sim(7,end), ' End A2', 'VerticalAlignment', 'top', 'HorizontalAlignment', 'left');

axis equal;
title('Car Trajectories with Direction Indicators and Vehicle Representations');
xlabel('X Position');
ylabel('Y Position');
grid on; % Optional: add a grid for better visual context
