function waypoints = WAY1_newnew(x0) 
% x03=-30;
% y0=x0(2); %% assuming that this referencing is for the first car
y0 = -5;
way =  [110 y0; 110 y0+1; 110 y0 + 2; 110 y0+3; 109 -0.50; 108 -0.20; 107 -0.10; 106 0; 105 0; 104 0; 103 0; 100 0; 95 0; 90 0; 85 0; 80 0];
refPath = referencePathFrenet(way);
% Sample points along the reference path
arcLengths = linspace(0, refPath.PathLength, 100); % 100 points along the path 350
positions = interpolate(refPath, arcLengths);  % Only one output: positions
X = positions(:, 1);
Y = positions(:, 2);

       
    waypoints = [X'; Y'];
%%     Plotting for visualization
if 0
   road_width = 5;
road_length = 60;
merging_lane_width = 5;
merging_lane_position = 100;

    figure(700);
rectangle('Position', [60, -2.5, road_length, road_width], 'FaceColor', [0, 1, 1, 0.5]);

% Merging lane
% rectangle('Position', [merging_lane_position - merging_lane_width/2, -road_width, merging_lane_width, road_width], 'FaceColor', 'r');
rectangle('Position', [107.5 -15 7.5 12.5], 'FaceColor', [0, 1, 0, 0.5]);
    axis equal;
    xlabel('x [m]');
    ylabel('y [m]');
    title('Trajectory');
    grid on;
    hold on;
    plot(X, Y, 'k--', 'LineWidth', 1);

    title('Vehicle Path at Junction');
    xlabel('X Coordinate');
    ylabel('Y Coordinate');
    grid on;
end
end
