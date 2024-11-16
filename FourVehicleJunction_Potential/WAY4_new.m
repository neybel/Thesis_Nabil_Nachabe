function waypoints = WAY4_new()
    % Define the first straight line (horizontal, moving leftwards)
    x1 = linspace(70, 105, 150); % First straight line
    y1 = linspace(10, 10, 150); % Horizontal line
    
    % Define the arc (quarter circle, concave outwards)
    theta = linspace(3*(pi/2), 2*pi, 150); % Angle for the arc
    radius = 15;
    centerX = 90+radius; % Center of the arc
    centerY = 25; % Adjust centerY to make the arc concave outward
    x2 = centerX + radius * cos(theta);
    y2 = centerY + radius * sin(theta);
    
    % Define the second straight line (vertical, moving upwards)
    x3 = linspace(x2(end), x2(end), 300); % Second straight line
    y3 = linspace(y2(end), y2(end) + 50, 300);
    
    % Concatenate all parts
    X = [x1, x2, x3];
    Y = [y1, y2, y3];
    
    waypoints = [X; Y];
    
    % Plotting for visualization
%     hold on;
%     plot(X, Y, 'b--', 'LineWidth', 1);
%     plot(centerX, centerY, 'r*');
%     title('Vehicle Path with Smooth Arc and Vertical Line');
%     xlabel('X Coordinate');
%     ylabel('Y Coordinate');
%     grid on;
end
