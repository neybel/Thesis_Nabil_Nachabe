function waypoints = WAY1_new() 
    % Define straight lines
    x1 = linspace(115, 115, 150); % First straight line
    y1 = linspace(-40, -15, 150);
    
    % Define the arc (quarter circle)
    theta = linspace(0, pi/2, 150); % Angle for the arc
    radius = 15;
    centerX = 100; % Center of the arc
    centerY = -20 + radius;
    x2 = centerX + radius * cos(theta);
    y2 = centerY + radius * sin(theta);
    
    % Define the second straight line
    x3 = linspace(x2(end),x2(end)-30,300); % Second straight line
    y3 = y2(end)*ones(1,300);
    
    % Concatenate all parts
    X = [x1, x2,x3];
    Y = [y1, y2,y3];
    
    waypoints = [X; Y];
    
%     % Plotting for visualization
% hold on
%     plot(X, Y, 'k--', 'LineWidth', 1);
% 
%     title('Vehicle Path at Junction');
%     xlabel('X Coordinate');
%     ylabel('Y Coordinate');
%     grid on;
end
