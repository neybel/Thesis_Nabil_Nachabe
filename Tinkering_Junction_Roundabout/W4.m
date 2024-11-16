function waypoints = W4() 
    % Define straight lines
    x1 = linspace(0, 0, 225); % First straight line
    y1 = linspace(60, 15, 225);
    
    % Define the arc (quarter circle)
    theta = linspace(pi/2, 3*pi/2, 150); % Angle for the arc
    radius = 15;
    centerX = 0; % Center of the arc
    centerY = 0;
    x2 = centerX + radius * cos(theta);
    y2 = centerY + radius * sin(theta);
    
    % Define the second straight line
    y3 = linspace(y2(end),y2(end)-30,300); % Second straight line
    x3 = x2(end)*ones(1,300);
    
    % Concatenate all parts
    X = [x1, x2,x3];
    Y = [y1, y2,y3];
    
    waypoints = [X; Y];
    
    % Plotting for visualization
% hold on
%     plot(X, Y, 'c--', 'LineWidth', 1);


% 
%     title('Vehicle Path at Junction');
%     xlabel('X Coordinate');
%     ylabel('Y Coordinate');
%     grid on;
end
