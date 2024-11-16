function waypoints = W3() 
    % Define straight lines
    x1 = linspace(-30, -15, 150/5); % First straight line
    y1 = linspace(0, 0, 150/5);
    
    % Define the arc (quarter circle)
    theta = linspace(pi, 2*pi, 150/5); % Angle for the arc
    radius = 15;
    centerX = 0; % Center of the arc
    centerY = 0;
    x2 = centerX + radius * cos(theta);
    y2 = centerY + radius * sin(theta);
    
    % Define the second straight line
    x3 = linspace(x2(end),x2(end)+50,300/3); % Second straight line
    y3 = y2(end)*ones(1,300/3);
    
    % Concatenate all parts
    X = [x1, x2,x3];
    Y = [y1, y2,y3];
    
    waypoints = [X; Y];
    
    % Plotting for visualization
% hold on
%     plot(X, Y, 'b--', 'LineWidth', 1);
    
% 
%     title('Vehicle Path at Junction');
%     xlabel('X Coordinate');
%     ylabel('Y Coordinate');
%     grid on;
end
