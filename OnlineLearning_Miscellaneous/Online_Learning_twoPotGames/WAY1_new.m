function waypoints = WAY1_new() 
    % Define straight lines
%     clc
%     clear 
%     close all
    x1 = linspace(110, 110, 10); % First straight line
    y1 = linspace(-5, -2.5, 10);
    
    % Define the arc (quarter circle)
    theta = linspace(0, pi/2, 10); % Angle for the arc
    radius = 2.5;
    centerX = 107.5; % Center of the arc
    centerY = -5 + radius;
    x2 = centerX + radius * cos(theta);
    y2 = centerY + radius * sin(theta);
    
    % Define the second straight line
    x3 = linspace(x2(end),x2(end)-60,130); % Second straight line it was 30 inst3ead of 130 before! and -20 for groundtruth1
    y3 = y2(end)*ones(1,130);
    
    % Concatenate all parts
    X = [x1, x2,x3];
    Y = [y1, y2,y3];
    
    waypoints = [X; Y];
    
% %     Plotting for visualization
% hold on
%     plot(X, Y, 'k--', 'LineWidth', 1);
% 
%     title('Vehicle Path at Junction');
%     xlabel('X Coordinate');
%     ylabel('Y Coordinate');
%     grid on;
end
