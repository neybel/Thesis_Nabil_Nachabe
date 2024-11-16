 function waypoints = WAY2_new()
    % Define the first straight line (vertical)
    x1 = linspace(90, 90, 150); % First straight line
    y1 = linspace(40, 15, 150);
    
    % Define the arc (quarter circle)
    theta = linspace(pi, 3*pi/2, 150); % Angle for the arc
    radius = 15;
    centerX = 105; % Center of the arc
    centerY = 20 - radius; % Adjust centerY to make the arc concave inward
    x2 = centerX + radius * cos(theta);
    y2 = centerY + radius * sin(theta);
    
    % Define the second straight line (horizontal)
    x3 = linspace(x2(end), x2(end) + 50, 300); % Second straight line
    y3 = y2(end) * ones(1, 300);
    
    % Concatenate all parts
    X = [x1, x2, x3];
    Y = [y1, y2, y3];
    
    waypoints = [X; Y];
    
%     % Plotting for visualization
%     hold on;    
%     plot(X, Y, 'k--', 'LineWidth', 1);
%         hold on 
%     plot(centerX,centerY,'r*')
 end