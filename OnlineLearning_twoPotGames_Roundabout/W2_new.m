function waypoints = W2_new(x0) 
%   y02=-30;
  y02=x0(2);
way=[0 y02;0 -25 ;0 -20;0 -15;13 -7 ;15 0;7 13 ;0 15;0 20 ;0 30 ;0 60];
refPath = referencePathFrenet(way);
% Sample points along the reference path
arcLengths = linspace(0, refPath.PathLength, 175); % 100 points along the path
positions = interpolate(refPath, arcLengths);  % Only one output: positions
X = positions(:, 1);
Y = positions(:, 2);

    
    waypoints = [X'; Y'];
    
    % Plotting for visualization
    if 0
hold on
    plot(X, Y, 'r--', 'LineWidth', 1);
%     
% 
%     title('Vehicle Path at Junction');
%     xlabel('X Coordinate');
%     ylabel('Y Coordinate');
%     grid on;
    end
end
