function waypoints = W3_new(x0) 
% x03=-30;
x03=x0(6);
   way=[x03 0;-25 0;-20 0; -16 0; -13 -7; -7 -13; 0 -15; 7 -13;13 -7; 16 0;20 0; 22 0; 25 0;27 0;30 0;40 0;50 0;60 0];
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
    plot(X, Y, 'b--', 'LineWidth', 1);
    end
    
% 
%     title('Vehicle Path at Junction');
%     xlabel('X Coordinate');
%     ylabel('Y Coordinate');
%     grid on;
end
