   
%% Second car
function waypoints = WAY2() 

    % Define spline control points for an outward leftward merge
    x_points = [90, 95, 100, 105];  % X coordinates of the control points
    y_points = [40, 30, 15, 5];   % Y coordinates of the control points
    
    % Generate more points for the spline
    t = linspace(0, 1, 200); % Parameter for the spline
    
    % Spline fitting
    pp_x = spline([0, 1/3, 2/3, 1], x_points); % Parametric x-coordinates
    pp_y = spline([0, 1/3, 2/3, 1], y_points); % Parametric y-coordinates
    
    % Evaluate the spline at points t
    x_spline = ppval(pp_x, t);
    y_spline = ppval(pp_y, t);
    
    % Define the horizontal part after the curve
    x_horizontal = linspace(x_spline(end), x_spline(end) + 50, 200); % Decrease X values to the left
    y_horizontal = y_spline(end) * ones(1, 200); % Y value remains constant
    
    % Concatenate all parts
    X = [x_spline, x_horizontal];
    Y = [y_spline, y_horizontal];
    
    waypoints = [X; Y];
%     hold on 
%     plot(X, Y, 'k--', 'LineWidth', 1);
%     return
%     
%     % Plotting for visualization
%     figure;
%     plot(x_spline, y_spline, 'r-', 'LineWidth', 2); hold on;
%     plot(x_horizontal, y_horizontal, 'g-', 'LineWidth', 2);
%     plot(X, Y, 'k--', 'LineWidth', 1);
%     legend('Spline Curve', 'Final Straight', 'Complete Path');
%     title('Vehicle Path at Junction');
%     xlabel('X Coordinate');
%     ylabel('Y Coordinate');
%     grid on;
end

