%% WAYPOINT First car   
function waypoints = WAY1() 
    % Define spline control points
%     x_points = [110, 105, 102.5, 102];
%     y_points = [-40, -15, -11, -10];
 x_points = [110, 105, 100, 95];    
    y_points = [-40, -20, -10, -5];
    
    % Generate more points for the spline
    t = linspace(0, 1, 200); % Parameter for the spline
    
    % Spline fitting
    pp_x = spline([0, 1/3, 2/3, 1], x_points); % Parametric x-coordinates
    pp_y = spline([0, 1/3, 2/3, 1], y_points); % Parametric y-coordinates
    
    % Evaluate the spline at points t
    x_spline = ppval(pp_x, t);
    y_spline = ppval(pp_y, t);
    
    % Define the horizontal part after the curve
    x3 = linspace(x_spline(end), 50, 450);
    y3 = y_spline(end) * ones(1, 450);
    
    % Concatenate all parts
    X = [x_spline, x3];
    Y = [y_spline, y3];
    
    waypoints = [X; Y];
%     hold on 
%     plot(X, Y, 'r--', 'LineWidth', 1);


%     return
    
    % Plotting for visualization
%     figure;
%     plot(x_spline, y_spline, 'r-', 'LineWidth', 2); hold on
%     plot(x3, y3, 'g-', 'LineWidth', 2);
%     plot(X, Y, 'k--', 'LineWidth', 1);
%     legend('Spline Curve', 'Final Straight', 'Complete Path');
%     title('Vehicle Path at Junction');
%     xlabel('X Coordinate');
%     ylabel('Y Coordinate');
%     grid on;
end