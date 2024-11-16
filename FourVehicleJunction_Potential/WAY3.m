   
%% Third car

function waypoints = WAY3() 

    % Define spline control points for an outward leftward merge
    x_points = [125, 115, 110, 107.5];  % X coordinates of the control points
    y_points = [-5, 10, 20, 30];   % Y coordinates of the control points
    
    % Generate more points for the spline
    t = linspace(0, 1, 200); % Parameter for the spline
    
    % Spline fitting
    pp_x = spline([0, 1/3, 2/3, 1], x_points); % Parametric x-coordinates
    pp_y = spline([0, 1/3, 2/3, 1], y_points); % Parametric y-coordinates
    
    % Evaluate the spline at points t
    x_spline = ppval(pp_x, t);
    y_spline = ppval(pp_y, t);
    
    % Define the horizontal part after the curve
    x_horizontal = linspace(x_spline(1) + 25, x_spline(1), 200); % Decrease X values to the left
    y_horizontal = y_spline(1) * ones(1, 200); % Y value remains constant

    x_vertical = x_spline(end)*ones(1,200);
    y_vertical = linspace(y_spline(end),y_spline(end) + 20,200);
    
    % Concatenate all parts
    X = [x_horizontal,x_spline,x_vertical];
    Y = [y_horizontal,y_spline,y_vertical];

    waypoints = [X; Y];
%     hold on 
%     plot(X, Y, 'b--', 'LineWidth', 1);
%     return
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

