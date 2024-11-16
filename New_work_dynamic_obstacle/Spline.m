% Original spline points
x_points = [110, 107.5, 105, 102.5];
y_points = [-10, -8, -7, -5];

% Generate more points for the spline

ocp_N=100;
t = linspace(0, 1, 200); % Parameter for the spline

% Spline fitting
pp_x = spline([0, 1/3, 2/3, 1], x_points); % Parametric x-coordinates
pp_y = spline([0, 1/3, 2/3, 1], y_points); % Parametric y-coordinates

% Evaluate the spline at points t
x_spline = ppval(pp_x, t);
y_spline = ppval(pp_y, t);

% Define the horizontal part
% x_horizontal = linspace(x_spline(end), 102.40, 1); % 10 points for the horizontal part
% y_horizontal = y_spline(end) * ones(1, 1); % y value remains constant

% Concatenate the original spline with the horizontal part
% x_spline = [x_spline, x_horizontal];
% y_spline = [y_spline, y_horizontal];
hold on

% Plot the result to check
plot(x_spline, y_spline, '-o');

xlabel('x [m]');
ylabel('y [m]');
title('Reference Trajectory with Horizontal Part');
grid on;

x_ref = x_spline_full;
y_ref = y_spline_full;

theta_ref = atan2(diff(y_ref), diff(x_ref)); % Approximate heading
theta_ref = [theta_ref, theta_ref(end)]; % Extend the last heading
