%% Tinkering with Frenel path reference generation
% Define waypoints as [x, y] coordinates
waypoints = [0 0;
             10 0;
             20 10;
             30 30];

% Create a referencePathFrenet object
refPath = referencePathFrenet(waypoints);

% Sample points along the reference path
arcLengths = linspace(0, refPath.PathLength, 100); % 100 points along the path
positions = interpolate(refPath, arcLengths);  % Only one output: positions

x = positions(:, 1);
y = positions(:, 2);

% Extract tangent vectors
tangentX = positions(:, 3);
tangentY = positions(:, 4);

% Plot the path and tangent vectors
figure;
hold on;
plot(waypoints(:,1), waypoints(:,2), 'ro', 'MarkerSize', 10, 'DisplayName', 'Waypoints');
plot(x, y, 'b-', 'LineWidth', 2, 'DisplayName', 'Reference Path');
quiver(x, y, tangentX, tangentY, 0.5, 'g', 'DisplayName', 'Tangent Vectors');
title('Reference Path with Tangent Vectors');
xlabel('X');
ylabel('Y');
legend('show');
grid on;
hold off;
%%
% connector = trajectoryGeneratorFrenet(refPath);
