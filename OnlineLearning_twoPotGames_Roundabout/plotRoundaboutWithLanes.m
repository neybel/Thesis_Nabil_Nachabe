function plotRoundaboutWithLanes()

    % Roundabout parameters
    outerRadius = 20; % Outer radius of the roundabout (smaller)
    innerRadius = 7.50; % Inner radius of the roundabout (central island)
    numLanes = 2;     % Number of lanes in the roundabout

    % Angles for plotting
    theta = linspace(0, 2*pi, 360);

    % Outer circle (roundabout)
    outerX = outerRadius * cos(theta);
    outerY = outerRadius * sin(theta);

    % Inner circle (central island)
    innerX = innerRadius * cos(theta);
    innerY = innerRadius * sin(theta);

    % Lane boundaries
    laneWidth = 15; % Increased lane width
    laneBoundariesX = zeros(numLanes-1, length(theta));
    laneBoundariesY = zeros(numLanes-1, length(theta));
    
    for i = 1:numLanes-1
        radius = innerRadius + i * laneWidth;
        laneBoundariesX(i, :) = radius * cos(theta);
        laneBoundariesY(i, :) = radius * sin(theta);
    end

    % Plot roundabout
    figure;
    hold on;
    fill(outerX, outerY, [0.7 0.7 0.7], 'EdgeColor', 'k'); % Outer circle
    fill(innerX, innerY, [0.3 0.3 0.3], 'EdgeColor', 'k'); % Inner circle (central island)

%     % Plot lane boundaries
%     for i = 1:numLanes-1
%         plot(laneBoundariesX(i, :), laneBoundariesY(i, :), 'k--');
%     end

    % Define entry and exit points
    entryExitLength = 60; % Length of entry/exit lanes
    numEntryExit = 4; % Number of entry/exit points

    % Entry/exit angles (0, pi/2, pi, 3*pi/2)
    entryExitAngles = linspace(0, 2*pi, numEntryExit+1);
    entryExitAngles(end) = [];

    % Plot entry/exit lanes
    for i = 1:numEntryExit
        angle = entryExitAngles(i);

        % Entry lanes (two lanes)
        for j = 0:1
            offset = j * laneWidth - laneWidth/2;
            ex = [outerRadius * cos(angle) + offset * cos(angle + pi/2), ...
                  (outerRadius + entryExitLength) * cos(angle) + offset * cos(angle + pi/2)];
            ey = [outerRadius * sin(angle) + offset * sin(angle + pi/2), ...
                  (outerRadius + entryExitLength) * sin(angle) + offset * sin(angle + pi/2)];
            plot(ex, ey, 'k-', 'LineWidth', 2);
        end

        % Exit lanes (two lanes)
        for j = 0:1
            offset = j * laneWidth - laneWidth/2;
            ex = [(outerRadius - laneWidth) * cos(angle) + offset * cos(angle + pi/2), ...
                  (outerRadius - laneWidth - entryExitLength) * cos(angle) + offset * cos(angle + pi/2)];
            ey = [(outerRadius - laneWidth) * sin(angle) + offset * sin(angle + pi/2), ...
                  (outerRadius - laneWidth - entryExitLength) * sin(angle) + offset * sin(angle + pi/2)];
            plot(ex, ey, 'k-', 'LineWidth', 2);
        end
    end

    % Set plot limits and aspect ratio
    axis equal;
    xlim([-outerRadius-entryExitLength, outerRadius+entryExitLength]);
    ylim([-outerRadius-entryExitLength, outerRadius+entryExitLength]);

    % Labels and title
    xlabel('X Coordinate (meters)');
    ylabel('Y Coordinate (meters)');
    title('Roundabout with Entry/Exit Lanes');
    grid on;
    hold off;

%     W1(); W2(); W3(); W4();
% m2=W2();m3=W3();
% hold on 
% plot(m1(1,:),m1(2,:))
% hold on 
% % plot(m2(1,:),m2(2,:))
% hold on 
% plot(m3(1,:),m3(2,:)) 
% hold on 
% plot(m4(1,:),m4(2,:))
%     legend('1','2','3','4')
end
