function [scenario, egoVehicle] = createDrivingScenario()
% createDrivingScenario Returns the drivingScenario defined in the Designer

% Construct a drivingScenario object.
scenario = drivingScenario;

% Roundabout parameters
outerRadius = 20; % Outer radius of the roundabout (smaller)
innerRadius = 10; % Inner radius of the roundabout (central island)
laneWidth = 15; % Increased lane width
entryExitLength = 60; % Length of entry/exit lanes

% Define the roundabout with lane boundaries
numLanes = 2;
theta = linspace(0, 2*pi, 360);

% Define road segments for roundabout
numSegments = 36; % Number of segments to approximate the roundabout

% Add outer and inner roundabout roads
for i = 1:numSegments
    angle1 = (i-1) * 2*pi / numSegments;
    angle2 = i * 2*pi / numSegments;

    % Flip x and y to match roundabout orientation
    roadCentersOuter = [outerRadius * sin(angle1), outerRadius * cos(angle1); ...
                        outerRadius * sin(angle2), outerRadius * cos(angle2)];
    roadCentersInner = [innerRadius * sin(angle1), innerRadius * cos(angle1); ...
                        innerRadius * sin(angle2), innerRadius * cos(angle2)];

    road(scenario, roadCentersOuter, 'Name', 'OuterRoad');
    road(scenario, roadCentersInner, 'Name', 'InnerRoad');
end

% Define entry/exit points (approximated as straight roads)
entryExitAngles = linspace(0, 2*pi, 5);
entryExitAngles(end) = [];

for i = 1:length(entryExitAngles)
    angle = entryExitAngles(i);
    entryPoint = [outerRadius * sin(angle), outerRadius * cos(angle)];
    exitPoint = [outerRadius + entryExitLength * sin(angle), outerRadius + entryExitLength * cos(angle)];
    
    % Add entry/exit lanes
    road(scenario, [entryPoint; exitPoint], 'Name', sprintf('EntryExitLane%d', i));
end

% Add the ego vehicle
egoVehicle = vehicle(scenario, ...
    'ClassID', 1, ...
    'Position', [6.8 4.8 0], ...
    'Mesh', driving.scenario.carMesh, ...
    'Name', 'Car');

% Define waypoints and speed
waypoints = [6.8 4.8 0;
    16.9 5 0;
    23.9 4.7 0;
    32.5 5 0;
    42.2 4.5 0;
    50 4.4 0];
speed = [30;30;30;30;30;30];
trajectory(egoVehicle, waypoints, speed);

end
