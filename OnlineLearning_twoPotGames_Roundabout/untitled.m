% Create a driving scenario
scenario = drivingScenario;

% Add a vehicle to the scenario
vehicle = vehicle(scenario, 'ClassID', 1);
vehicle.Position = [0 0 0];
vehicle.Yaw = 0;

% Define the road
roadCenters = [0 0; 100 0];
road(scenario, roadCenters, 'Lanes', lanespec(1));

% Set up simulation parameters
simTime = 10; % Simulation time in seconds
sampleTime = 1; % Sample time in seconds
% Create a constant speed controller
% Initialize control inputs
throttle = 50; % Throttle value (between 0 and 1)
steering = 0; % Steering angle in radians

% Simulation loop
t = 0;
while t < simTime
    % Apply control inputs
    vehicle.Velocity = [throttle; 0; 0]; % Simplified velocity control
    vehicle.Yaw = steering; % Simplified steering control
    
    % Advance the simulation
    advance(scenario);
    
    % Increment time
    t = t + sampleTime;
end
% % Plot the scenario
% plot(scenario);