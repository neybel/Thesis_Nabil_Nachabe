clc
clearvars
close all
% Create and set up the scenario
[scenario, egoVehicle] = createDrivingScenario();

% Create a figure for visualization
fig = figure;
ax = axes(fig); % Create axes within the figure

% Run the simulation
while advance(scenario)
    % Clear the current axes but keep the figure open
    cla(ax);
    
    % Plot the scenario and update the visualization
    plot(scenario, 'Parent', ax);
    drawnow; % Refresh the figure
end
