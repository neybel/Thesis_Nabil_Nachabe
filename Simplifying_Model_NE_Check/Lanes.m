% Define road parameters
clc
close all
clear
road_width = 20;
road_length = 200;
merging_lane_width = 10;
merging_lane_position = 100;

% Plot road
figure;
hold on;

% Main road
rectangle('Position', [0, -road_width/2, road_length, road_width], 'FaceColor', 'g');

% Merging lane
% rectangle('Position', [merging_lane_position - merging_lane_width/2, -road_width, merging_lane_width, road_width], 'FaceColor', 'r');
rectangle('Position', [95 -60 20 50], 'FaceColor', 'r');

% Set axis properties
axis equal;
xlabel('x [m]');
ylabel('y [m]');
title('Road');
grid on;

hold off;
