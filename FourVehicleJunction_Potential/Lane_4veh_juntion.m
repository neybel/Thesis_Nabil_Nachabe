% Define road parameters
clc
close all
clear
road_width = 40;
road_length = 200;
merging_lane_width = 40;
merging_lane_position = 100;

% Plot road
figure;
hold on;

% Main road
rectangle('Position', [0, -road_width/2, road_length, road_width], 'FaceColor', 'c');

% Merging lane
% rectangle('Position', [merging_lane_position - merging_lane_width/2, -road_width, merging_lane_width, road_width], 'FaceColor', 'r');
rectangle('Position', [85 -80 40 140], 'FaceColor', 'g');

% Set axis properties
axis equal;
xlabel('x [m]');
ylabel('y [m]');
title('Road');
grid on;

hold on
if 1
m1=WAY1_new();
m2=WAY2_new();
m3=WAY3_new();
m4=WAY4_new ();
plot(m1(1,:), m1(2,:), 'b--', 'LineWidth', 1); hold on
plot(m2(1,:), m2(2,:), 'r--', 'LineWidth', 1); hold on
plot(m3(1,:), m3(2,:), 'k--', 'LineWidth', 1); hold on
plot(m4(1,:), m4(2,:), 'k--', 'LineWidth', 1); hold on
end
