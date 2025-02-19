clc; clear; close all;

%% Parameters
dt = 0.1;          % Time step (s)
L = 0.1;           % Lookahead distance (m)
v = 5.0;           % Constant velocity (m/s)
x = 0; y = 0;      % Initial position
theta = 0;         % Initial heading (rad)
idx = 1;           % Start tracking from the first waypoint

%% Define Original Path (Waypoints)
path_original = [0,0; 10,5; 20,10; 30,15; 40,20]; % Original waypoints

%% Interpolate Path to 0.1m Spacing
path_distance = cumsum([0; sqrt(sum(diff(path_original).^2, 2))]); % Cumulative distances
interp_distances = 0:0.1:path_distance(end); % Generate new points at 0.1m intervals

path_interp(:,1) = interp1(path_distance, path_original(:,1), interp_distances, 'linear'); % Interpolated X
path_interp(:,2) = interp1(path_distance, path_original(:,2), interp_distances, 'linear'); % Interpolated Y

num_waypoints = size(path_interp, 1);
path = path_interp; % Use interpolated path

%% Simulation Loop
figure; hold on;
plot(path(:,1), path(:,2), 'k--', 'LineWidth', 2); % Plot path
xlabel('X Position (m)'); ylabel('Y Position (m)');
title('Pure Pursuit Path Tracking (0.1m Resolution)');
grid on;

for i = 1:1000  % Increased iteration count for finer tracking
    % Find Lookahead Point (only move forward)
    while idx < num_waypoints && norm([x, y] - path(idx, :)) < L
        idx = idx + 1; % Move to the next waypoint if within lookahead distance
    end
    if idx >= num_waypoints
        break;  % Stop if reached the final waypoint
    end
    lookahead_point = path(idx, :);
    
    % Compute Steering Angle
    dx = lookahead_point(1) - x;
    dy = lookahead_point(2) - y;
    alpha = atan2(dy, dx) - theta;   % Angle to lookahead point
    kappa = (2 * sin(alpha)) / L;    % Curvature
    omega = v * kappa;               % Angular velocity
    
    % Update State (Point Mass Model)
    theta = theta + omega * dt;  % Update heading
    x = x + v * cos(theta) * dt; % Update x position
    y = y + v * sin(theta) * dt; % Update y position
    
    % Plot Vehicle
    plot(x, y, 'ro', 'MarkerFaceColor', 'r');
    pause(0.01);  % Faster visualization
end

disp('Simulation Completed.');
