%% visualize_potential_field.m
% Generates publication-quality potential field visualizations for the
% multi-robot firefighting system. Shows:
%   1. Fire attractive potential (from temperature gradient)
%   2. Robot repulsive potential (from 6 robots)
%   3. Fire safety repulsive potential
%   4. Combined total potential field
%
% This script creates visualizations suitable for technical reports.

clear; close all; clc;

%% ============================
%   1. Setup environment
% =============================
env = env_setup();

%% ============================
%   2. Define scenario: 1 fire + 6 robots
% =============================
% Use a single fire for clarity
env.num_fires = 1;
env.fire_positions = [0; 0];  % Fire at origin
env.fire_intensities = 1.0;
env.fire_spreads = 2.0;

% Place 6 robots in a semi-circle formation around the fire
% This creates an interesting asymmetric potential field
robot_positions = [
    -4,  -2,   2,   4,   3,  -3;   % x coordinates
    -2,  -4,  -4,  -2,   2,   2    % y coordinates
];

N = 6;

%% ============================
%   3. Create spatial grid for potential field
% =============================
x_min = -env.world_size(1)/2;
x_max =  env.world_size(1)/2;
y_min = -env.world_size(2)/2;
y_max =  env.world_size(2)/2;

% High-resolution grid for smooth visualization
grid_res = 200;
x_lin = linspace(x_min, x_max, grid_res);
y_lin = linspace(y_min, y_max, grid_res);
[X, Y] = meshgrid(x_lin, y_lin);

%% ============================
%   4. Compute potential fields
% =============================
fprintf('Computing potential fields...\n');

% Initialize potential grids
U_fire_attract = zeros(size(X));    % Attractive potential from fire (negative)
U_robot_repel = zeros(size(X));     % Repulsive potential from robots
U_fire_repel = zeros(size(X));      % Repulsive potential from fire safety zone
U_total = zeros(size(X));           % Total combined potential

% Loop through each grid point
for ix = 1:grid_res
    for iy = 1:grid_res
        pos = [X(iy, ix); Y(iy, ix)];
        
        % --- Fire Attractive Potential (negative of temperature) ---
        % We use negative temperature as attractive potential
        [T, ~] = temperature_field(pos, env);
        U_fire_attract(iy, ix) = -env.k_T * T;
        
        % --- Robot-Robot Repulsive Potential ---
        U_rep = 0;
        for i = 1:N
            robot_pos = robot_positions(:, i);
            dx = pos - robot_pos;
            dist = norm(dx);
            
            % Repulsive potential: U = k_rep * (1/d - 1/R_s)^2 for d < R_s
            if dist < env.R_s && dist > 1e-6
                U_rep = U_rep + 0.5 * env.k_rep * (1/dist - 1/env.R_s)^2;
            end
        end
        U_robot_repel(iy, ix) = U_rep;
        
        % --- Fire Safety Repulsive Potential ---
        U_fire_rep = 0;
        for k = 1:env.num_fires
            fire_pos = env.fire_positions(:, k);
            dx = pos - fire_pos;
            dist = norm(dx);
            
            % Repulsive potential from fire safety zone
            if dist < env.R_safe_fire && dist > 1e-6
                U_fire_rep = U_fire_rep + 0.5 * env.k_fire_rep * (1/dist - 1/env.R_safe_fire)^2;
            end
        end
        U_fire_repel(iy, ix) = U_fire_rep;
        
        % --- Total Potential ---
        U_total(iy, ix) = U_fire_attract(iy, ix) + U_robot_repel(iy, ix) + U_fire_repel(iy, ix);
    end
end

fprintf('Potential field computation complete!\n');

%% ============================
%   5. Create visualizations
% =============================

% Create a large figure with 4 subplots
fig = figure('Name', 'Potential Field Analysis', 'Position', [100, 100, 1400, 1000]);
set(fig, 'Color', 'w');

% Common plotting parameters
theta = linspace(0, 2*pi, 100);

%% Subplot 1: Fire Attractive Potential
subplot(2, 2, 1);
surf(X, Y, U_fire_attract, 'EdgeColor', 'none');
hold on;
% Mark fire location
plot3(env.fire_positions(1), env.fire_positions(2), min(U_fire_attract(:)), ...
      'rp', 'MarkerSize', 20, 'MarkerFaceColor', 'y', 'LineWidth', 2);
view(3);
colormap(gca, 'parula');
colorbar;
xlabel('x [m]', 'FontSize', 11);
ylabel('y [m]', 'FontSize', 11);
zlabel('Potential', 'FontSize', 11);
title('Fire Attractive Potential', 'FontSize', 13, 'FontWeight', 'bold');
grid on;
axis equal tight;
lighting gouraud;
camlight('headlight');

%% Subplot 2: Robot Repulsive Potential
subplot(2, 2, 2);
surf(X, Y, U_robot_repel, 'EdgeColor', 'none');
hold on;
% Mark robot locations
for i = 1:N
    plot3(robot_positions(1, i), robot_positions(2, i), 0, ...
          'bo', 'MarkerSize', 12, 'MarkerFaceColor', 'c', 'LineWidth', 2);
    % Draw repulsion radius circles at z=0
    cx = robot_positions(1, i) + env.R_s * cos(theta);
    cy = robot_positions(2, i) + env.R_s * sin(theta);
    plot3(cx, cy, zeros(size(cx)), 'b--', 'LineWidth', 1.5);
end
view(3);
colormap(gca, 'hot');
colorbar;
xlabel('x [m]', 'FontSize', 11);
ylabel('y [m]', 'FontSize', 11);
zlabel('Potential', 'FontSize', 11);
title('Robot Repulsive Potential', 'FontSize', 13, 'FontWeight', 'bold');
grid on;
axis equal tight;
lighting gouraud;
camlight('headlight');

%% Subplot 3: Fire Safety Repulsive Potential
subplot(2, 2, 3);
surf(X, Y, U_fire_repel, 'EdgeColor', 'none');
hold on;
% Mark fire location and safety radius
plot3(env.fire_positions(1), env.fire_positions(2), 0, ...
      'rp', 'MarkerSize', 20, 'MarkerFaceColor', 'y', 'LineWidth', 2);
cx_safe = env.fire_positions(1) + env.R_safe_fire * cos(theta);
cy_safe = env.fire_positions(2) + env.R_safe_fire * sin(theta);
plot3(cx_safe, cy_safe, zeros(size(cx_safe)), 'r--', 'LineWidth', 2);
view(3);
colormap(gca, 'hot');
colorbar;
xlabel('x [m]', 'FontSize', 11);
ylabel('y [m]', 'FontSize', 11);
zlabel('Potential', 'FontSize', 11);
title('Fire Safety Repulsive Potential', 'FontSize', 13, 'FontWeight', 'bold');
grid on;
axis equal tight;
lighting gouraud;
camlight('headlight');

%% Subplot 4: Combined Total Potential
subplot(2, 2, 4);
surf(X, Y, U_total, 'EdgeColor', 'none');
hold on;
% Mark fire location
plot3(env.fire_positions(1), env.fire_positions(2), min(U_total(:)), ...
      'rp', 'MarkerSize', 20, 'MarkerFaceColor', 'y', 'LineWidth', 2);
% Mark robot locations
for i = 1:N
    plot3(robot_positions(1, i), robot_positions(2, i), max(U_total(:)), ...
          'bo', 'MarkerSize', 12, 'MarkerFaceColor', 'c', 'LineWidth', 2);
end
view(3);
colormap(gca, 'jet');
colorbar;
xlabel('x [m]', 'FontSize', 11);
ylabel('y [m]', 'FontSize', 11);
zlabel('Potential', 'FontSize', 11);
title('Combined Total Potential Field', 'FontSize', 13, 'FontWeight', 'bold');
grid on;
axis equal tight;
lighting gouraud;
camlight('headlight');

% Add overall title
sgtitle('Multi-Robot Firefighting Potential Field Analysis', ...
        'FontSize', 16, 'FontWeight', 'bold');

%% ============================
%   6. Create 2D contour visualization
% =============================
fig2 = figure('Name', 'Potential Field Contours', 'Position', [150, 150, 1200, 500]);
set(fig2, 'Color', 'w');

%% 2D Contour: Fire Attractive
subplot(1, 3, 1);
contourf(X, Y, U_fire_attract, 20, 'LineColor', 'none');
hold on;
plot(env.fire_positions(1), env.fire_positions(2), ...
     'rp', 'MarkerSize', 18, 'MarkerFaceColor', 'y', 'LineWidth', 2);
colormap(gca, 'parula');
colorbar;
xlabel('x [m]', 'FontSize', 11);
ylabel('y [m]', 'FontSize', 11);
title('Fire Attractive Potential (2D)', 'FontSize', 12, 'FontWeight', 'bold');
axis equal tight;
grid on;

%% 2D Contour: Robot Repulsive
subplot(1, 3, 2);
contourf(X, Y, U_robot_repel, 20, 'LineColor', 'none');
hold on;
for i = 1:N
    plot(robot_positions(1, i), robot_positions(2, i), ...
         'bo', 'MarkerSize', 10, 'MarkerFaceColor', 'c', 'LineWidth', 2);
    cx = robot_positions(1, i) + env.R_s * cos(theta);
    cy = robot_positions(2, i) + env.R_s * sin(theta);
    plot(cx, cy, 'b--', 'LineWidth', 1.2);
end
colormap(gca, 'hot');
colorbar;
xlabel('x [m]', 'FontSize', 11);
ylabel('y [m]', 'FontSize', 11);
title('Robot Repulsive Potential (2D)', 'FontSize', 12, 'FontWeight', 'bold');
axis equal tight;
grid on;

%% 2D Contour: Combined Total
subplot(1, 3, 3);
contourf(X, Y, U_total, 20, 'LineColor', 'none');
hold on;
plot(env.fire_positions(1), env.fire_positions(2), ...
     'rp', 'MarkerSize', 18, 'MarkerFaceColor', 'y', 'LineWidth', 2);
for i = 1:N
    plot(robot_positions(1, i), robot_positions(2, i), ...
         'bo', 'MarkerSize', 10, 'MarkerFaceColor', 'c', 'LineWidth', 2);
end
% Draw safety radius
cx_safe = env.fire_positions(1) + env.R_safe_fire * cos(theta);
cy_safe = env.fire_positions(2) + env.R_safe_fire * sin(theta);
plot(cx_safe, cy_safe, 'r--', 'LineWidth', 1.5);
colormap(gca, 'jet');
colorbar;
xlabel('x [m]', 'FontSize', 11);
ylabel('y [m]', 'FontSize', 11);
title('Combined Total Potential (2D)', 'FontSize', 12, 'FontWeight', 'bold');
axis equal tight;
grid on;

sgtitle('2D Contour Views of Potential Fields', 'FontSize', 14, 'FontWeight', 'bold');

%% ============================
%   7. Create vector field visualization (gradient)
% =============================
fig3 = figure('Name', 'Gradient Vector Field', 'Position', [200, 200, 800, 700]);
set(fig3, 'Color', 'w');

% Compute gradient of total potential (this gives the force field)
[Fx, Fy] = gradient(-U_total, x_lin(2)-x_lin(1), y_lin(2)-y_lin(1));

% Downsample for clearer visualization
skip = 8;
X_sub = X(1:skip:end, 1:skip:end);
Y_sub = Y(1:skip:end, 1:skip:end);
Fx_sub = Fx(1:skip:end, 1:skip:end);
Fy_sub = Fy(1:skip:end, 1:skip:end);

% Plot vector field
quiver(X_sub, Y_sub, Fx_sub, Fy_sub, 1.5, 'b', 'LineWidth', 1.2);
hold on;

% Overlay contour of total potential
contour(X, Y, U_total, 15, 'LineColor', [0.7 0.7 0.7], 'LineWidth', 0.5);

% Mark fire
plot(env.fire_positions(1), env.fire_positions(2), ...
     'rp', 'MarkerSize', 20, 'MarkerFaceColor', 'y', 'LineWidth', 2);

% Mark robots
for i = 1:N
    plot(robot_positions(1, i), robot_positions(2, i), ...
         'bo', 'MarkerSize', 12, 'MarkerFaceColor', 'c', 'LineWidth', 2);
end

% Draw safety radius
cx_safe = env.fire_positions(1) + env.R_safe_fire * cos(theta);
cy_safe = env.fire_positions(2) + env.R_safe_fire * sin(theta);
plot(cx_safe, cy_safe, 'r--', 'LineWidth', 2);

xlabel('x [m]', 'FontSize', 12);
ylabel('y [m]', 'FontSize', 12);
title('Gradient Vector Field (Force Field)', 'FontSize', 14, 'FontWeight', 'bold');
axis equal tight;
grid on;
legend('Force vectors', 'Potential contours', 'Fire', 'Robots', 'Safety radius', ...
       'Location', 'best', 'FontSize', 10);

fprintf('\nVisualization complete!\n');
fprintf('Three figures generated:\n');
fprintf('  1. 3D surface plots of all potential components\n');
fprintf('  2. 2D contour plots\n');
fprintf('  3. Gradient vector field (force field)\n');
