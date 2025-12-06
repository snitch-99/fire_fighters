%% run_propertyA.m
% Combined Property A + C:
%  - Aggregation toward fire using gradient + repulsion + safety radius
%  - Fire intensity decays when robots are within suppression radius
%
% Logs:
%   - Distance of each robot to nearest fire over time
%   - Min and mean inter-robot distances over time
%   - Fire intensity over time
% and produces plots at the end.

clear; clc; close all;

%% ============================
%   1. Load environment
% =============================
env = env_setup();
N   = env.N;

% World bounds (centered at 0,0)
x_min = -env.world_size(1)/2;
x_max =  env.world_size(1)/2;
y_min = -env.world_size(2)/2;
y_max =  env.world_size(2)/2;

%% ============================
%   2. Grid for temperature field heatmap
% =============================
nx = 80;   % grid resolution in x
ny = 80;   % grid resolution in y

x_lin = linspace(x_min, x_max, nx);
y_lin = linspace(y_min, y_max, ny);

% Initial heatmap for reference color scaling
T_grid = zeros(ny, nx);
for ix = 1:nx
    for iy = 1:ny
        pos = [x_lin(ix); y_lin(iy)];
        [T_here, ~] = temperature_field(pos, env);
        T_grid(iy, ix) = T_here;
    end
end
Tmax0 = max(T_grid(:));   % store initial max temperature for fixed colormap

%% ============================
%   3. Initialize robot positions
% =============================
x = zeros(2, N);   % 2 x N

for i = 1:N
    x(1, i) = env.initial_pos_range(1,1) + ...
              (env.initial_pos_range(1,2) - env.initial_pos_range(1,1))*rand;
    x(2, i) = env.initial_pos_range(2,1) + ...
              (env.initial_pos_range(2,2) - env.initial_pos_range(2,1))*rand;
end

%% ============================
%   4. Simulation parameters & logging arrays
% =============================
dt       = env.dt;
T_final  = env.T_final;
numSteps = round(T_final / dt);

t_hist = zeros(1, numSteps+1);         % time history
dist_to_fire  = zeros(N, numSteps+1);  % distance of each robot to nearest fire
min_pair_dist = zeros(1, numSteps+1);  % minimum inter-robot distance
mean_pair_dist= zeros(1, numSteps+1);  % mean inter-robot distance
I_hist        = zeros(env.num_fires, numSteps+1);  % fire intensities over time

% Initial time and log index
t = 0;
log_idx = 1;

% ----- Log initial metrics -----
t_hist(log_idx) = t;
I_hist(:, log_idx) = env.fire_intensities(:);

% Distances to nearest fire (initial)
for i = 1:N
    xi = x(:, i);
    d_min = inf;
    for f = 1:env.num_fires
        fk = env.fire_positions(:, f);
        d  = norm(xi - fk);
        if d < d_min
            d_min = d;
        end
    end
    dist_to_fire(i, log_idx) = d_min;
end

% Inter-robot distances (initial)
pair_dists = [];
for i = 1:N-1
    for j = i+1:N
        dij = norm(x(:,i) - x(:,j));
        pair_dists(end+1) = dij; %#ok<SAGROW>
    end
end
min_pair_dist(log_idx)  = min(pair_dists);
mean_pair_dist(log_idx) = mean(pair_dists);

%% ============================
%   5. Simulation loop
% =============================
figure('Name','Property A + C: Aggregation and Fire Extinction');
set(gcf,'Color','w');

theta = linspace(0, 2*pi, 100);  % for drawing circles

for k = 1:numSteps

    % ----- Compute velocities via Property A dynamics -----
    xdot = robot_dynamics(x, env);

    % ----- Integrate (Euler) -----
    x = x + dt * xdot;
    t = t + dt;

    % ----- Property C: update fire intensities based on suppressing robots -----
    for f = 1:env.num_fires
        fk = env.fire_positions(:, f);

        % Count robots within suppression radius R_sup
        N_sup = 0;
        for i = 1:N
            if norm(x(:,i) - fk) <= env.R_sup
                N_sup = N_sup + 1;
            end
        end

        % Intensity update: I_k(t+dt) = I_k(t) - beta * N_sup * dt
        I_new = env.fire_intensities(f) - env.beta * N_sup * dt;

        % Clamp at I_min (safe / extinguished)
        env.fire_intensities(f) = max(env.I_min, I_new);
    end

    % ----- Log metrics -----
    log_idx = log_idx + 1;
    t_hist(log_idx) = t;
    I_hist(:, log_idx) = env.fire_intensities(:);

    % Distances to nearest fire
    for i = 1:N
        xi = x(:, i);
        d_min = inf;
        for f = 1:env.num_fires
            fk = env.fire_positions(:, f);
            d  = norm(xi - fk);
            if d < d_min
                d_min = d;
            end
        end
        dist_to_fire(i, log_idx) = d_min;
    end

    % Inter-robot distances (all pairs)
    pair_dists = [];
    for i = 1:N-1
        for j = i+1:N
            dij = norm(x(:,i) - x(:,j));
            pair_dists(end+1) = dij; %#ok<SAGROW>
        end
    end
    min_pair_dist(log_idx)  = min(pair_dists);
    mean_pair_dist(log_idx) = mean(pair_dists);

    % ----- Visualization (update every env.plot_refresh steps) -----
    if mod(k, env.plot_refresh) == 1 || k == 1
        clf;

        % Recompute heatmap with current env.fire_intensities
        T_grid = zeros(ny, nx);
        for ix = 1:nx
            for iy = 1:ny
                pos = [x_lin(ix); y_lin(iy)];
                [T_here, ~] = temperature_field(pos, env);
                T_grid(iy, ix) = T_here;
            end
        end

        % Heatmap of temperature field
        imagesc(x_lin, y_lin, T_grid);
        set(gca, 'YDir', 'normal');   % y increases upward
        colormap('hot');
        caxis([0 Tmax0]);             % FIXED scale so colors fade as intensity drops
        colorbar;
        hold on;

        % Bounding box for environment
        rectangle('Position', [x_min, y_min, env.world_size(1), env.world_size(2)], ...
                  'EdgeColor', 'k', 'LineWidth', 2);

        % Plot fire hotspot(s), safety circles, and suppression radius
        for f = 1:env.num_fires
            fx = env.fire_positions(1, f);
            fy = env.fire_positions(2, f);

            % Fire marker
            plot(fx, fy, 'rp', 'MarkerSize', 14, 'MarkerFaceColor', 'y');

            % Safety circle (no-go)
            cx_safe = fx + env.R_safe_fire * cos(theta);
            cy_safe = fy + env.R_safe_fire * sin(theta);
            plot(cx_safe, cy_safe, 'w--', 'LineWidth', 1.5);

            % Suppression radius (where robots contribute to extinction)
            cx_sup = fx + env.R_sup * cos(theta);
            cy_sup = fy + env.R_sup * sin(theta);
            plot(cx_sup, cy_sup, 'g-.', 'LineWidth', 1.2);
        end

        % Plot robots
        plot(x(1,:), x(2,:), 'bo', 'MarkerSize', 8, 'MarkerFaceColor', 'c');

        % Formatting
        axis equal;
        xlim([x_min, x_max]);
        ylim([y_min, y_max]);
        title(sprintf('Property A + C: t = %.2f s', t), 'FontSize', 12);
        xlabel('x [m]');
        ylabel('y [m]');

        drawnow;
    end
end

%% ============================
%   6. Post-simulation plots
% =============================

% Trim histories
t_hist         = t_hist(1:log_idx);
dist_to_fire   = dist_to_fire(:, 1:log_idx);
min_pair_dist  = min_pair_dist(1:log_idx);
mean_pair_dist = mean_pair_dist(1:log_idx);
I_hist         = I_hist(:, 1:log_idx);

% ---- Plot 1: distance to nearest fire for each robot ----
figure('Name','Distance to Nearest Fire vs Time');
set(gcf,'Color','w');
hold on;
for i = 1:N
    plot(t_hist, dist_to_fire(i, :), 'LineWidth', 1.2);
end
yline(env.R_safe_fire, '--k', 'Safety radius', 'LineWidth', 1.2);
yline(env.R_sup, 'g-.', 'Suppression radius', 'LineWidth', 1.2);
xlabel('Time [s]');
ylabel('Distance to nearest fire [m]');
title('Robot Distances to Nearest Fire Over Time');
legendStrings = arrayfun(@(i) sprintf('Robot %d', i), 1:N, 'UniformOutput', false);
legend([legendStrings, {'R_{safe}', 'R_{sup}'}], 'Location', 'bestoutside');
grid on;

% ---- Plot 2: min and mean inter-robot distance vs time ----
figure('Name','Inter-Robot Distances vs Time');
set(gcf,'Color','w');
plot(t_hist, min_pair_dist, 'LineWidth', 1.5);
hold on;
plot(t_hist, mean_pair_dist, 'LineWidth', 1.5);
yline(env.R_s, '--k', 'Spacing radius R_s', 'LineWidth', 1.2);
xlabel('Time [s]');
ylabel('Distance [m]');
title('Inter-Robot Distances Over Time');
legend('Min pairwise distance', 'Mean pairwise distance', 'R_s', ...
       'Location', 'best');
grid on;

% ---- Plot 3: fire intensity vs time ----
figure('Name','Fire Intensity vs Time');
set(gcf,'Color','w');
hold on;
for f = 1:env.num_fires
    plot(t_hist, I_hist(f, :), 'LineWidth', 1.5);
end
yline(env.I_min, '--k', 'I_{min}', 'LineWidth', 1.2);
xlabel('Time [s]');
ylabel('Intensity I_k(t)');
title('Fire Intensity Over Time (Property C)');
legendStringsF = arrayfun(@(f) sprintf('Fire %d', f), 1:env.num_fires, 'UniformOutput', false);
legend([legendStringsF, {'I_{min}'}], 'Location', 'best');
grid on;
