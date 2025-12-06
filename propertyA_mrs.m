%% run_propertyA_mrs_full.m
% Combined Property A + C simulation with:
%   - Custom figure: heatmap, safety/suppression circles, fading fire
%   - Mobile Robotics Simulation Toolbox (MultiRobotEnv) visualization
%
% Requires:
%   env_setup.m
%   temperature_field.m
%   robot_dynamics.m
%   Mobile Robotics Simulation Toolbox on path

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

% Store initial positions for Survey Mode (orbiting)
env.initial_positions = x;

% Initialize spiral tracking
cumulative_angles = zeros(1, N);
% Previous angle relative to center (for tracking change)
% We need to define "angle relative to center". Since they start AT the center (or close),
% this is tricky. But wait, they start at x. And center IS x.
% So distance is 0.
% Let's assume they start at angle 0 relative to the center for calculation purposes,
% but physically they are at the center.
% Actually, if they are AT the center, atan2 is undefined/zero.
% As soon as they move, we can track.
prev_angles = zeros(1, N); 

%% ============================
%   4. Create MultiRobotEnv (toolbox visualization)
% =============================
mrsEnv = MultiRobotEnv(N);
mrsEnv.robotRadius      = 0.2;
mrsEnv.showTrajectory   = false;   % IMPORTANT: disable trajectories to avoid handle errors
mrsEnv.hasWaypoints     = false;
mrsEnv.hasLidar         = false;
mrsEnv.hasRobotDetector = false;
% Use default map; do not set mapName to avoid map errors

% Initialize robot poses in MR env
poses = [x; zeros(1, N)]; % [x; y; theta]
mrsEnv(1:N, poses);

%% ============================
%   5. Simulation parameters & logging arrays
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
%   6. Simulation loop
% =============================
% Setup integrated visualization
hold on;
h_heatmap = imagesc(x_lin, y_lin, T_grid);
set(gca, 'YDir', 'normal');
colormap('hot');
caxis([0 Tmax0]);
colorbar;
uistack(h_heatmap, 'bottom'); % Keep heatmap behind robots

% Bounding box
rectangle('Position', [x_min, y_min, env.world_size(1), env.world_size(2)], ...
          'EdgeColor', 'k', 'LineWidth', 2);
axis equal;
xlim([x_min, x_max]);
ylim([y_min, y_max]);
title('Property A + C Simulation');
xlabel('x [m]');
ylabel('y [m]');

theta = linspace(0, 2*pi, 100);  % for drawing circles

for k = 1:numSteps

    % ----- Calculate Dynamic Radii (Spiral) -----
    % Update cumulative angles
    for i = 1:N
        center = env.initial_positions(:, i);
        d_vec = x(:, i) - center;
        curr_angle = atan2(d_vec(2), d_vec(1));
        
        % Angular difference in [-pi, pi]
        d_theta = curr_angle - prev_angles(i);
        while d_theta <= -pi, d_theta = d_theta + 2*pi; end
        while d_theta > pi,   d_theta = d_theta - 2*pi; end
        
        % Accumulate absolute angle change (so it grows regardless of direction)
        % OR just accumulate d_theta if we know they orbit CCW.
        % The tangential force is CCW ([-y, x]), so d_theta should be positive.
        % Let's use abs to be safe and ensure growth.
        cumulative_angles(i) = cumulative_angles(i) + abs(d_theta);
        prev_angles(i) = curr_angle;
    end
    
    % R = R_start + (Angle / 2pi) * Step
    R_dyn = env.R_survey + (cumulative_angles / (2*pi)) * env.spiral_step;

    % ----- Robot Communication: Broadcast Fire Location -----
    broadcast_target = []; % Empty = no broadcast
    for i = 1:N
        [~, gradT_i] = temperature_field(x(:,i), env);
        if norm(gradT_i) > env.grad_thresh
            % Robot i found fire, broadcast its position
            broadcast_target = x(:, i);
            break; % Only need one broadcast
        end
    end

    % ----- Property A dynamics: compute velocities -----
    xdot = robot_dynamics(x, env, R_dyn, broadcast_target);

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

    % ----- Update MultiRobotEnv poses (toolbox visualization) -----
    poses = [x; zeros(1, N)];
    mrsEnv(1:N, poses);

    % ----- Custom visualization: heatmap + circles + robots -----
    if mod(k, env.plot_refresh) == 1 || k == 1
        % Update heatmap data
        % Recompute heatmap with current env.fire_intensities
        T_grid = zeros(ny, nx);
        for ix = 1:nx
            for iy = 1:ny
                pos = [x_lin(ix); y_lin(iy)];
                [T_here, ~] = temperature_field(pos, env);
                T_grid(iy, ix) = T_here;
            end
        end
        set(h_heatmap, 'CData', T_grid);
        
        % Update fire markers and circles (clear and redraw only these if needed, 
        % or use handle graphics for better performance. For simplicity, we'll 
        % just delete the old markers and redraw them, but keep the heatmap)
        
        % Delete old markers if they exist
        if exist('h_markers', 'var')
            delete(h_markers);
        end
        h_markers = [];
        
        % Plot fire hotspot(s), safety circles, and suppression radius
        for f = 1:env.num_fires
            fx = env.fire_positions(1, f);
            fy = env.fire_positions(2, f);

            % Fire marker
            h_markers = [h_markers, plot(fx, fy, 'rp', 'MarkerSize', 14, 'MarkerFaceColor', 'y')];

            % Safety circle (no-go)
            cx_safe = fx + env.R_safe_fire * cos(theta);
            cy_safe = fy + env.R_safe_fire * sin(theta);
            h_markers = [h_markers, plot(cx_safe, cy_safe, 'w--', 'LineWidth', 1.5)];

            % Suppression radius (where robots contribute to extinction)
            cx_sup = fx + env.R_sup * cos(theta);
            cy_sup = fy + env.R_sup * sin(theta);
            h_markers = [h_markers, plot(cx_sup, cy_sup, 'g-.', 'LineWidth', 1.2)];
        end
        
        drawnow;
    end

    pause(0.01);   % slow down a bit so both windows are viewable
end

%% ============================
%   7. Post-simulation plots
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
