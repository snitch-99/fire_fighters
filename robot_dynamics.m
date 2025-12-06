function xdot = robot_dynamics(x, env, R_dyn, broadcast_target)
% ROBOT_DYNAMICS
% Computes the time derivative of robot positions (velocities) for
% the aggregation-with-spacing behavior (Property A) with a safety
% radius around the fire.
%
% Inputs:
%   x               : 2 x N matrix of robot positions, each column is [x; y] of robot i
%   env             : struct from env_setup()
%   R_dyn           : (Optional) 1 x N vector of dynamic survey radii. Defaults to env.R_survey.
%   broadcast_target: (Optional) 2x1 broadcast fire location. Empty = no broadcast.
%         - N            : number of robots
%         - k_T          : gain for temperature gradient
%         - k_rep        : robot-robot repulsion strength
%         - R_s          : robot-robot repulsion radius
%         - num_fires    : number of hotspots
%         - fire_positions : 2 x num_fires array
%         - R_safe_fire  : safety radius around fire
%         - k_fire_rep   : fire repulsion strength
%
% Output:
%   xdot : 2 x N matrix of position derivatives (velocities)

    % Number of robots
    N = env.N;
    
    % Handle optional R_dyn
    if nargin < 3
        R_dyn = env.R_survey * ones(1, N);
    end
    
    % Handle optional broadcast_target
    if nargin < 4
        broadcast_target = [];
    end

    % Ensure x is 2 x N
    if size(x, 1) ~= 2 && size(x, 2) == 2
        % If user passed N x 2, transpose to 2 x N
        x = x.';
    elseif size(x, 1) ~= 2
        error('Input x must be of size 2xN or Nx2.');
    end

    % Preallocate velocity matrix
    xdot = zeros(2, N);

    for i = 1:N
        xi = x(:, i);

        % ====== Fire-seeking term: follow temperature gradient ======
        [T_i, gradT_i] = temperature_field(xi, env);   % 2x1 gradient at xi
        
        % --- MODE SWITCHING LOGIC ---
        grad_mag = norm(gradT_i);
        
        if grad_mag > env.grad_thresh
            % FIREFIGHT MODE: Attracted to fire
            F_grad = env.k_T * gradT_i;
            F_survey = [0; 0];
        else
            % SURVEY MODE
            if ~isempty(broadcast_target)
                % Navigate to broadcast location
                d_vec = broadcast_target - xi;
                dist = norm(d_vec);
                if dist > 0.1
                    F_survey = env.k_survey * (d_vec / dist);
                else
                    F_survey = [0; 0]; % Reached target
                end
            else
                % Normal orbit around initial position
                center = env.initial_positions(:, i);
                d_vec = xi - center;
                dist = norm(d_vec);
                
                if dist < 1e-3
                    d_vec = [1; 0]; dist = 1; % Avoid singularity
                end
                
                % Radial component: pull to R_dyn(i) (Spiral/Circle)
                u_rad = -(dist - R_dyn(i)) * (d_vec / dist);
                
                % Tangential component: orbit
                u_tan = env.k_survey * [-d_vec(2); d_vec(1)] / dist;
                
                F_survey = u_rad + u_tan;
            end
            F_grad = [0; 0];
        end

        % ====== Robot-robot repulsive term: spacing ======
        F_rep_total = [0; 0];
        for j = 1:N
            if j == i
                continue;
            end

            xj  = x(:, j);
            dx  = xi - xj;
            dij = norm(dx);

            if dij < env.R_s && dij > 1e-6
                F_ij = env.k_rep * (1/dij - 1/env.R_s) * (dx / dij);
                F_rep_total = F_rep_total + F_ij;
            end
        end

        % ====== NEW: Fire safety repulsion term ======
        F_fire_rep_total = [0; 0];

        for k = 1:env.num_fires
            fk  = env.fire_positions(:, k);   % fire k position [x; y]
            dxf = xi - fk;
            dfk = norm(dxf);                  % distance to fire k

            % Repel if inside safety radius (and avoid singularity)
            if dfk < env.R_safe_fire && dfk > 1e-6
                F_fire_k = env.k_fire_rep * (1/dfk - 1/env.R_safe_fire) * (dxf / dfk);
                F_fire_rep_total = F_fire_rep_total + F_fire_k;
            end
        end

        % ====== Total velocity for robot i ======
        % Note: F_survey is now handled inside the mode switching block
        xdot(:, i) = F_grad + F_survey + F_rep_total + F_fire_rep_total;
    end
end
