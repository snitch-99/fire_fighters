function env = env_setup()
% ENV_SETUP
% Returns a struct containing all basic environment parameters
% for the multi-robot firefighting simulation (Property A, C, and full system).

    %% ============================
    %   WORLD / WORKSPACE INFO
    % =============================
    env.world_size = [40, 40];      % Workspace size in meters (X by Y)

    %% ============================
    %   ROBOT TEAM INFO
    % =============================
    env.N = 6;                      % Number of robots
    env.initial_pos_range = [-5 5;  % Range for sampling initial positions (x) - center region
                               -5 5]; % (y)

    %% ============================
    %   FIRE HOTSPOTS (GENERAL)
    % =============================
    env.num_fires = 1;              % 4 fires at corners
    
    % Each column is [x; y] position of one fire
    % Corners of 40x40 world: (-20,-20), (20,-20), (-20,20), (20,20)
    % Offset slightly inward to avoid wall issues
    env.fire_positions = [0, 0, -10,  10; 
                          -10, -10, 10,  10];    % 2 x num_fires
    
    % Initial fire intensities I_k(0)
    env.fire_intensities = 1.0 * ones(1, env.num_fires);
    
    % Spread (sigma) for each fire
    env.fire_spreads = 2.0 * ones(1, env.num_fires);
    
    % ---- NEW: fire dynamics parameters (Property C) ----
    env.I_min = 0.1;      % safe intensity threshold
    env.beta  = 0.02;     % suppression rate per robot
    env.R_sup = 2.2;      % suppression radius (where robots actually fight the fire)
    
    %% ============================
    %   CONTROL / INTERACTION PARAMETERS
    % =============================
    env.k_T   = 1.0;      % Gain for following ∇T
    env.k_rep = 3.0;      % Robot-robot repulsion strength
    env.R_s   = 1.0;      % Robot-robot repulsion radius
    
    % Safety radius around fire (no-go zone)
    env.R_safe_fire = 2.0;
    env.k_fire_rep  = 5.0;
    
    % Random walk gain (roaming speed)
    env.k_rand = 0.5;
    
    % Temperature threshold for detection (Mode Switching)
    env.T_detect = 0.05;
    
    % Circular Survey Parameters
    env.grad_thresh = 0.05; % Gradient magnitude threshold
    env.R_survey    = 2.0;  % Radius of survey circle
    env.k_survey    = 1.0;  % Speed of orbiting
    env.spiral_step = 1.0;  % Increase in radius per full circle (Spiral)
    
    % Fixed start positions (2xN) - near center of arena
    env.start_positions = [-3, -3, -3,  3,  3,  3;
                           -3,  0,  3, -3,  0,  3];
                            
    % Wall Constraints
    env.wall_margin = 1.0;
    env.k_wall      = 10.0;


    %% ============================
    %   SIMULATION PARAMETERS
    % =============================
    env.dt      = 0.05;   % Time step for integration (seconds)
    env.T_final = 200;     % Total simulation time (seconds)

    %% ============================
    %   VISUALIZATION PARAMETERS
    % =============================
    env.plot_refresh = 5; % Plot every N simulation steps

end
