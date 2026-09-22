function cfg = config()
% CONFIG Returns central configuration parameters for the 3D Autonomous Drone Simulation
% Designed for high-fidelity physics, realistic perception, smooth navigation, and UI.

%% Time & Simulation Step
cfg.sim.dt = 0.04;              % Simulation timestep (25 Hz update loop)
cfg.sim.maxTime = 90.0;         % Maximum mission time in seconds
cfg.sim.realTimeSync = true;    % Synchronize with real-time clock
cfg.sim.speedMultiplier = 1.0;  % Playback speed (1.0 = normal, 1.5 = fast)

%% Physical Drone Properties
cfg.drone.mass = 1.5;           % Total mass (kg)
cfg.drone.armLength = 0.28;     % Center-to-motor arm length (m)
cfg.drone.bodyRadius = 0.40;    % Physical bounding radius (m)
cfg.drone.height = 0.18;        % Physical height (m)
cfg.drone.safetyMargin = 1.20;  % Additional safety buffer around obstacles (m)
cfg.drone.inflationRadius = cfg.drone.bodyRadius + cfg.drone.safetyMargin; % Total obstacle clearance (m)

% Dynamic Limits
cfg.drone.maxSpeedHoriz = 6.0;  % Maximum horizontal velocity (m/s)
cfg.drone.maxSpeedVert = 2.5;   % Maximum climb/descent velocity (m/s)
cfg.drone.maxAccelHoriz = 3.0;  % Max horizontal acceleration (m/s^2)
cfg.drone.maxAccelVert = 2.5;   % Max vertical acceleration (m/s^2)
cfg.drone.maxTiltAngle = 0.45;  % Maximum bank/pitch angle (approx 25.8 deg)
cfg.drone.maxYawRate = 1.8;     % Maximum yaw angular velocity (rad/s)
cfg.drone.g = 9.81;             % Gravitational acceleration (m/s^2)

%% Sensor Suite Parameters
% 3D LiDAR Scanner
cfg.lidar.enabled = true;
cfg.lidar.maxRange = 22.0;      % Maximum detection distance (m)
cfg.lidar.numRaysHoriz = 32;    % Radial beam count (360 deg sweep)
cfg.lidar.elevations = [-25, -15, -5, 0, 5, 15, 25]; % Vertical scanning rings (deg)
cfg.lidar.updateRate = 15;      % Update frequency (Hz)
cfg.lidar.noiseSigma = 0.03;    % Range measurement Gaussian noise (m)

% Global Positioning System (GPS)
cfg.gps.noiseSigmaHoriz = 0.12; % Horizontal position error (m)
cfg.gps.noiseSigmaVert = 0.20;  % Vertical position error (m)

% Inertial Measurement Unit (IMU)
cfg.imu.accelNoise = 0.05;      % Accelerometer noise (m/s^2)
cfg.imu.gyroNoise = 0.015;      % Gyroscope angular rate noise (rad/s)

% Altimeter
cfg.altimeter.noiseSigma = 0.04;% Altitude measurement noise (m)

%% Cascaded Flight Controller Gains (PID)
% Position Control (Outer Loop -> Desired Velocity & Acceleration)
cfg.ctrl.kp_pos = [2.2, 2.2, 2.8];  % [X, Y, Z]
cfg.ctrl.ki_pos = [0.05, 0.05, 0.10];
cfg.ctrl.kd_pos = [1.6, 1.6, 1.8];

% Attitude Control (Inner Loop -> Banking and Tilting response)
cfg.ctrl.kp_att = [6.5, 6.5, 4.0];  % [Roll, Pitch, Yaw]
cfg.ctrl.kd_att = [1.8, 1.8, 1.2];

%% Occupancy Map & Autonomous Path Planning
cfg.map.boundsX = [-5, 85];     % Map spatial limits in X (m)
cfg.map.boundsY = [-5, 85];     % Map spatial limits in Y (m)
cfg.map.boundsZ = [0, 24];      % Map spatial limits in Z (m)
cfg.map.resolution = 1.0;       % Voxel grid resolution (m)
cfg.map.safeFlightCeiling = 20; % Max nominal cruise altitude (m)
cfg.map.safeFlightFloor = 2.0;  % Min cruise altitude above ground (m)

% Autonomous Route Parameters (Note: Internal planner is strictly encapsulated)
cfg.planner.gridStep = 1.0;     % Search node lattice spacing (m)
cfg.planner.replanDistance = 14;% Obstacle proximity trigger for replanning (m)
cfg.planner.emergencyDist = 4.0;% Emergency braking trigger distance (m)

%% Visual Theme & Color Palette
cfg.theme.bgColor = [0.06, 0.08, 0.12];        % Deep space navy
cfg.theme.gridColor = [0.15, 0.22, 0.32];       % Subtle tech grid
cfg.theme.groundColor = [0.08, 0.11, 0.16];     % Dark matte ground
cfg.theme.bldgWallColor = [0.22, 0.28, 0.38];   % Matte slate facade
cfg.theme.bldgRoofColor = [0.14, 0.18, 0.26];   % Roof trim
cfg.theme.windowGlow = [0.40, 0.75, 0.95];      % Cyan office illumination
cfg.theme.pathPlanned = [0.00, 0.85, 1.00];     % Electric cyan
cfg.theme.pathReplanned = [0.00, 0.95, 0.55];   % Neon emerald
cfg.theme.droneTrail = [1.00, 0.80, 0.20];      % Amber gold trail
cfg.theme.lidarRays = [0.00, 0.70, 0.90];       % Cyan laser beams
cfg.theme.lidarHits = [1.00, 0.30, 0.15];       % Crimson alert points
cfg.theme.dynamicObs = [0.95, 0.20, 0.20];      % Hazard warning crimson
cfg.theme.hudBg = [0.08, 0.10, 0.15];           % Mission control HUD panel
cfg.theme.hudText = [0.90, 0.94, 0.98];         % Crisp white-blue text
cfg.theme.hudAccent = [0.00, 0.80, 1.00];       % Cyan accent
cfg.theme.hudAlert = [1.00, 0.35, 0.20];        % Red-orange alert

%% Camera Modes
% 1: Follow Drone (Chase Camera)
% 2: Top-Down Tactical (Overhead)
% 3: Isometric Overview
% 4: Free Orbit
cfg.cam.defaultMode = 1;
cfg.cam.chaseDistance = 12.0;    % Camera distance behind drone (m)
cfg.cam.chaseAltitude = 5.5;     % Camera height above drone (m)
cfg.cam.smoothFactor = 0.15;     % Camera smoothing interpolation weight

end
