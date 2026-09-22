function droneGraphics = renderDrone3D(ax, cfg)
% RENDERDRONE3D Constructs a high-fidelity 3D quadcopter model using MATLAB graphics.
%
% Uses hgtransform hierarchical scene graph for 60+ FPS real-time rendering.
%
% Architecture:
%   ax
%    └─ droneRoot (hgtransform) -> translates to [x, y, z] and rotates (yaw, pitch, roll)
%        ├─ Central fuselage chassis (patch)
%        ├─ LiDAR perception dome (surface)
%        ├─ 4 Carbon-fiber arms (cylinders/lines)
%        ├─ 4 Motor nacelles (cylinders)
%        ├─ Forward directional arrow / canopy (patch)
%        ├─ Navigation LED beacons (scatter3)
%        └─ 4 Propeller sub-assemblies (hgtransform -> rotates each blade around motor axis)

hold(ax, 'on');

% 1. Create root transform group
droneRoot = hgtransform('Parent', ax);

L = cfg.drone.armLength; % 0.28 m
armRadius = 0.015; %#ok<NASGU> % named constant kept for geometric clarity
motorRadius = 0.035;
motorHeight = 0.035;
bladeRadius = 0.16;

% 2. Central Fuselage Body (Sleek aerodynamic pod)
% Body dimensions: 0.20m x 0.14m x 0.06m
bx = 0.11; by = 0.08; bz = 0.035;
[Xb, Yb, Zb] = ellipsoid(0, 0, 0, bx, by, bz, 16);
hBody = surf(droneRoot, Xb, Yb, Zb, ...
             'FaceColor', [0.12, 0.15, 0.20], ... % Dark stealth carbon
             'EdgeColor', [0.25, 0.35, 0.45], ...
             'FaceAlpha', 0.98);

% 3. Sensor Dome / LiDAR Turret on top
[Xs, Ys, Zs] = sphere(12);
hTurret = surf(droneRoot, Xs*0.035, Ys*0.035, Zs*0.025 + bz, ...
               'FaceColor', [0.0, 0.8, 1.0], ... % Glowing cyan sensor glass
               'EdgeColor', 'none', ...
               'FaceAlpha', 0.90);

% 4. Forward Cockpit / Directional Chevron Indicator
chevX = [bx, bx + 0.07, bx];
chevY = [-0.035, 0, 0.035];
chevZ = [0.005, 0.005, 0.005];
hChevron = patch(droneRoot, 'XData', chevX, 'YData', chevY, 'ZData', chevZ, ...
                 'FaceColor', [0.0, 0.85, 1.0], 'EdgeColor', 'w', ...
                 'LineWidth', 1.2);

% 5. Four Carbon Arms (X-configuration: 45, 135, 225, 315 degrees)
armAngles = [pi/4, 3*pi/4, 5*pi/4, 7*pi/4];
armEnds = zeros(4, 3);
hArms   = gobjects(4, 1); % pre-allocated
hMotors = gobjects(4, 1); % pre-allocated
hProps = gobjects(4, 1);

for i = 1:4
    ang = armAngles(i);
    ex = L * cos(ang);
    ey = L * sin(ang);
    armEnds(i, :) = [ex, ey, 0];
    
    % Carbon tubular arm line
    hArm = line(droneRoot, [0, ex], [0, ey], [0, 0], ...
                'Color', [0.25, 0.28, 0.35], 'LineWidth', 3.5);
    hArms(i) = hArm;
    
    % Motor Nacelle (brushless motor housing)
    [Xm, Ym, Zm] = cylinder(motorRadius, 12);
    Xm = Xm + ex;
    Ym = Ym + ey;
    Zm = Zm * motorHeight;
    hMotor = surf(droneRoot, Xm, Ym, Zm, ...
                  'FaceColor', [0.35, 0.40, 0.48], ... % Anodized metal
                  'EdgeColor', 'none');
    hMotors(i) = hMotor;
    
    % Propeller Transform Group (child of droneRoot, centered at motor hub)
    propTrans = hgtransform('Parent', droneRoot);
    set(propTrans, 'Matrix', makehgtform('translate', [ex, ey, motorHeight + 0.005]));
    hProps(i) = propTrans;
    
    % Propeller Blades (2 aerodynamic tapered airfoils)
    
    % Blade color: Front motors cyan/bright, Rear motors orange/stealth
    if i == 1 || i == 4 % Front (+X facing)
        bladeCol = [0.0, 0.85, 1.0];
    else % Rear (-X facing)
        bladeCol = [1.0, 0.55, 0.1];
    end
    
    patch(propTrans, 'XData', [-bladeRadius, -bladeRadius*0.2, 0, bladeRadius*0.2, bladeRadius, bladeRadius*0.2, 0, -bladeRadius*0.2], ...
                     'YData', [0, -0.018, 0, 0.018, 0, -0.018, 0, 0.018], ...
                     'ZData', zeros(1, 8), ...
                     'FaceColor', bladeCol, 'EdgeColor', [0.2 0.2 0.2], ...
                     'FaceAlpha', 0.85, 'LineWidth', 0.5);
    
    % Center hub spinner
    [Xh, Yh, Zh] = cylinder(0.012, 8);
    Zh = Zh * 0.015;
    surf(propTrans, Xh, Yh, Zh, 'FaceColor', [0.9 0.9 0.9], 'EdgeColor', 'none');
end

% 6. Navigation LED Beacons
% Front Right (Starboard) - Green
scatter3(droneRoot, armEnds(1, 1), armEnds(1, 2), -0.01, 40, [0.0 1.0 0.3], 'filled');
% Front Left (Port) - Red
scatter3(droneRoot, armEnds(4, 1), armEnds(4, 2), -0.01, 40, [1.0 0.1 0.1], 'filled');
% Rear Strobe - White
scatter3(droneRoot, -bx, 0, -0.01, 35, [1.0 1.0 1.0], 'filled');

% Package return struct
droneGraphics.droneRoot = droneRoot;
droneGraphics.hProps = hProps;
droneGraphics.armEnds = armEnds;
droneGraphics.hBody = hBody;
droneGraphics.hTurret = hTurret;
droneGraphics.hChevron = hChevron;
droneGraphics.hArms = hArms;
droneGraphics.hMotors = hMotors;

end
