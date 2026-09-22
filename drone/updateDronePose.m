function updateDronePose(droneGraphics, pos, euler, propAngles)
% UPDATEDRONEPOSE Updates position, attitude, and propeller spins of 3D quadcopter.
%
% Inputs:
%   droneGraphics - Graphics handle structure returned by renderDrone3D
%   pos           - [x; y; z] Drone position in world frame
%   euler         - [phi; theta; psi] Roll, pitch, yaw in radians
%   propAngles    - [theta1, theta2, theta3, theta4] Propeller rotation angles

if isempty(droneGraphics) || ~isfield(droneGraphics, 'droneRoot') || ~isvalid(droneGraphics.droneRoot)
    return;
end

phi = euler(1);
theta = euler(2);
psi = euler(3);

% Root transform: Translation followed by Yaw (Z), Pitch (Y), Roll (X)
% Note: In makehgtform, order of operations is right-to-left:
% T = Translation * Rz * Ry * Rx
T_drone = makehgtform('translate', [pos(1), pos(2), pos(3)], ...
                      'zrotate', psi, ...
                      'yrotate', theta, ...
                      'xrotate', phi);

set(droneGraphics.droneRoot, 'Matrix', T_drone);

% Propeller spins relative to motor nacelles
armEnds = droneGraphics.armEnds;
motorHeight = 0.035;

for i = 1:4
    if isvalid(droneGraphics.hProps(i))
        T_prop = makehgtform('translate', [armEnds(i, 1), armEnds(i, 2), motorHeight + 0.005], ...
                             'zrotate', propAngles(i));
        set(droneGraphics.hProps(i), 'Matrix', T_prop);
    end
end

end
