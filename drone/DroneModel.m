classdef DroneModel < handle
    % DRONEMODEL Simulates 6-DOF quadcopter physics, kinematics, and propeller dynamics.
    
    properties
        cfg
        
        % State Vector: [x, y, z, vx, vy, vz, phi, theta, psi, p, q, r]
        pos = [0; 0; 0]      % [x; y; z] (m)
        vel = [0; 0; 0]      % [vx; vy; vz] (m/s)
        accel = [0; 0; 0]    % [ax; ay; az] (m/s^2)
        euler = [0; 0; 0]    % [phi; theta; psi] (rad) [roll, pitch, yaw]
        omega = [0; 0; 0]    % [p; q; r] (rad/s)
        
        % Propeller visual phases (rad)
        propAngles = [0, 0, 0, 0]
        propSpeed = 120.0    % Base angular velocity (rad/s)
        
        % Motor thrusts (N)
        thrust = 14.715      % Total thrust (N) [m*g at hover]
        armed = false
        
        % Actuator time constants
        tauAttitude = 0.08   % Attitude response time (s)
    end
    
    methods
        function obj = DroneModel(cfg, initialPos, initialYaw)
            obj.cfg = cfg;
            if nargin >= 2 && ~isempty(initialPos)
                obj.pos = reshape(initialPos(1:3), [3, 1]);
            else
                obj.pos = [0; 0; 0];
            end
            
            if nargin >= 3 && ~isempty(initialYaw)
                obj.euler(3) = initialYaw;
            end
            
            obj.thrust = obj.cfg.drone.mass * obj.cfg.drone.g;
        end
        
        function step(obj, dt, thrustCmd, eulerCmd, windVector)
            % Advances quadcopter physics by one simulation step dt
            
            if nargin < 5 || isempty(windVector)
                windVector = [0; 0; 0];
            else
                windVector = reshape(windVector, [3, 1]);
            end
            
            if ~obj.armed
                % On ground / idle
                obj.vel = [0; 0; 0];
                obj.accel = [0; 0; 0];
                obj.euler = [0; 0; obj.euler(3)];
                obj.propAngles = obj.propAngles + 5.0 * dt * [1, -1, 1, -1];
                return;
            end
            
            % Clamp commanded attitude to physical safety limits
            maxTilt = obj.cfg.drone.maxTiltAngle;
            phi_c = max(-maxTilt, min(maxTilt, eulerCmd(1)));
            theta_c = max(-maxTilt, min(maxTilt, eulerCmd(2)));
            psi_c = eulerCmd(3);
            
            % Smooth attitude tracking (first-order closed-loop model)
            dPhi = (phi_c - obj.euler(1)) / obj.tauAttitude;
            dTheta = (theta_c - obj.euler(2)) / obj.tauAttitude;
            
            % Yaw error with wraparound [-pi, pi]
            yawErr = angdiff(obj.euler(3), psi_c);
            dPsi = max(-obj.cfg.drone.maxYawRate, min(obj.cfg.drone.maxYawRate, yawErr / obj.tauAttitude));
            
            obj.euler(1) = obj.euler(1) + dPhi * dt;
            obj.euler(2) = obj.euler(2) + dTheta * dt;
            obj.euler(3) = wrapToPi(obj.euler(3) + dPsi * dt);
            
            % Clamp thrust
            m = obj.cfg.drone.mass;
            g = obj.cfg.drone.g;
            minThrust = 0.2 * m * g;
            maxThrust = 2.4 * m * g;
            obj.thrust = max(minThrust, min(maxThrust, thrustCmd));
            
            % Rotation Matrix (Z-Y-X Tait-Bryan / Rz*Ry*Rx)
            phi = obj.euler(1);
            theta = obj.euler(2);
            psi = obj.euler(3);
            
            Rz = [cos(psi) -sin(psi) 0; sin(psi) cos(psi) 0; 0 0 1];
            Ry = [cos(theta) 0 sin(theta); 0 1 0; -sin(theta) 0 cos(theta)];
            Rx = [1 0 0; 0 cos(phi) -sin(phi); 0 sin(phi) cos(phi)];
            R = Rz * Ry * Rx;
            
            % Translational accelerations in world frame
            thrustVectorBody = [0; 0; obj.thrust];
            thrustVectorWorld = R * thrustVectorBody;
            gravityVectorWorld = [0; 0; -m * g];
            
            % Aerodynamic drag / disturbance
            dragCoeff = 0.25;
            airVel = obj.vel - windVector;
            dragForce = -dragCoeff * airVel;
            
            obj.accel = (thrustVectorWorld + gravityVectorWorld + dragForce) / m;
            
            % Integrate velocity and position
            obj.vel = obj.vel + obj.accel * dt;
            
            % Speed clamping
            horizSpeed = norm(obj.vel(1:2));
            if horizSpeed > obj.cfg.drone.maxSpeedHoriz
                obj.vel(1:2) = obj.vel(1:2) * (obj.cfg.drone.maxSpeedHoriz / horizSpeed);
            end
            obj.vel(3) = max(-obj.cfg.drone.maxSpeedVert, min(obj.cfg.drone.maxSpeedVert, obj.vel(3)));
            
            obj.pos = obj.pos + obj.vel * dt;
            
            % Ground collision barrier
            if obj.pos(3) < 0
                obj.pos(3) = 0;
                obj.vel(3) = max(0, obj.vel(3));
            end
            
            % Spin propellers (alternate CW and CCW)
            spinRate = obj.propSpeed * (obj.thrust / (m * g));
            obj.propAngles(1) = obj.propAngles(1) + spinRate * dt;
            obj.propAngles(2) = obj.propAngles(2) - spinRate * dt;
            obj.propAngles(3) = obj.propAngles(3) + spinRate * dt;
            obj.propAngles(4) = obj.propAngles(4) - spinRate * dt;
            obj.propAngles = wrapTo2Pi(obj.propAngles);
        end
        
        function R = getRotationMatrix(obj)
            % Computes current 3x3 direction cosine matrix
            phi = obj.euler(1);
            theta = obj.euler(2);
            psi = obj.euler(3);
            
            Rz = [cos(psi) -sin(psi) 0; sin(psi) cos(psi) 0; 0 0 1];
            Ry = [cos(theta) 0 sin(theta); 0 1 0; -sin(theta) 0 cos(theta)];
            Rx = [1 0 0; 0 cos(phi) -sin(phi); 0 sin(phi) cos(phi)];
            R = Rz * Ry * Rx;
        end
    end
end
