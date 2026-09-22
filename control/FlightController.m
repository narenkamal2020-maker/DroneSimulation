classdef FlightController < handle
    % FLIGHTCONTROLLER Cascaded Position & Attitude PID Flight Control System.
    %
    % Controls 3D position, altitude, velocities, and banking angles.
    % Rejects wind disturbances and eliminates steady-state tracking errors.
    
    properties
        cfg
        
        % Position PID gains [X, Y, Z]
        Kp_pos
        Ki_pos
        Kd_pos
        
        % Error accumulators
        posErrorIntegral = [0; 0; 0]
        maxIntegral = [3.0; 3.0; 4.0]
        
        lastPosError = [0; 0; 0]
    end
    
    methods
        function obj = FlightController(cfg)
            obj.cfg = cfg;
            obj.Kp_pos = reshape(cfg.ctrl.kp_pos, [3, 1]);
            obj.Ki_pos = reshape(cfg.ctrl.ki_pos, [3, 1]);
            obj.Kd_pos = reshape(cfg.ctrl.kd_pos, [3, 1]);
        end
        
        function reset(obj)
            obj.posErrorIntegral = [0; 0; 0];
            obj.lastPosError = [0; 0; 0];
        end
        
        function [thrustCmd, eulerCmd, trackingError] = computeControl(obj, dt, currPos, currVel, currEuler, ...
                                                                        desPos, desVel, desAccel, desYaw)
            % Computes commanded thrust and Euler angles for quadcopter
            %
            % Inputs:
            %   dt        - Timestep (s)
            %   currPos   - [3 x 1] Current drone position [x; y; z]
            %   currVel   - [3 x 1] Current drone velocity [vx; vy; vz]
            %   currEuler - [3 x 1] Current drone attitude [phi; theta; psi]
            %   desPos    - [3 x 1] Target position
            %   desVel    - [3 x 1] Target velocity
            %   desAccel  - [3 x 1] Feedforward acceleration
            %   desYaw    - Target yaw angle (rad)
            
            currPos = reshape(currPos(1:3), [3, 1]);
            currVel = reshape(currVel(1:3), [3, 1]);
            currEuler = reshape(currEuler(1:3), [3, 1]);
            desPos = reshape(desPos(1:3), [3, 1]);
            desVel = reshape(desVel(1:3), [3, 1]);
            desAccel = reshape(desAccel(1:3), [3, 1]);
            
            % Tracking errors
            e_pos = desPos - currPos;
            e_vel = desVel - currVel;
            
            % Integral accumulation with anti-windup clamping
            obj.posErrorIntegral = obj.posErrorIntegral + e_pos * dt;
            obj.posErrorIntegral = max(-obj.maxIntegral, min(obj.maxIntegral, obj.posErrorIntegral));
            
            % Commanded world-frame accelerations (PID + Feedforward)
            a_cmd = desAccel + obj.Kp_pos .* e_pos + ...
                    obj.Ki_pos .* obj.posErrorIntegral + ...
                    obj.Kd_pos .* e_vel;
                
            % Clamp maximum horizontal acceleration
            aHoriz = norm(a_cmd(1:2));
            if aHoriz > obj.cfg.drone.maxAccelHoriz
                a_cmd(1:2) = a_cmd(1:2) * (obj.cfg.drone.maxAccelHoriz / aHoriz);
            end
            
            % Clamp vertical acceleration
            a_cmd(3) = max(-obj.cfg.drone.maxAccelVert, min(obj.cfg.drone.maxAccelVert, a_cmd(3)));
            
            % Dynamic bank-to-turn attitude synthesis
            m = obj.cfg.drone.mass;
            g = obj.cfg.drone.g;
            psi = currEuler(3);
            
            % Transform commanded horizontal acceleration into heading-aligned frame
            R_yaw = [cos(psi),  sin(psi); ...
                    -sin(psi),  cos(psi)];
            a_body_xy = R_yaw * a_cmd(1:2);
            a_fwd = a_body_xy(1);
            a_lat = a_body_xy(2);
            
            % Desired pitch and roll from aerodynamic equilibrium
            denomZ = max(0.5, g + a_cmd(3));
            theta_des = atan2(a_fwd, denomZ);
            phi_des   = atan2(-a_lat, denomZ);
            
            % Clamp bank angles
            maxTilt = obj.cfg.drone.maxTiltAngle;
            phi_des   = max(-maxTilt, min(maxTilt, phi_des));
            theta_des = max(-maxTilt, min(maxTilt, theta_des));
            
            % Commanded thrust
            thrustCmd = m * denomZ / (cos(phi_des) * cos(theta_des));
            
            eulerCmd = [phi_des; theta_des; desYaw];
            trackingError = norm(e_pos);
            
            obj.lastPosError = e_pos;
        end
    end
end
