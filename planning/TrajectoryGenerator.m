classdef TrajectoryGenerator < handle
    % TRAJECTORYGENERATOR Continuous C2 Smooth 3D Trajectory Generation.
    %
    % Converts discrete waypoints into time-parameterized smooth polynomial
    % splines with continuous position, velocity, acceleration, and yaw.
    % Works in 100% pure base MATLAB without external toolbox dependencies.
    
    properties
        cfg
        waypoints       % [M x 3] Spatial waypoints
        timeNodes       % [1 x M] Arrival times at waypoints
        totalDuration   % Total trajectory flight time (s)
        
        % Piecewise polynomial structs for X, Y, Z
        ppX, ppY, ppZ
        ppVx, ppVy, ppVz
        ppAx, ppAy, ppAz
        
        lastYaw = 0
    end
    
    methods
        function obj = TrajectoryGenerator(cfg)
            obj.cfg = cfg;
        end
        
        function success = generate(obj, waypoints, nominalSpeed, initialVel)
            % Fits clamped smooth cubic splines through 3D waypoints
            %
            % Inputs:
            %   waypoints    - [M x 3] Waypoint coordinates
            %   nominalSpeed - Average cruise speed (m/s) [default = 3.5]
            %   initialVel   - [1 x 3] Optional starting velocity vector
            
            if nargin < 3 || isempty(nominalSpeed)
                nominalSpeed = 3.5;
            end
            if nargin < 4 || isempty(initialVel)
                initialVel = [0, 0, 0];
            end
            
            M = size(waypoints, 1);
            if M < 2
                success = false;
                return;
            end
            
            % Remove duplicate adjacent waypoints
            dists = sqrt(sum(diff(waypoints, 1, 1).^2, 2));
            keepMask = [true; dists > 0.05];
            waypoints = waypoints(keepMask, :);
            M = size(waypoints, 1);
            
            if M < 2
                waypoints = [waypoints; waypoints + [0.1, 0, 0]];
                M = 2;
            end
            
            obj.waypoints = waypoints;
            
            % Allocate segment times based on Euclidean distance
            segmentDists = sqrt(sum(diff(waypoints, 1, 1).^2, 2));
            segmentTimes = max(1.2, segmentDists / nominalSpeed);
            
            obj.timeNodes = [0, cumsum(segmentTimes)'];
            obj.totalDuration = obj.timeNodes(end);
            
            % Generate clamped cubic splines with specified endpoint velocities
            t = obj.timeNodes;
            x = waypoints(:, 1)';
            y = waypoints(:, 2)';
            z = waypoints(:, 3)';
            
            % Clamped spline syntax: spline(t, [v_start, pos_vals, v_end])
            obj.ppX = spline(t, [initialVel(1), x, 0]);
            obj.ppY = spline(t, [initialVel(2), y, 0]);
            obj.ppZ = spline(t, [initialVel(3), z, 0]);
            
            % Analytically differentiate piecewise polynomials
            obj.ppVx = obj.differentiatePP(obj.ppX);
            obj.ppVy = obj.differentiatePP(obj.ppY);
            obj.ppVz = obj.differentiatePP(obj.ppZ);
            
            obj.ppAx = obj.differentiatePP(obj.ppVx);
            obj.ppAy = obj.differentiatePP(obj.ppVy);
            obj.ppAz = obj.differentiatePP(obj.ppVz);
            
            % Initialize heading to forward path direction
            if M >= 2
                dx0 = waypoints(2, 1) - waypoints(1, 1);
                dy0 = waypoints(2, 2) - waypoints(1, 2);
                if norm([dx0, dy0]) > 0.1
                    obj.lastYaw = atan2(dy0, dx0);
                end
            end
            
            success = true;
        end
        
        function [pos_d, vel_d, accel_d, yaw_d] = evaluate(obj, t)
            % Evaluates trajectory state at time t
            %
            % Outputs:
            %   pos_d   - [3 x 1] Desired position [x; y; z]
            %   vel_d   - [3 x 1] Desired velocity [vx; vy; vz]
            %   accel_d - [3 x 1] Desired acceleration [ax; ay; az]
            %   yaw_d   - Desired yaw heading (rad)
            
            if isempty(obj.ppX)
                pos_d = [0; 0; 0];
                vel_d = [0; 0; 0];
                accel_d = [0; 0; 0];
                yaw_d = 0;
                return;
            end
            
            tEval = max(0, min(t, obj.totalDuration));
            
            % Position
            pos_d = [ppval(obj.ppX, tEval); ...
                     ppval(obj.ppY, tEval); ...
                     ppval(obj.ppZ, tEval)];
                 
            % Velocity
            if t >= obj.totalDuration
                vel_d = [0; 0; 0];
                accel_d = [0; 0; 0];
            else
                vel_d = [ppval(obj.ppVx, tEval); ...
                         ppval(obj.ppVy, tEval); ...
                         ppval(obj.ppVz, tEval)];
                     
                accel_d = [ppval(obj.ppAx, tEval); ...
                           ppval(obj.ppAy, tEval); ...
                           ppval(obj.ppAz, tEval)];
            end
            
            % Natural heading alignment with velocity vector
            horizSpeed = norm(vel_d(1:2));
            if horizSpeed > 0.3
                targetYaw = atan2(vel_d(2), vel_d(1));
                obj.lastYaw = targetYaw;
            end
            yaw_d = obj.lastYaw;
        end
        
        function points = samplePoints(obj, numSamples)
            % Samples the continuous trajectory for smooth visualization
            if nargin < 2
                numSamples = 150;
            end
            
            if isempty(obj.ppX)
                points = obj.waypoints;
                return;
            end
            
            tVec = linspace(0, obj.totalDuration, numSamples);
            x = ppval(obj.ppX, tVec);
            y = ppval(obj.ppY, tVec);
            z = ppval(obj.ppZ, tVec);
            points = [x(:), y(:), z(:)];
        end
    end
    
    methods (Static)
        function dpp = differentiatePP(pp)
            % Base MATLAB analytical derivative of a piecewise polynomial struct
            % For cubic: a*t^3 + b*t^2 + c*t + d -> 3*a*t^2 + 2*b*t + c
            dpp = pp;
            order = pp.order;
            coefs = pp.coefs;
            numPieces = pp.pieces;
            dim = pp.dim;
            
            if order == 1
                % Derivative of constant is zero
                dpp.order = 1;
                dpp.coefs = zeros(numPieces * dim, 1);
            else
                dpp.order = order - 1;
                dCoefs = zeros(numPieces * dim, order - 1);
                for i = 1:(order - 1)
                    power = order - i;
                    dCoefs(:, i) = coefs(:, i) * power;
                end
                dpp.coefs = dCoefs;
            end
        end
    end
end
