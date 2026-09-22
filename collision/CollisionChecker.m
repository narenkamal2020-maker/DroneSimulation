classdef CollisionChecker < handle
    % COLLISIONCHECKER Exact 3D continuous obstacle clearance and collision detection.
    
    properties
        cfg
        droneRadius
        droneHeight
    end
    
    methods
        function obj = CollisionChecker(cfg)
            obj.cfg = cfg;
            obj.droneRadius = cfg.drone.bodyRadius;
            obj.droneHeight = cfg.drone.height;
        end
        
        function [inCollision, minClearance, closestObsIdx] = check(obj, dronePos, obstacleBounds)
            % Evaluates minimum distance to all 3D obstacles
            %
            % Inputs:
            %   dronePos       - [x; y; z] Drone position
            %   obstacleBounds - Struct array of obstacle bounds
            %
            % Outputs:
            %   inCollision    - Boolean flag (true if clearance <= 0)
            %   minClearance   - Instantaneous minimum clearance distance (m)
            %   closestObsIdx  - Index of closest obstacle
            
            dronePos = reshape(dronePos(1:3), [1, 3]);
            minClearance = inf;
            closestObsIdx = 0;
            inCollision = false;
            
            numObs = length(obstacleBounds);
            for k = 1:numObs
                obs = obstacleBounds(k);
                
                if strcmp(obs.type, 'box')
                    b = obs.bounds; % [xMin, xMax, yMin, yMax, zMin, zMax]
                    % Find closest point on box to drone center
                    cx = max(b(1), min(b(2), dronePos(1)));
                    cy = max(b(3), min(b(4), dronePos(2)));
                    cz = max(b(5), min(b(6), dronePos(3)));
                    
                    distCenter = norm(dronePos - [cx, cy, cz]);
                    distSurface = distCenter - obj.droneRadius;
                    
                    if distSurface < minClearance
                        minClearance = distSurface;
                        closestObsIdx = k;
                    end
                    
                elseif strcmp(obs.type, 'cylinder')
                    % Cylinder: center(1:2), radius, bounds(5:6)
                    cx = obs.center(1); cy = obs.center(2);
                    r = obs.radius;
                    zMin = obs.bounds(5); zMax = obs.bounds(6);
                    
                    % Closest Z
                    cz = max(zMin, min(zMax, dronePos(3)));
                    
                    % Vector from cylinder axis to drone in XY
                    dx = dronePos(1) - cx;
                    dy = dronePos(2) - cy;
                    distXY = norm([dx, dy]);
                    
                    if distXY < 1e-6
                        cXY = [cx, cy];
                    else
                        cXY = [cx, cy] + [dx, dy] * min(r, distXY) / distXY;
                    end
                    
                    distCenter = norm(dronePos - [cXY, cz]);
                    distSurface = distCenter - obj.droneRadius;
                    
                    if distSurface < minClearance
                        minClearance = distSurface;
                        closestObsIdx = k;
                    end
                end
            end
            
            if minClearance <= 0
                inCollision = true;
                minClearance = 0;
            end
        end
    end
end
