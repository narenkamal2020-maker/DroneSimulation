classdef OccupancyMap3D < handle
    % OCCUPANCYMAP3D 3D Voxel Grid Environment Model with Obstacle Inflation.
    
    properties
        cfg
        res             % Grid cell resolution (m)
        boundsX         % [xMin, xMax]
        boundsY         % [yMin, yMax]
        boundsZ         % [zMin, zMax]
        
        nx, ny, nz      % Grid dimension counts
        grid            % 3D uint8 array: 0 = FREE, 1 = OCCUPIED
        inflationRadius % (m)
    end
    
    methods
        function obj = OccupancyMap3D(cfg)
            obj.cfg = cfg;
            obj.res = cfg.map.resolution;
            obj.boundsX = cfg.map.boundsX;
            obj.boundsY = cfg.map.boundsY;
            obj.boundsZ = cfg.map.boundsZ;
            obj.inflationRadius = cfg.drone.inflationRadius;
            
            obj.nx = round((obj.boundsX(2) - obj.boundsX(1)) / obj.res) + 1;
            obj.ny = round((obj.boundsY(2) - obj.boundsY(1)) / obj.res) + 1;
            obj.nz = round((obj.boundsZ(2) - obj.boundsZ(1)) / obj.res) + 1;
            
            obj.grid = zeros(obj.nx, obj.ny, obj.nz, 'uint8');
        end
        
        function reset(obj)
            obj.grid(:) = 0;
        end
        
        function buildFromObstacles(obj, obstacleBounds)
            % Populates 3D voxel grid with static/dynamic obstacle bounds
            obj.reset();
            
            infRad = obj.inflationRadius;
            numObs = length(obstacleBounds);
            
            for k = 1:numObs
                obs = obstacleBounds(k);
                
                if strcmp(obs.type, 'box')
                    % Inflated bounding box
                    b = obs.bounds;
                    xMin = max(obj.boundsX(1), b(1) - infRad);
                    xMax = min(obj.boundsX(2), b(2) + infRad);
                    yMin = max(obj.boundsY(1), b(3) - infRad);
                    yMax = min(obj.boundsY(2), b(4) + infRad);
                    zMin = max(obj.boundsZ(1), b(5)); % Keep ground level
                    zMax = min(obj.boundsZ(2), b(6) + infRad);
                    
                    [iMin, jMin, kMin] = obj.posToGrid([xMin, yMin, zMin]);
                    [iMax, jMax, kMax] = obj.posToGrid([xMax, yMax, zMax]);
                    
                    obj.grid(iMin:iMax, jMin:jMax, kMin:kMax) = 1;
                    
                elseif strcmp(obs.type, 'cylinder')
                    % Inflated cylinder
                    cx = obs.center(1);
                    cy = obs.center(2);
                    rInf = obs.radius + infRad;
                    zMin = max(obj.boundsZ(1), obs.bounds(5));
                    zMax = min(obj.boundsZ(2), obs.bounds(5) + obs.height + infRad);
                    
                    [iMin, jMin, kMin] = obj.posToGrid([cx - rInf, cy - rInf, zMin]);
                    [iMax, jMax, kMax] = obj.posToGrid([cx + rInf, cy + rInf, zMax]);
                    
                    for i = iMin:iMax
                        for j = jMin:jMax
                            p = obj.gridToPos([i, j, kMin]);
                            distXY = sqrt((p(1) - cx)^2 + (p(2) - cy)^2);
                            if distXY <= rInf
                                obj.grid(i, j, kMin:kMax) = 1;
                            end
                        end
                    end
                end
            end
            
            % Enforce flight ceiling and floor buffer
            [~, ~, kFloor] = obj.posToGrid([0, 0, obj.cfg.map.safeFlightFloor]);
            [~, ~, kCeil] = obj.posToGrid([0, 0, obj.cfg.map.safeFlightCeiling]);
            obj.grid(:, :, 1:max(1, kFloor-1)) = 1;
            if kCeil < obj.nz
                obj.grid(:, :, kCeil+1:end) = 1;
            end
        end
        
        function [ix, iy, iz] = posToGrid(obj, pos)
            % Maps continuous world 3D position to grid indices [1..nx, 1..ny, 1..nz]
            ix = round((pos(1) - obj.boundsX(1)) / obj.res) + 1;
            iy = round((pos(2) - obj.boundsY(1)) / obj.res) + 1;
            iz = round((pos(3) - obj.boundsZ(1)) / obj.res) + 1;
            
            ix = max(1, min(obj.nx, ix));
            iy = max(1, min(obj.ny, iy));
            iz = max(1, min(obj.nz, iz));
        end
        
        function pos = gridToPos(obj, gridIdx)
            % Maps grid indices to continuous world 3D coordinates (cell center)
            x = obj.boundsX(1) + (gridIdx(1) - 1) * obj.res;
            y = obj.boundsY(1) + (gridIdx(2) - 1) * obj.res;
            z = obj.boundsZ(1) + (gridIdx(3) - 1) * obj.res;
            pos = [x, y, z];
        end
        
        function occupied = isOccupied(obj, pos)
            % Fast continuous coordinate collision test
            [ix, iy, iz] = obj.posToGrid(pos);
            occupied = (obj.grid(ix, iy, iz) == 1);
        end
        
        function blocked = isGridBlocked(obj, ix, iy, iz)
            % Direct index check with boundary safety
            if ix < 1 || ix > obj.nx || iy < 1 || iy > obj.ny || iz < 1 || iz > obj.nz
                blocked = true;
            else
                blocked = (obj.grid(ix, iy, iz) == 1);
            end
        end
        
        function clear = checkLineOfSight(obj, p1, p2, stepDist)
            % 3D Ray-marching line-of-sight test between two points
            if nargin < 4
                stepDist = obj.res * 0.5;
            end
            
            dist = norm(p2 - p1);
            if dist < 1e-4
                clear = ~obj.isOccupied(p1);
                return;
            end
            
            nSteps = ceil(dist / stepDist);
            clear = true;
            
            for s = 0:nSteps
                alpha = s / nSteps;
                p = p1 + alpha * (p2 - p1);
                if obj.isOccupied(p)
                    clear = false;
                    return;
                end
            end
        end
    end
end
