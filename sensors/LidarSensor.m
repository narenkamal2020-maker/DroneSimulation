classdef LidarSensor < handle
    % LIDARSENSOR Simulates a 3D LiDAR scanner with raycasting, noise, and detection.
    
    properties
        cfg
        maxRange
        numRaysHoriz
        elevations
        noiseSigma
        
        % Ray directions in body frame (unit vectors)
        rayDirsBody
        numRays
        
        % Graphic handles for visualization
        hRays = []
        hHits = []
    end
    
    methods
        function obj = LidarSensor(cfg)
            obj.cfg = cfg;
            obj.maxRange = cfg.lidar.maxRange;
            obj.numRaysHoriz = cfg.lidar.numRaysHoriz;
            obj.elevations = cfg.lidar.elevations;
            obj.noiseSigma = cfg.lidar.noiseSigma;
            
            % Precompute ray directions in body frame
            azimuths = linspace(0, 2*pi - 2*pi/obj.numRaysHoriz, obj.numRaysHoriz);
            elevRads = deg2rad(obj.elevations);
            
            [AZ, EL] = meshgrid(azimuths, elevRads);
            AZ = AZ(:);
            EL = EL(:);
            obj.numRays = length(AZ);
            
            % Unit direction vectors in body frame
            dx = cos(EL) .* cos(AZ);
            dy = cos(EL) .* sin(AZ);
            dz = sin(EL);
            obj.rayDirsBody = [dx, dy, dz];
        end
        
        function [ranges, hitPoints, rayEnds, minDistance, hitObstacleIndices] = scan(obj, dronePos, R_world_body, obstacleBounds)
            % Executes a 3D LiDAR scan against world obstacles
            %
            % Inputs:
            %   dronePos       - [x; y; z] Drone origin
            %   R_world_body   - 3x3 Rotation matrix
            %   obstacleBounds - Struct array of obstacle bounding geometries
            %
            % Outputs:
            %   ranges         - Array of measured ranges (m)
            %   hitPoints      - [K x 3] 3D points where rays intersected obstacles
            %   rayEnds        - [numRays x 3] Endpoints of rays for visualization
            %   minDistance    - Minimum measured distance to any obstacle
            %   hitObstacleIndices - List of obstacles detected in scan
            
            dronePos = reshape(dronePos(1:3), [1, 3]);
            
            % Transform ray directions to world frame: D_world = D_body * R^T
            rayDirsWorld = obj.rayDirsBody * (R_world_body');
            
            ranges = obj.maxRange * ones(obj.numRays, 1);
            hitMask = false(obj.numRays, 1);
            hitObstacleIndices = zeros(obj.numRays, 1);
            hitCount = 0;
            
            numObs = length(obstacleBounds);
            if numObs == 0
                rayEnds = dronePos + rayDirsWorld * obj.maxRange;
                hitPoints = zeros(0, 3);
                minDistance = obj.maxRange;
                hitObstacleIndices = [];
                return;
            end
            
            % Test each ray against all obstacles
            for i = 1:obj.numRays
                d = rayDirsWorld(i, :);
                closestT = obj.maxRange;
                hitIdx = 0;
                
                for k = 1:numObs
                    obs = obstacleBounds(k);
                    
                    if strcmp(obs.type, 'box')
                        % Ray-AABB intersection (Slab method)
                        b = obs.bounds; % [xMin, xMax, yMin, yMax, zMin, zMax]
                        
                        % Avoid division by zero
                        invD = 1 ./ (d + 1e-12 * (d == 0));
                        t1 = (b([1, 3, 5]) - dronePos) .* invD;
                        t2 = (b([2, 4, 6]) - dronePos) .* invD;
                        
                        tMin = min(t1, t2);
                        tMax = max(t1, t2);
                        
                        tNear = max(tMin);
                        tFar = min(tMax);
                        
                        if tFar >= tNear && tFar >= 0 && tNear < closestT
                            tHit = max(0, tNear);
                            if tHit < closestT
                                closestT = tHit;
                                hitIdx = k;
                            end
                        end
                        
                    elseif strcmp(obs.type, 'cylinder')
                        % Ray-Cylinder intersection
                        % Cylinder center: obs.center(1:2), radius: obs.radius, z: [cz, cz+h]
                        cx = obs.center(1); cy = obs.center(2);
                        r = obs.radius;
                        cz = obs.bounds(5); h = obs.height;
                        
                        ox = dronePos(1) - cx;
                        oy = dronePos(2) - cy;
                        
                        a = d(1)^2 + d(2)^2;
                        if a > 1e-8
                            b_coeff = 2 * (ox * d(1) + oy * d(2));
                            c_coeff = ox^2 + oy^2 - r^2;
                            disc = b_coeff^2 - 4 * a * c_coeff;
                            
                            if disc >= 0
                                tCyl1 = (-b_coeff - sqrt(disc)) / (2 * a);
                                tCyl2 = (-b_coeff + sqrt(disc)) / (2 * a);
                                
                                for tCand = [tCyl1, tCyl2]
                                    if tCand >= 0 && tCand < closestT
                                        zHit = dronePos(3) + tCand * d(3);
                                        if zHit >= cz && zHit <= (cz + h)
                                            closestT = tCand;
                                            hitIdx = k;
                                            break;
                                        end
                                    end
                                end
                            end
                        end
                    end
                end
                
                % Ground plane intersection (z = 0)
                if d(3) < -1e-4
                    tGround = -dronePos(3) / d(3);
                    if tGround >= 0 && tGround < closestT
                        closestT = tGround;
                    end
                end
                
                if closestT < obj.maxRange
                    % Add sensor noise
                    noisyRange = closestT + randn() * obj.noiseSigma;
                    ranges(i) = max(0.1, noisyRange);
                    hitMask(i) = true;
                    if hitIdx > 0
                        hitCount = hitCount + 1;
                        hitObstacleIndices(hitCount) = hitIdx;
                    end
                end
            end
            
            hitObstacleIndices = unique(hitObstacleIndices(1:hitCount));
            rayEnds = dronePos + rayDirsWorld .* ranges;
            hitPoints = rayEnds(hitMask, :);
            minDistance = min(ranges);
        end
        
        function renderInit(obj, ax)
            % Initializes graphic handles for rays and hit points
            hold(ax, 'on');
            % Rays: thin transparent lines
            obj.hRays = plot3(ax, nan, nan, nan, 'Color', [obj.cfg.theme.lidarRays, 0.25], ...
                              'LineWidth', 0.6);
            % Hit points: glowing markers
            obj.hHits = scatter3(ax, nan, nan, nan, 24, obj.cfg.theme.lidarHits, ...
                                 'filled', 'MarkerEdgeColor', [1 0.9 0.5]);
        end
        
        function updateVisuals(obj, dronePos, rayEnds, hitPoints)
            % Updates LiDAR beam graphics efficiently
            if isempty(obj.hRays) || ~isvalid(obj.hRays)
                return;
            end
            
            % Subsample rays for clean visual aesthetics (e.g. 16 rays)
            subIdx = round(linspace(1, obj.numRays, min(24, obj.numRays)));
            
            % Build line segments: DronePos -> RayEnd -> NaN
            N = length(subIdx);
            xLines = zeros(3 * N, 1);
            yLines = zeros(3 * N, 1);
            zLines = zeros(3 * N, 1);
            
            for k = 1:N
                idx = subIdx(k);
                base = (k - 1) * 3;
                xLines(base+1) = dronePos(1);
                xLines(base+2) = rayEnds(idx, 1);
                xLines(base+3) = NaN;
                
                yLines(base+1) = dronePos(2);
                yLines(base+2) = rayEnds(idx, 2);
                yLines(base+3) = NaN;
                
                zLines(base+1) = dronePos(3);
                zLines(base+2) = rayEnds(idx, 3);
                zLines(base+3) = NaN;
            end
            
            set(obj.hRays, 'XData', xLines, 'YData', yLines, 'ZData', zLines);
            
            % Update hit points
            if ~isempty(hitPoints)
                set(obj.hHits, 'XData', hitPoints(:, 1), ...
                               'YData', hitPoints(:, 2), ...
                               'ZData', hitPoints(:, 3));
            else
                set(obj.hHits, 'XData', nan, 'YData', nan, 'ZData', nan);
            end
        end
    end
end
