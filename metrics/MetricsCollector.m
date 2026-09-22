classdef MetricsCollector < handle
    % METRICSCOLLECTOR Records simulation telemetry and calculates mission KPIs.
    
    properties
        cfg
        
        % Time series data buffers
        time = []
        posActual = []      % [N x 3]
        posDesired = []     % [N x 3]
        velActual = []      % [N x 3]
        velDesired = []     % [N x 3]
        eulerActual = []    % [N x 3] (deg)
        eulerCmd = []       % [N x 3] (deg)
        thrust = []         % [N x 1]
        clearance = []      % [N x 1]
        trackingError = []  % [N x 1]
        states = {}         % {N x 1}
        
        % Event counters
        numObstacleDetections = 0
        numRouteReplans = 0
        numCollisions = 0
        
        plannedPathLength = 0
    end
    
    methods
        function obj = MetricsCollector(cfg)
            obj.cfg = cfg;
        end
        
        function log(obj, t, pAct, pDes, vAct, vDes, eAct, eCmd, T, clr, err, stateStr)
            % Appends one telemetry record
            obj.time(end+1, 1) = t;
            obj.posActual(end+1, :) = reshape(pAct(1:3), [1, 3]);
            obj.posDesired(end+1, :) = reshape(pDes(1:3), [1, 3]);
            obj.velActual(end+1, :) = reshape(vAct(1:3), [1, 3]);
            obj.velDesired(end+1, :) = reshape(vDes(1:3), [1, 3]);
            obj.eulerActual(end+1, :) = rad2deg(reshape(eAct(1:3), [1, 3]));
            obj.eulerCmd(end+1, :) = rad2deg(reshape(eCmd(1:3), [1, 3]));
            obj.thrust(end+1, 1) = T;
            obj.clearance(end+1, 1) = clr;
            obj.trackingError(end+1, 1) = err;
            obj.states{end+1, 1} = stateStr;
        end
        
        function recordObstacleDetection(obj)
            obj.numObstacleDetections = obj.numObstacleDetections + 1;
        end
        
        function recordReplan(obj)
            obj.numRouteReplans = obj.numRouteReplans + 1;
        end
        
        function recordCollision(obj)
            obj.numCollisions = obj.numCollisions + 1;
        end
        
        function summary = getSummary(obj)
            % Computes verified mission performance statistics
            if isempty(obj.time)
                summary = struct();
                return;
            end
            
            summary.totalMissionTime = obj.time(end) - obj.time(1);
            
            % Actual flight distance
            diffs = diff(obj.posActual, 1, 1);
            summary.actualDistance = sum(sqrt(sum(diffs.^2, 2)));
            summary.plannedDistance = obj.plannedPathLength;
            
            % Speed statistics
            speeds = sqrt(sum(obj.velActual.^2, 2));
            summary.avgSpeed = mean(speeds);
            summary.maxSpeed = max(speeds);
            
            % Obstacle clearance
            validClr = obj.clearance(~isinf(obj.clearance));
            if isempty(validClr)
                summary.minClearance = inf;
            else
                summary.minClearance = min(validClr);
            end
            
            % Tracking accuracy
            summary.rmsTrackingError = sqrt(mean(obj.trackingError.^2));
            summary.maxTrackingError = max(obj.trackingError);
            
            % Events
            summary.obstacleDetections = obj.numObstacleDetections;
            summary.routeReplans = obj.numRouteReplans;
            summary.collisions = obj.numCollisions;
        end
    end
end
