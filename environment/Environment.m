classdef Environment < handle
    % ENVIRONMENT Manages the 3D simulation world, obstacles, and dynamic hazards.
    
    properties
        cfg
        ax
        startPos
        goalPos
        staticObstacles
        dynamicObstacle
        hasDynamicObstacle = false
        hasEmergencyObstacle = false
        emergencyTriggered = false
        emergencyObstacleDef
        
        % Graphic handles
        groundHandles
        staticHandles
        dynamicPatchHandles = []
        emergencyPatchHandles = []
        
        % Analytical bounds cache for collision & raycasting
        staticBounds
        dynamicBounds
    end
    
    methods
        function obj = Environment(cfg, scenarioData)
            % Constructor
            obj.cfg = cfg;
            obj.startPos = scenarioData.startPos;
            obj.goalPos = scenarioData.goalPos;
            obj.staticObstacles = scenarioData.obstacles;
            
            if isfield(scenarioData, 'dynamicObstacle') && ~isempty(scenarioData.dynamicObstacle)
                obj.dynamicObstacle = scenarioData.dynamicObstacle;
                obj.hasDynamicObstacle = true;
            end
            
            if isfield(scenarioData, 'emergencyObstacle') && ~isempty(scenarioData.emergencyObstacle)
                obj.emergencyObstacleDef = scenarioData.emergencyObstacle;
                obj.hasEmergencyObstacle = true;
            end
        end
        
        function render(obj, ax)
            % Renders static scene elements to axes
            obj.ax = ax;
            obj.groundHandles = createGround(ax, obj.cfg, obj.startPos, obj.goalPos);
            [obj.staticHandles, obj.staticBounds] = createObstacleGeometries(ax, obj.cfg, obj.staticObstacles);
            
            if obj.hasDynamicObstacle
                obj.renderDynamicObstacle();
            end
        end
        
        function renderDynamicObstacle(obj)
            % Initializes graphics for moving obstacle
            if ~obj.hasDynamicObstacle || isempty(obj.ax) || ~isvalid(obj.ax)
                return;
            end
            
            p = obj.dynamicObstacle.pos;
            sz = obj.dynamicObstacle.size; % [dx, dy, dz]
            col = obj.cfg.theme.dynamicObs;
            
            % Generate 3D box vertices
            [v, f] = obj.getBoxMesh(p(1)-sz(1)/2, p(2)-sz(2)/2, p(3)-sz(3)/2, sz(1), sz(2), sz(3));
            
            hold(obj.ax, 'on');
            h = patch(obj.ax, 'Vertices', v, 'Faces', f, ...
                      'FaceColor', col, 'EdgeColor', [1.0 0.8 0.2], ...
                      'LineWidth', 1.8, 'FaceAlpha', 0.95);
            
            % Warning strobe text
            hTxt = text(obj.ax, p(1), p(2), p(3) + sz(3)/2 + 1.2, 'HAZARD - DYNAMIC', ...
                        'Color', [1.0, 0.4, 0.2], 'FontSize', 8, 'FontWeight', 'bold', ...
                        'HorizontalAlignment', 'center');
            
            obj.dynamicPatchHandles = [h; hTxt];
            obj.updateDynamicBounds();
        end
        
        function update(obj, simTime, dronePos)
            % Updates moving obstacles and triggers emergency pop-up if applicable
            
            % 1. Dynamic moving obstacle trajectory update
            if obj.hasDynamicObstacle
                dyn = obj.dynamicObstacle;
                
                % Parametric motion along waypoint path or oscillation
                if isfield(dyn, 'trajectoryFunc') && ~isempty(dyn.trajectoryFunc)
                    newPos = dyn.trajectoryFunc(simTime);
                else
                    % Default linear motion between waypoints
                    tNorm = mod(simTime / dyn.period, 2.0);
                    if tNorm > 1.0
                        ratio = 2.0 - tNorm;
                    else
                        ratio = tNorm;
                    end
                    newPos = dyn.startPos + ratio * (dyn.endPos - dyn.startPos);
                end
                
                obj.dynamicObstacle.pos = newPos;
                
                % Update dynamic mesh graphics
                if ~isempty(obj.dynamicPatchHandles) && all(isvalid(obj.dynamicPatchHandles))
                    sz = dyn.size;
                    [v, ~] = obj.getBoxMesh(newPos(1)-sz(1)/2, newPos(2)-sz(2)/2, newPos(3)-sz(3)/2, sz(1), sz(2), sz(3));
                    set(obj.dynamicPatchHandles(1), 'Vertices', v);
                    set(obj.dynamicPatchHandles(2), 'Position', [newPos(1), newPos(2), newPos(3) + sz(3)/2 + 1.2]);
                end
                
                obj.updateDynamicBounds();
            end
            
            % 2. Emergency pop-up obstacle check
            if obj.hasEmergencyObstacle && ~obj.emergencyTriggered
                eDef = obj.emergencyObstacleDef;
                distToTrigger = norm(dronePos(1:2) - eDef.triggerLocation(1:2));
                
                if distToTrigger < eDef.triggerDistance || simTime >= eDef.triggerTime
                    obj.triggerEmergencyObstacle();
                end
            end
        end
        
        function triggerEmergencyObstacle(obj)
            % Instantiates emergency pop-up barrier
            obj.emergencyTriggered = true;
            eDef = obj.emergencyObstacleDef;
            
            % Add to static obstacles list
            newObs = struct('type', 'building', ...
                            'params', [eDef.pos(1)-eDef.size(1)/2, eDef.pos(2)-eDef.size(2)/2, 0, eDef.size(1), eDef.size(2), eDef.size(3)], ...
                            'color', [0.95, 0.15, 0.15]);
            
            obj.staticObstacles = [obj.staticObstacles; newObs];
            
            % Render to axes
            if ~isempty(obj.ax) && isvalid(obj.ax)
                [h, b] = createObstacleGeometries(obj.ax, obj.cfg, newObs);
                obj.emergencyPatchHandles = h;
                obj.staticBounds = [obj.staticBounds, b];
                
                % Warning beacon
                text(obj.ax, eDef.pos(1), eDef.pos(2), eDef.size(3) + 1.5, ...
                     '⚠ SUDDEN INTRUSION', 'Color', [1 0.2 0.1], 'FontSize', 9, ...
                     'FontWeight', 'bold', 'HorizontalAlignment', 'center');
            end
        end
        
        function updateDynamicBounds(obj)
            % Updates analytical bounding box for dynamic obstacle
            p = obj.dynamicObstacle.pos;
            sz = obj.dynamicObstacle.size;
            b.type = 'box';
            b.bounds = [p(1)-sz(1)/2, p(1)+sz(1)/2, ...
                        p(2)-sz(2)/2, p(2)+sz(2)/2, ...
                        p(3)-sz(3)/2, p(3)+sz(3)/2];
            b.center = p;
            b.radius = sqrt((sz(1)/2)^2 + (sz(2)/2)^2);
            b.height = sz(3);
            b.color = obj.cfg.theme.dynamicObs;
            obj.dynamicBounds = b;
        end
        
        function allBounds = getAllObstacleBounds(obj)
            % Returns combined bounds of all static and dynamic obstacles
            allBounds = obj.staticBounds;
            if obj.hasDynamicObstacle && ~isempty(obj.dynamicBounds)
                allBounds = [allBounds, obj.dynamicBounds];
            end
        end
        
        function [v, f] = getBoxMesh(~, x0, y0, z0, dx, dy, dz)
            % Generates vertex and face matrices for a cuboid
            v = [
                x0,      y0,      z0;
                x0 + dx, y0,      z0;
                x0 + dx, y0 + dy, z0;
                x0,      y0 + dy, z0;
                x0,      y0,      z0 + dz;
                x0 + dx, y0,      z0 + dz;
                x0 + dx, y0 + dy, z0 + dz;
                x0,      y0 + dy, z0 + dz
            ];
            f = [
                1 2 6 5;
                2 3 7 6;
                3 4 8 7;
                4 1 5 8;
                5 6 7 8;
                1 4 3 2
            ];
        end
    end
end
