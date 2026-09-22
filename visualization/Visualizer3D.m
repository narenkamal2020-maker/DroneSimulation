classdef Visualizer3D < handle
    % VISUALIZER3D Coordinates high-fidelity 3D rendering, lighting, and trajectories.
    
    properties
        cfg
        fig
        ax
        droneGraphics
        cameraCtrl
        
        % Trajectory graphics handles
        hPlannedPath
        hReplannedPath
        hTrail
        
        % Trail history buffer
        trailHistory = zeros(0, 3)
        maxTrailPts = 450
        
        % Lights
        camLightHandle
    end
    
    methods
        function obj = Visualizer3D(cfg, fig, ax)
            obj.cfg = cfg;
            obj.fig = fig;
            obj.ax = ax;
            
            % Setup 3D Viewport aesthetics
            hold(ax, 'on');
            axis(ax, 'equal');
            grid(ax, 'off');
            box(ax, 'on');
            
            set(ax, 'Color', cfg.theme.bgColor, ...
                    'XColor', [0.2 0.3 0.4], ...
                    'YColor', [0.2 0.3 0.4], ...
                    'ZColor', [0.2 0.3 0.4], ...
                    'LineWidth', 1.0);
                
            xlim(ax, cfg.map.boundsX);
            ylim(ax, cfg.map.boundsY);
            zlim(ax, [0, cfg.map.boundsZ(2) + 2]);
            
            xlabel(ax, 'X (meters)', 'Color', [0.7 0.8 0.9], 'FontWeight', 'bold');
            ylabel(ax, 'Y (meters)', 'Color', [0.7 0.8 0.9], 'FontWeight', 'bold');
            zlabel(ax, 'Altitude Z (meters)', 'Color', [0.7 0.8 0.9], 'FontWeight', 'bold');
            
            camproj(ax, 'perspective');
            lighting(ax, 'gouraud');
            obj.camLightHandle = camlight(ax, 'headlight');
            
            % Initialize Trajectory lines
            % Planned initial route (Cyan dashed line)
            obj.hPlannedPath = plot3(ax, nan, nan, nan, '--', ...
                                     'Color', [cfg.theme.pathPlanned, 0.85], ...
                                     'LineWidth', 2.2, 'DisplayName', 'Planned Route');
                                 
            % Replanned detour route (Neon emerald/amber line)
            obj.hReplannedPath = plot3(ax, nan, nan, nan, '-', ...
                                       'Color', [cfg.theme.pathReplanned, 0.95], ...
                                       'LineWidth', 2.8, 'DisplayName', 'Active Safe Detour');
                                   
            % Flight trail (Amber/yellow trail)
            obj.hTrail = plot3(ax, nan, nan, nan, '-', ...
                               'Color', [cfg.theme.droneTrail, 0.75], ...
                               'LineWidth', 1.6, 'DisplayName', 'Flight Path');
                           
            % Initialize 3D quadcopter graphics
            obj.droneGraphics = renderDrone3D(ax, cfg);
            
            % Initialize Camera controller
            obj.cameraCtrl = CameraController(cfg, ax);
        end
        
        function setPlannedPath(obj, pathPoints)
            if ~isempty(pathPoints)
                set(obj.hPlannedPath, 'XData', pathPoints(:, 1), ...
                                      'YData', pathPoints(:, 2), ...
                                      'ZData', pathPoints(:, 3));
            end
        end
        
        function setReplannedPath(obj, pathPoints)
            if ~isempty(pathPoints)
                set(obj.hReplannedPath, 'XData', pathPoints(:, 1), ...
                                        'YData', pathPoints(:, 2), ...
                                        'ZData', pathPoints(:, 3));
            end
        end
        
        function update(obj, dronePos, droneEuler, propAngles)
            % Updates quadcopter pose and animation
            dronePos = reshape(dronePos(1:3), [3, 1]);
            
            % Update drone transformation
            updateDronePose(obj.droneGraphics, dronePos, droneEuler, propAngles);
            
            % Update flight trail
            obj.trailHistory = [obj.trailHistory; dronePos'];
            if size(obj.trailHistory, 1) > obj.maxTrailPts
                obj.trailHistory(1:end-obj.maxTrailPts, :) = [];
            end
            
            set(obj.hTrail, 'XData', obj.trailHistory(:, 1), ...
                            'YData', obj.trailHistory(:, 2), ...
                            'ZData', obj.trailHistory(:, 3));
                        
            % Update Camera
            obj.cameraCtrl.update(dronePos, droneEuler(3));
            
            % Keep headlight oriented with camera
            if ~isempty(obj.camLightHandle) && isvalid(obj.camLightHandle)
                camlight(obj.camLightHandle, 'headlight');
            end
        end
    end
end
