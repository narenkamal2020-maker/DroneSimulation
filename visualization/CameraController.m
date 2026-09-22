classdef CameraController < handle
    % CAMERACONTROLLER Multi-mode intelligent 3D camera tracking system.
    %
    % Modes:
    %   1 - Chase / Follow Drone (Smooth third-person chase cam)
    %   2 - Top-Down Tactical (Overhead birds-eye view)
    %   3 - Isometric Arena Overview (Wide cinematic perspective)
    %   4 - Free Camera (Interactive user orbit)
    
    properties
        cfg
        ax
        mode = 1
        
        currCamPos = [0, 0, 0]
        currCamTarget = [0, 0, 0]
        initialized = false
    end
    
    methods
        function obj = CameraController(cfg, ax)
            obj.cfg = cfg;
            obj.ax = ax;
            obj.mode = cfg.cam.defaultMode;
        end
        
        function setMode(obj, newMode)
            obj.mode = newMode;
            if obj.mode == 4
                rotate3d(obj.ax, 'on');
            else
                rotate3d(obj.ax, 'off');
            end
        end
        
        function update(obj, dronePos, droneYaw)
            if isempty(obj.ax) || ~isvalid(obj.ax)
                return;
            end
            
            dronePos = reshape(dronePos(1:3), [1, 3]);
            
            switch obj.mode
                case 1
                    % 1. Smooth Follow Drone Chase Camera
                    dist = obj.cfg.cam.chaseDistance;
                    alt = obj.cfg.cam.chaseAltitude;
                    
                    % Calculate camera position behind drone based on yaw
                    backVec = [-cos(droneYaw), -sin(droneYaw), 0];
                    targetCamPos = dronePos + backVec * dist + [0, 0, alt];
                    targetCamTarget = dronePos + [0, 0, 0.4];
                    
                    if ~obj.initialized
                        obj.currCamPos = targetCamPos;
                        obj.currCamTarget = targetCamTarget;
                        obj.initialized = true;
                    else
                        alpha = obj.cfg.cam.smoothFactor;
                        obj.currCamPos = (1 - alpha) * obj.currCamPos + alpha * targetCamPos;
                        obj.currCamTarget = (1 - alpha) * obj.currCamTarget + alpha * targetCamTarget;
                    end
                    
                    campos(obj.ax, obj.currCamPos);
                    camtarget(obj.ax, obj.currCamTarget);
                    camva(obj.ax, 42); % Field of view angle (deg)
                    camup(obj.ax, [0, 0, 1]);
                    
                case 2
                    % 2. Top-Down Tactical (Overhead)
                    campos(obj.ax, [dronePos(1), dronePos(2), 52]);
                    camtarget(obj.ax, [dronePos(1), dronePos(2), 0]);
                    camva(obj.ax, 45);
                    camup(obj.ax, [0, 1, 0]);
                    
                case 3
                    % 3. Isometric Arena Overview
                    bx = obj.cfg.map.boundsX;
                    by = obj.cfg.map.boundsY;
                    midX = mean(bx);
                    midY = mean(by);
                    
                    campos(obj.ax, [midX + 65, midY - 65, 48]);
                    camtarget(obj.ax, [midX, midY, 8]);
                    camva(obj.ax, 38);
                    camup(obj.ax, [0, 0, 1]);
                    
                case 4
                    % 4. Free Camera: preserve user rotation
            end
        end
    end
end
