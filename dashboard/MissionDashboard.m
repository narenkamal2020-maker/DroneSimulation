classdef MissionDashboard < handle
    % MISSIONDASHBOARD Professional Aerospace UAV Mission-Control HUD & Telemetry.
    %
    % Displays real-time mission state, telemetry cards, alerts, and camera controls.
    % All internal path search details remain completely encapsulated.
    
    properties
        cfg
        fig
        panel
        cameraCtrl
        
        % UI Elements
        lblTitle
        lblSubTitle
        lblStatus
        lblAlert
        
        % Telemetry value text handles
        txtAltitude
        txtSpeed
        txtDistGoal
        txtTime
        txtObstacles
        txtRouteUpdates
        txtClearance
        txtCollisions
        txtBattery
        
        btnCamFollow
        btnCamTop
        btnCamOverview
        btnCamFree
    end
    
    methods
        function obj = MissionDashboard(cfg, fig, cameraCtrl, scenarioName)
            obj.cfg = cfg;
            obj.fig = fig;
            obj.cameraCtrl = cameraCtrl;
            
            if nargin < 4 || isempty(scenarioName)
                scenarioName = 'Autonomous Flight Mission';
            end
            
            % Dark aerospace cockpit theme
            bgColor = [0.06, 0.08, 0.13];
            cardBg = [0.10, 0.13, 0.20];
            textPrimary = [0.92, 0.95, 0.98];
            textSecondary = [0.55, 0.65, 0.75];
            cyanAccent = [0.0, 0.85, 1.0];
            
            % Create main right-hand mission panel (28% width)
            obj.panel = uipanel('Parent', fig, ...
                                'Units', 'normalized', ...
                                'Position', [0.72, 0.0, 0.28, 1.0], ...
                                'BackgroundColor', bgColor, ...
                                'BorderType', 'line', ...
                                'HighlightColor', [0.18, 0.24, 0.35]);
                            
            % 1. Header & Title Block
            uicontrol('Parent', obj.panel, 'Style', 'text', ...
                      'Units', 'normalized', 'Position', [0.05, 0.93, 0.90, 0.045], ...
                      'String', 'AUTONOMOUS UAV', ...
                      'FontSize', 14, 'FontWeight', 'bold', ...
                      'ForegroundColor', cyanAccent, 'BackgroundColor', bgColor, ...
                      'HorizontalAlignment', 'center');
                  
            uicontrol('Parent', obj.panel, 'Style', 'text', ...
                      'Units', 'normalized', 'Position', [0.05, 0.895, 0.90, 0.03], ...
                      'String', 'MISSION CONTROL SYSTEM', ...
                      'FontSize', 9, 'FontWeight', 'bold', ...
                      'ForegroundColor', textSecondary, 'BackgroundColor', bgColor, ...
                      'HorizontalAlignment', 'center');
                  
            % Scenario Name Tag
            uicontrol('Parent', obj.panel, 'Style', 'text', ...
                      'Units', 'normalized', 'Position', [0.05, 0.865, 0.90, 0.025], ...
                      'String', upper(scenarioName), ...
                      'FontSize', 8, 'ForegroundColor', [0.4, 0.75, 0.9], ...
                      'BackgroundColor', cardBg, 'HorizontalAlignment', 'center');
                  
            % 2. Mission State Badge
            uicontrol('Parent', obj.panel, 'Style', 'text', ...
                      'Units', 'normalized', 'Position', [0.06, 0.815, 0.88, 0.025], ...
                      'String', 'MISSION STATUS', ...
                      'FontSize', 8, 'FontWeight', 'bold', ...
                      'ForegroundColor', textSecondary, 'BackgroundColor', bgColor, ...
                      'HorizontalAlignment', 'left');
                  
            obj.lblStatus = uicontrol('Parent', obj.panel, 'Style', 'text', ...
                                      'Units', 'normalized', 'Position', [0.06, 0.765, 0.88, 0.045], ...
                                      'String', '● INITIALIZING', ...
                                      'FontSize', 12, 'FontWeight', 'bold', ...
                                      'ForegroundColor', [0.0, 0.95, 0.55], ...
                                      'BackgroundColor', cardBg, ...
                                      'HorizontalAlignment', 'center');
                                  
            % 3. Real-Time Alert Banner
            obj.lblAlert = uicontrol('Parent', obj.panel, 'Style', 'text', ...
                                     'Units', 'normalized', 'Position', [0.06, 0.695, 0.88, 0.06], ...
                                     'String', 'SYSTEMS NOMINAL - CORRIDOR CLEAR', ...
                                     'FontSize', 9, 'FontWeight', 'bold', ...
                                     'ForegroundColor', [0.4, 0.9, 0.6], ...
                                     'BackgroundColor', [0.08, 0.16, 0.14], ...
                                     'HorizontalAlignment', 'center');
                                 
            % 4. Telemetry Metrics Display Grid
            yStart = 0.62;
            rowH = 0.052;
            spacing = 0.008;
            
            % Row 1: Altitude & Speed
            [~, obj.txtAltitude] = obj.createMetricCard(0.06, yStart, 0.42, rowH, 'ALTITUDE', '0.0 m', cyanAccent);
            [~, obj.txtSpeed]    = obj.createMetricCard(0.52, yStart, 0.42, rowH, 'SPEED', '0.0 m/s', cyanAccent);
            
            % Row 2: Distance to Goal & Mission Time
            yStart = yStart - (rowH + spacing);
            [~, obj.txtDistGoal] = obj.createMetricCard(0.06, yStart, 0.42, rowH, 'DIST TO GOAL', '0.0 m', textPrimary);
            [~, obj.txtTime]     = obj.createMetricCard(0.52, yStart, 0.42, rowH, 'MISSION TIME', '00:00', textPrimary);
            
            % Row 3: Obstacles Detected & Route Updates
            yStart = yStart - (rowH + spacing);
            [~, obj.txtObstacles]    = obj.createMetricCard(0.06, yStart, 0.42, rowH, 'OBSTACLES DETECTED', '00', [1.0, 0.65, 0.1]);
            [~, obj.txtRouteUpdates] = obj.createMetricCard(0.52, yStart, 0.42, rowH, 'ROUTE UPDATES', '00', [0.0, 0.95, 0.55]);
            
            % Row 4: Minimum Clearance & Collisions
            yStart = yStart - (rowH + spacing);
            [~, obj.txtClearance]  = obj.createMetricCard(0.06, yStart, 0.42, rowH, 'MIN CLEARANCE', '-- m', textPrimary);
            [~, obj.txtCollisions] = obj.createMetricCard(0.52, yStart, 0.42, rowH, 'COLLISIONS', '00', [0.2, 1.0, 0.4]);
            
            % Row 5: Battery Status
            yStart = yStart - (rowH + spacing);
            [~, obj.txtBattery] = obj.createMetricCard(0.06, yStart, 0.88, rowH, 'ONBOARD BATTERY', '100% [NOMINAL]', [0.2, 0.9, 0.4]);
            
            % 5. Camera View Controls
            yCam = 0.16;
            uicontrol('Parent', obj.panel, 'Style', 'text', ...
                      'Units', 'normalized', 'Position', [0.06, yCam + 0.085, 0.88, 0.025], ...
                      'String', 'CAMERA VIEW SELECTOR', ...
                      'FontSize', 8, 'FontWeight', 'bold', ...
                      'ForegroundColor', textSecondary, 'BackgroundColor', bgColor, ...
                      'HorizontalAlignment', 'left');
                  
            btnW = 0.42; btnH = 0.038;
            obj.btnCamFollow   = uicontrol('Parent', obj.panel, 'Style', 'pushbutton', ...
                                           'Units', 'normalized', 'Position', [0.06, yCam + 0.042, btnW, btnH], ...
                                           'String', 'Follow Drone', 'FontWeight', 'bold', ...
                                           'BackgroundColor', cardBg, 'ForegroundColor', cyanAccent, ...
                                           'Callback', @(~,~) obj.setCamera(1));
                                       
            obj.btnCamTop      = uicontrol('Parent', obj.panel, 'Style', 'pushbutton', ...
                                           'Units', 'normalized', 'Position', [0.52, yCam + 0.042, btnW, btnH], ...
                                           'String', 'Top View', 'FontWeight', 'bold', ...
                                           'BackgroundColor', cardBg, 'ForegroundColor', textPrimary, ...
                                           'Callback', @(~,~) obj.setCamera(2));
                                       
            obj.btnCamOverview = uicontrol('Parent', obj.panel, 'Style', 'pushbutton', ...
                                           'Units', 'normalized', 'Position', [0.06, yCam, btnW, btnH], ...
                                           'String', 'Overview', 'FontWeight', 'bold', ...
                                           'BackgroundColor', cardBg, 'ForegroundColor', textPrimary, ...
                                           'Callback', @(~,~) obj.setCamera(3));
                                       
            obj.btnCamFree     = uicontrol('Parent', obj.panel, 'Style', 'pushbutton', ...
                                           'Units', 'normalized', 'Position', [0.52, yCam, btnW, btnH], ...
                                           'String', 'Free Orbit', 'FontWeight', 'bold', ...
                                           'BackgroundColor', cardBg, 'ForegroundColor', textPrimary, ...
                                           'Callback', @(~,~) obj.setCamera(4));
                                       
            % Footer watermark
            uicontrol('Parent', obj.panel, 'Style', 'text', ...
                      'Units', 'normalized', 'Position', [0.05, 0.015, 0.90, 0.03], ...
                      'String', 'AUTONOMOUS PERCEPTION & CONTROL LAB', ...
                      'FontSize', 7, 'ForegroundColor', [0.35, 0.45, 0.55], ...
                      'BackgroundColor', bgColor, 'HorizontalAlignment', 'center');
        end
        
        function [hCard, hVal] = createMetricCard(obj, x, y, w, h, titleStr, initVal, valColor)
            bgColor = [0.10, 0.13, 0.20];
            hCard = uipanel('Parent', obj.panel, 'Units', 'normalized', ...
                            'Position', [x, y, w, h], ...
                            'BackgroundColor', bgColor, ...
                            'BorderType', 'line', 'HighlightColor', [0.18, 0.24, 0.35]);
                        
            uicontrol('Parent', hCard, 'Style', 'text', ...
                      'Units', 'normalized', 'Position', [0.05, 0.52, 0.90, 0.44], ...
                      'String', titleStr, 'FontSize', 7, 'FontWeight', 'bold', ...
                      'ForegroundColor', [0.55, 0.65, 0.75], 'BackgroundColor', bgColor, ...
                      'HorizontalAlignment', 'center');
                  
            hVal = uicontrol('Parent', hCard, 'Style', 'text', ...
                             'Units', 'normalized', 'Position', [0.05, 0.04, 0.90, 0.50], ...
                             'String', initVal, 'FontSize', 10, 'FontWeight', 'bold', ...
                             'ForegroundColor', valColor, 'BackgroundColor', bgColor, ...
                             'HorizontalAlignment', 'center');
        end
        
        function setCamera(obj, mode)
            if ~isempty(obj.cameraCtrl)
                obj.cameraCtrl.setMode(mode);
            end
        end
        
        function updateStatus(obj, stateStr, alertStr, alertType)
            % Updates the mission state badge and notification banner
            if nargin < 4 || isempty(alertType)
                alertType = 'nominal';
            end
            
            % Color coding by state
            switch upper(stateStr)
                case {'OBSTACLE DETECTED', 'AVOIDING'}
                    col = [1.0, 0.35, 0.15]; % Crimson-orange
                case {'REPLANNING', 'CALCULATING SAFE ROUTE'}
                    col = [1.0, 0.75, 0.10]; % Amber
                case {'MISSION COMPLETE', 'DESTINATION REACHED'}
                    col = [0.1, 0.95, 0.45]; % Emerald green
                case {'NAVIGATING', 'SAFE ROUTE GENERATED'}
                    col = [0.0, 0.85, 1.00]; % Electric cyan
                case {'TAKEOFF', 'ARMING', 'HOVER'}
                    col = [0.4, 0.75, 1.00]; % Blue
                case {'LANDING', 'GOAL APPROACH'}
                    col = [0.8, 0.60, 1.00]; % Purple-blue
                otherwise
                    col = [0.8, 0.85, 0.90];
            end
            
            set(obj.lblStatus, 'String', ['● ', upper(stateStr)], 'ForegroundColor', col);
            
            if nargin >= 3 && ~isempty(alertStr)
                switch lower(alertType)
                    case 'warning'
                        bgA = [0.22, 0.08, 0.08];
                        fgA = [1.0, 0.35, 0.20];
                    case 'success'
                        bgA = [0.08, 0.20, 0.12];
                        fgA = [0.2, 0.95, 0.5];
                    otherwise
                        bgA = [0.08, 0.14, 0.20];
                        fgA = [0.4, 0.85, 1.0];
                end
                set(obj.lblAlert, 'String', alertStr, 'BackgroundColor', bgA, 'ForegroundColor', fgA);
            end
        end
        
        function updateTelemetry(obj, data)
            % Updates all numeric readouts
            %
            % data fields:
            %   altitude, speed, distGoal, time, numObstacles,
            %   routeUpdates, minClearance, collisions, battery
            
            set(obj.txtAltitude, 'String', sprintf('%.1f m', data.altitude));
            set(obj.txtSpeed, 'String', sprintf('%.1f m/s', data.speed));
            set(obj.txtDistGoal, 'String', sprintf('%.1f m', data.distGoal));
            
            mins = floor(data.time / 60);
            secs = mod(floor(data.time), 60);
            set(obj.txtTime, 'String', sprintf('%02d:%02d', mins, secs));
            
            set(obj.txtObstacles, 'String', sprintf('%02d', data.numObstacles));
            set(obj.txtRouteUpdates, 'String', sprintf('%02d', data.routeUpdates));
            
            if isinf(data.minClearance)
                set(obj.txtClearance, 'String', '> 20 m');
            else
                set(obj.txtClearance, 'String', sprintf('%.2f m', data.minClearance));
            end
            
            set(obj.txtCollisions, 'String', sprintf('%02d', data.collisions));
            
            % Battery
            batPct = max(0, min(100, round(data.battery)));
            if batPct > 50
                bCol = [0.2, 0.9, 0.4];
            elseif batPct > 20
                bCol = [1.0, 0.7, 0.1];
            else
                bCol = [1.0, 0.2, 0.2];
            end
            set(obj.txtBattery, 'String', sprintf('%d%% [NOMINAL]', batPct), 'ForegroundColor', bCol);
        end
    end
end
