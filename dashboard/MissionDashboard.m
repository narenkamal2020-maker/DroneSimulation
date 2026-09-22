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
        txtHeading
        
        % Event Log
        txtEventLog
        logEntries = {}
        
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
            
            % Row 5: Battery Status & Heading
            yStart = yStart - (rowH + spacing);
            [~, obj.txtBattery] = obj.createMetricCard(0.06, yStart, 0.42, rowH, 'BATTERY', '100%', [0.2, 0.9, 0.4]);
            [~, obj.txtHeading] = obj.createMetricCard(0.52, yStart, 0.42, rowH, 'HEADING', '000°', cyanAccent);
            
            % 5. Real-Time Mission Event Log
            yLog = 0.17;
            logH = 0.15;
            uicontrol('Parent', obj.panel, 'Style', 'text', ...
                      'Units', 'normalized', 'Position', [0.06, yLog + logH + 0.005, 0.88, 0.022], ...
                      'String', 'MISSION EVENT LOG', ...
                      'FontSize', 8, 'FontWeight', 'bold', ...
                      'ForegroundColor', textSecondary, 'BackgroundColor', bgColor, ...
                      'HorizontalAlignment', 'left');
                  
            obj.txtEventLog = uicontrol('Parent', obj.panel, 'Style', 'text', ...
                                        'Units', 'normalized', 'Position', [0.06, yLog, 0.88, logH], ...
                                        'String', {'[00:00] SYS: Systems online & calibrated'}, ...
                                        'FontSize', 7.5, 'ForegroundColor', [0.70, 0.85, 0.95], ...
                                        'BackgroundColor', [0.04, 0.06, 0.10], ...
                                        'HorizontalAlignment', 'left', ...
                                        'FontName', 'Consolas', ...
                                        'Max', 10);
            
            % 6. Camera View Controls
            yCam = 0.052;
            uicontrol('Parent', obj.panel, 'Style', 'text', ...
                      'Units', 'normalized', 'Position', [0.06, yCam + 0.078, 0.88, 0.022], ...
                      'String', 'CAMERA VIEW SELECTOR', ...
                      'FontSize', 8, 'FontWeight', 'bold', ...
                      'ForegroundColor', textSecondary, 'BackgroundColor', bgColor, ...
                      'HorizontalAlignment', 'left');
                  
            btnW = 0.42; btnH = 0.035;
            obj.btnCamFollow   = uicontrol('Parent', obj.panel, 'Style', 'pushbutton', ...
                                           'Units', 'normalized', 'Position', [0.06, yCam + 0.038, btnW, btnH], ...
                                           'String', 'Follow Drone', 'FontWeight', 'bold', ...
                                           'BackgroundColor', cardBg, 'ForegroundColor', cyanAccent, ...
                                           'Callback', @(~,~) obj.setCamera(1));
                                       
            obj.btnCamTop      = uicontrol('Parent', obj.panel, 'Style', 'pushbutton', ...
                                           'Units', 'normalized', 'Position', [0.52, yCam + 0.038, btnW, btnH], ...
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
                      'Units', 'normalized', 'Position', [0.05, 0.012, 0.90, 0.025], ...
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
            set(obj.txtBattery, 'String', sprintf('%d%%', batPct), 'ForegroundColor', bCol);
            
            % Heading
            if isfield(data, 'heading') && ~isempty(obj.txtHeading) && isvalid(obj.txtHeading)
                set(obj.txtHeading, 'String', sprintf('%03.0f°', data.heading));
            end
        end
        
        function addLogEntry(obj, simTime, msg, type)
            % Appends timestamped log entry to the mission event feed
            if nargin < 4 || isempty(type)
                type = 'info';
            end
            mins = floor(simTime / 60);
            secs = mod(floor(simTime), 60);
            timeStr = sprintf('%02d:%02d', mins, secs);
            
            switch lower(type)
                case 'warn'
                    tag = 'WARN';
                case 'replan'
                    tag = 'RPLN';
                case 'success'
                    tag = 'OK  ';
                otherwise
                    tag = 'INFO';
            end
            
            entry = sprintf('[%s] %s: %s', timeStr, tag, msg);
            obj.logEntries = [obj.logEntries; {entry}];
            if length(obj.logEntries) > 7
                obj.logEntries(1:end-7) = [];
            end
            if ~isempty(obj.txtEventLog) && isvalid(obj.txtEventLog)
                set(obj.txtEventLog, 'String', obj.logEntries);
            end
        end
        
        function showMissionComplete(obj, summary)
            % Renders a presentation-ready aerospace flight debrief card overlay
            if isempty(obj.fig) || ~ishandle(obj.fig)
                return;
            end
            
            overlay = uipanel('Parent', obj.fig, 'Units', 'normalized', ...
                              'Position', [0.10, 0.14, 0.52, 0.70], ...
                              'BackgroundColor', [0.07, 0.09, 0.15], ...
                              'BorderType', 'line', ...
                              'HighlightColor', [0.0, 0.85, 1.0], ...
                              'BorderWidth', 2);
            
            % Title
            uicontrol('Parent', overlay, 'Style', 'text', 'Units', 'normalized', ...
                      'Position', [0.05, 0.88, 0.90, 0.08], ...
                      'String', 'MISSION DEBRIEF REPORT', ...
                      'FontSize', 14, 'FontWeight', 'bold', ...
                      'ForegroundColor', [0.0, 0.85, 1.0], ...
                      'BackgroundColor', [0.07, 0.09, 0.15], ...
                      'HorizontalAlignment', 'center');
                  
            statusStr = '✓ FLIGHT OBJECTIVE ACCOMPLISHED — ZERO COLLISIONS';
            if summary.collisions > 0
                statusStr = sprintf('⚠ MISSION COMPLETED WITH %d CONTACT EVENTS', summary.collisions);
                statusCol = [1.0, 0.4, 0.2];
            else
                statusCol = [0.1, 0.95, 0.45];
            end
            
            uicontrol('Parent', overlay, 'Style', 'text', 'Units', 'normalized', ...
                      'Position', [0.05, 0.81, 0.90, 0.06], ...
                      'String', statusStr, ...
                      'FontSize', 10, 'FontWeight', 'bold', ...
                      'ForegroundColor', statusCol, ...
                      'BackgroundColor', [0.07, 0.09, 0.15], ...
                      'HorizontalAlignment', 'center');
                  
            % Separator
            uicontrol('Parent', overlay, 'Style', 'text', 'Units', 'normalized', ...
                      'Position', [0.08, 0.78, 0.84, 0.003], ...
                      'String', '', 'BackgroundColor', [0.2, 0.35, 0.5]);
                  
            % KPI Grid
            kpiList = {
                'Total Flight Time:', sprintf('%.2f s', summary.totalMissionTime);
                'Distance Flown:', sprintf('%.2f m (Planned: %.2f m)', summary.actualDistance, summary.plannedDistance);
                'Average Speed:', sprintf('%.2f m/s', summary.avgSpeed);
                'Maximum Speed:', sprintf('%.2f m/s', summary.maxSpeed);
                'Min Obstacle Clearance:', sprintf('%.2f m', summary.minClearance);
                'RMS Tracking Error:', sprintf('%.3f m', summary.rmsTrackingError);
                'Obstacle Detections:', sprintf('%d', summary.obstacleDetections);
                'Dynamic Detour Replans:', sprintf('%d', summary.routeReplans);
            };
            
            yKpi = 0.70;
            for i = 1:size(kpiList, 1)
                uicontrol('Parent', overlay, 'Style', 'text', 'Units', 'normalized', ...
                          'Position', [0.08, yKpi, 0.44, 0.05], ...
                          'String', kpiList{i, 1}, 'FontSize', 9, ...
                          'ForegroundColor', [0.70, 0.80, 0.90], ...
                          'BackgroundColor', [0.07, 0.09, 0.15], ...
                          'HorizontalAlignment', 'left');
                uicontrol('Parent', overlay, 'Style', 'text', 'Units', 'normalized', ...
                          'Position', [0.54, yKpi, 0.38, 0.05], ...
                          'String', kpiList{i, 2}, 'FontSize', 9, 'FontWeight', 'bold', ...
                          'ForegroundColor', [0.0, 0.95, 0.75], ...
                          'BackgroundColor', [0.07, 0.09, 0.15], ...
                          'HorizontalAlignment', 'right');
                yKpi = yKpi - 0.068;
            end
            
            % Dismiss button
            uicontrol('Parent', overlay, 'Style', 'pushbutton', 'Units', 'normalized', ...
                      'Position', [0.26, 0.04, 0.48, 0.07], ...
                      'String', 'CLOSE OVERLAY / EXPLORE 3D SCENE', ...
                      'FontSize', 9, 'FontWeight', 'bold', ...
                      'ForegroundColor', [0.95, 0.95, 0.98], ...
                      'BackgroundColor', [0.15, 0.22, 0.35], ...
                      'Callback', @(~,~) delete(overlay));
        end
    end
end
