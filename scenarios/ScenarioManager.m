classdef ScenarioManager
    % SCENARIOMANAGER Instantiates the 6 simulation mission scenarios.
    
    methods (Static)
        function list = getScenarioList()
            list = { ...
                'dynamic',   '4. Dynamic Moving Obstacle Avoidance (Flagship)'; ...
                'emergency', '5. Sudden Pop-up Emergency Obstacle'; ...
                'urban',     '2. Dense Urban Canyon Navigation'; ...
                'maze',      '3. 3D Spatial Complex Maze'; ...
                'wind',      '6. Turbulent Crosswind Disturbance Rejection'; ...
                'open',      '1. Open Flight Arena Benchmark' ...
            };
        end
        
        function scenario = loadScenario(scenarioId, cfg)
            if nargin < 1 || isempty(scenarioId)
                scenarioId = 'dynamic';
            end
            
            switch lower(scenarioId)
                case {'open', '1'}
                    scenario = ScenarioManager.createOpenScenario(cfg);
                case {'urban', '2'}
                    scenario = ScenarioManager.createUrbanScenario(cfg);
                case {'maze', '3'}
                    scenario = ScenarioManager.createMazeScenario(cfg);
                case {'dynamic', '4'}
                    scenario = ScenarioManager.createDynamicScenario(cfg);
                case {'emergency', '5'}
                    scenario = ScenarioManager.createEmergencyScenario(cfg);
                case {'wind', '6'}
                    scenario = ScenarioManager.createWindScenario(cfg);
                otherwise
                    scenario = ScenarioManager.createDynamicScenario(cfg);
            end
        end
        
        function sc = createOpenScenario(~)
            sc.id = 'open';
            sc.name = 'Open Flight Arena Benchmark';
            sc.startPos = [8, 8, 0];
            sc.goalPos = [72, 72, 8];
            sc.initialYaw = pi/4;
            sc.wind = [0; 0; 0];
            sc.dynamicObstacle = [];
            sc.emergencyObstacle = [];
            
            % Sparse perimeter obstacles
            obs = [ ...
                struct('type', 'building', 'params', [35, 35, 0, 10, 10, 16], 'color', [0.22, 0.28, 0.38]); ...
                struct('type', 'cylinder', 'params', [20, 50, 0, 4, 14], 'color', [0.30, 0.35, 0.45]); ...
                struct('type', 'cylinder', 'params', [50, 20, 0, 4, 14], 'color', [0.30, 0.35, 0.45]) ...
            ];
            sc.obstacles = obs;
        end
        
        function sc = createUrbanScenario(~)
            sc.id = 'urban';
            sc.name = 'Dense Urban Canyon Navigation';
            sc.startPos = [8, 10, 0];
            sc.goalPos = [72, 72, 12];
            sc.initialYaw = pi/4;
            sc.wind = [0; 0; 0];
            sc.dynamicObstacle = [];
            sc.emergencyObstacle = [];
            
            % Skyscraper district
            obs = [ ...
                struct('type', 'building', 'params', [18, 5, 0, 12, 14, 20], 'color', [0.20, 0.25, 0.35]); ...
                struct('type', 'building', 'params', [18, 25, 0, 12, 16, 22], 'color', [0.24, 0.30, 0.40]); ...
                struct('type', 'building', 'params', [18, 48, 0, 12, 16, 18], 'color', [0.22, 0.27, 0.36]); ...
                struct('type', 'building', 'params', [38, 12, 0, 14, 18, 24], 'color', [0.18, 0.22, 0.30]); ...
                struct('type', 'building', 'params', [38, 38, 0, 14, 18, 22], 'color', [0.25, 0.32, 0.42]); ...
                struct('type', 'cylinder', 'params', [60, 25, 0, 6, 20], 'color', [0.32, 0.38, 0.48]); ...
                struct('type', 'building', 'params', [56, 42, 0, 12, 14, 19], 'color', [0.21, 0.26, 0.34]); ...
                struct('type', 'building', 'params', [38, 62, 0, 14, 14, 17], 'color', [0.26, 0.33, 0.43]) ...
            ];
            sc.obstacles = obs;
        end
        
        function sc = createMazeScenario(~)
            sc.id = 'maze';
            sc.name = '3D Spatial Complex Maze';
            sc.startPos = [8, 8, 0];
            sc.goalPos = [72, 72, 10];
            sc.initialYaw = pi/4;
            sc.wind = [0; 0; 0];
            sc.dynamicObstacle = [];
            sc.emergencyObstacle = [];
            
            % Interlocking wall barriers
            obs = [ ...
                struct('type', 'building', 'params', [20, 0, 0, 6, 50, 15], 'color', [0.22, 0.28, 0.38]); ...
                struct('type', 'building', 'params', [40, 25, 0, 6, 55, 15], 'color', [0.24, 0.30, 0.40]); ...
                struct('type', 'building', 'params', [58, 0, 0, 6, 50, 15], 'color', [0.20, 0.25, 0.35]); ...
                struct('type', 'cylinder', 'params', [23, 62, 0, 5, 14], 'color', [0.32, 0.38, 0.48]); ...
                struct('type', 'cylinder', 'params', [43, 12, 0, 5, 14], 'color', [0.32, 0.38, 0.48]) ...
            ];
            sc.obstacles = obs;
        end
        
        function sc = createDynamicScenario(~)
            sc.id = 'dynamic';
            sc.name = 'Dynamic Moving Obstacle Avoidance (Flagship)';
            sc.startPos = [8, 12, 0];
            sc.goalPos = [72, 68, 10];
            sc.initialYaw = pi/4;
            sc.wind = [0; 0; 0];
            sc.emergencyObstacle = [];
            
            % Static corridor barriers
            obs = [ ...
                struct('type', 'building', 'params', [16, 28, 0, 10, 14, 18], 'color', [0.20, 0.25, 0.35]); ...
                struct('type', 'building', 'params', [48, 14, 0, 12, 12, 18], 'color', [0.24, 0.30, 0.40]); ...
                struct('type', 'cylinder', 'params', [60, 46, 0, 5, 16], 'color', [0.28, 0.35, 0.45]); ...
                struct('type', 'building', 'params', [30, 54, 0, 14, 12, 17], 'color', [0.22, 0.27, 0.37]) ...
            ];
            sc.obstacles = obs;
            
            % Dynamic moving hazard (crosses diagonal flight corridor directly)
            dyn.pos = [52, 26, 10];
            dyn.startPos = [54, 24, 10];
            dyn.endPos = [26, 52, 10];
            dyn.size = [5.0, 5.0, 6.0]; % [dx, dy, dz]
            dyn.period = 28.0;          % Motion period
            dyn.trajectoryFunc = [];
            sc.dynamicObstacle = dyn;
        end
        
        function sc = createEmergencyScenario(~)
            sc.id = 'emergency';
            sc.name = 'Sudden Pop-up Emergency Obstacle';
            sc.startPos = [8, 10, 0];
            sc.goalPos = [72, 45, 10];
            sc.initialYaw = 0.45;
            sc.wind = [0; 0; 0];
            sc.dynamicObstacle = [];
            
            % Base corridor obstacles
            obs = [ ...
                struct('type', 'building', 'params', [22, 30, 0, 10, 14, 18], 'color', [0.20, 0.25, 0.35]); ...
                struct('type', 'building', 'params', [50, 18, 0, 12, 12, 18], 'color', [0.24, 0.30, 0.40]); ...
                struct('type', 'cylinder', 'params', [62, 55, 0, 5, 16], 'color', [0.28, 0.35, 0.45]) ...
            ];
            sc.obstacles = obs;
            
            % Sudden intrusion barrier that appears right in the flight corridor
            em.pos = [36, 26, 7];
            em.size = [6.0, 6.0, 14.0];
            em.triggerLocation = [26, 20, 10]; % Drone triggers when approaching this coordinate
            em.triggerDistance = 14.0;
            em.triggerTime = 11.0;
            sc.emergencyObstacle = em;
        end
        
        function sc = createWindScenario(~)
            sc.id = 'wind';
            sc.name = 'Turbulent Crosswind Disturbance Rejection';
            sc.startPos = [8, 12, 0];
            sc.goalPos = [72, 68, 10];
            sc.initialYaw = pi/4;
            sc.dynamicObstacle = [];
            sc.emergencyObstacle = [];
            
            % Strong crosswind vector [Vx, Vy, Vz]
            sc.wind = [3.5; -2.0; 0.2];
            
            obs = [ ...
                struct('type', 'building', 'params', [22, 20, 0, 10, 16, 18], 'color', [0.20, 0.25, 0.35]); ...
                struct('type', 'building', 'params', [45, 35, 0, 12, 14, 20], 'color', [0.24, 0.30, 0.40]); ...
                struct('type', 'cylinder', 'params', [35, 60, 0, 5, 16], 'color', [0.28, 0.35, 0.45]); ...
                struct('type', 'building', 'params', [58, 20, 0, 10, 12, 16], 'color', [0.22, 0.27, 0.37]) ...
            ];
            sc.obstacles = obs;
        end
    end
end
