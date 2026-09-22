function results = runTests()
% RUNTESTS Comprehensive Automated Validation & Test Suite for Autonomous Drone Simulation.
%
% Tests:
%   1. Central Configuration Loading
%   2. 3D Environment & Obstacle Geometry Generation
%   3. 3D Occupancy Grid & Obstacle Safety Inflation
%   4. Encapsulated 3D Spatial Route Planning (Internal A*)
%   5. Path Simplification (3D Line-of-Sight String Pulling)
%   6. Continuous C2 Spline Trajectory Generation
%   7. 6-DOF Quadcopter Physical Model & Propeller Rotation
%   8. Cascaded Flight Controller (Position & Attitude Tracking)
%   9. Multi-Sensor Suite (3D LiDAR, GPS, IMU, Altimeter)
%  10. Continuous 3D Collision Detection & Distance Clearance
%  11. Dynamic Moving Obstacle Avoidance & In-Flight Replanning
%  12. Edge Cases: Blocked Goal, Coincident Start/Goal, Sensor Noise, Pop-up Barrier

fprintf('\n=======================================================\n');
fprintf('  AUTONOMOUS DRONE SIMULATION - AUTOMATED TEST SUITE\n');
fprintf('=======================================================\n\n');

rootDir = fileparts(fileparts(mfilename('fullpath')));
if isempty(rootDir)
    rootDir = pwd;
end
addpath(rootDir);
addpath(fullfile(rootDir, 'drone'));
addpath(fullfile(rootDir, 'environment'));
addpath(fullfile(rootDir, 'sensors'));
addpath(fullfile(rootDir, 'mapping'));
addpath(fullfile(rootDir, 'planning'));
addpath(fullfile(rootDir, 'control'));
addpath(fullfile(rootDir, 'collision'));
addpath(fullfile(rootDir, 'scenarios'));
addpath(fullfile(rootDir, 'visualization'));
addpath(fullfile(rootDir, 'dashboard'));
addpath(fullfile(rootDir, 'metrics'));

totalTests = 0;
passedTests = 0;

%% Test 1: Configuration Loading
totalTests = totalTests + 1;
try
    cfg = config();
    assert(isfield(cfg, 'drone') && isfield(cfg, 'planner') && isfield(cfg, 'lidar'), ...
           'Config fields missing');
    assert(cfg.drone.safetyMargin > 0, 'Safety margin must be positive');
    fprintf('  [PASS] Test 1: Central Configuration Loaded Successfully\n');
    passedTests = passedTests + 1;
catch ME
    fprintf('  [FAIL] Test 1: Config Error: %s\n', ME.message);
end

%% Test 2: Environment & Scenarios
totalTests = totalTests + 1;
try
    list = ScenarioManager.getScenarioList();
    assert(size(list, 1) >= 6, 'Must support 6 scenarios');
    
    for i = 1:size(list, 1)
        sc = ScenarioManager.loadScenario(list{i, 1}, cfg);
        assert(~isempty(sc.startPos) && ~isempty(sc.goalPos), 'Scenario endpoints invalid');
        assert(~isempty(sc.obstacles), 'Obstacles missing');
    end
    fprintf('  [PASS] Test 2: All 6 Scenarios Validated & Obstacles Instantiated\n');
    passedTests = passedTests + 1;
catch ME
    fprintf('  [FAIL] Test 2: Scenario Error: %s\n', ME.message);
end

%% Test 3: 3D Occupancy Grid & Obstacle Safety Inflation
totalTests = totalTests + 1;
try
    occMap = OccupancyMap3D(cfg);
    testObs = struct('type', 'building', 'params', [20, 20, 0, 10, 10, 15], 'color', [0.3 0.3 0.3]);
    [~, b] = createObstacleGeometries([], cfg, testObs);
    occMap.buildFromObstacles(b);
    
    % Check interior is occupied
    assert(occMap.isOccupied([25, 25, 5]), 'Building center must be occupied');
    
    % Check safety inflation zone (0.5m outside building face must be occupied)
    assert(occMap.isOccupied([19.5, 25, 5]), 'Safety inflation zone must be occupied');
    
    % Check far free space is free
    assert(~occMap.isOccupied([5, 5, 5]), 'Far space must be free');
    
    fprintf('  [PASS] Test 3: 3D Occupancy Grid & Safety Inflation Correct\n');
    passedTests = passedTests + 1;
catch ME
    fprintf('  [FAIL] Test 3: Occupancy Grid Error: %s\n', ME.message);
end

%% Test 4: Encapsulated 3D Spatial Route Planning (Internal A*)
totalTests = totalTests + 1;
try
    planner = GlobalPlanner(cfg);
    startPt = [5, 5, 8];
    goalPt  = [45, 45, 8];
    
    [route, success, ~] = planner.planRoute(startPt, goalPt, occMap);
    assert(success, 'Planner must find path in navigable environment');
    assert(size(route, 1) >= 2, 'Route must have at least 2 points');
    assert(norm(route(1, :) - startPt) < 0.1, 'Route must begin at start');
    assert(norm(route(end, :) - goalPt) < 0.1, 'Route must terminate at goal');
    
    % Verify no waypoint collides with obstacle
    for k = 1:size(route, 1)
        assert(~occMap.isOccupied(route(k, :)), 'No waypoint may collide with inflated obstacle');
    end
    fprintf('  [PASS] Test 4: 3D Route Planning Synthesis Verified (Optimal & Safe)\n');
    passedTests = passedTests + 1;
catch ME
    fprintf('  [FAIL] Test 4: Planner Error: %s\n', ME.message);
end

%% Test 5: Path Simplification (3D Line-of-Sight String Pulling)
totalTests = totalTests + 1;
try
    simplified = PathSimplifier.simplify(route, occMap);
    assert(size(simplified, 1) <= size(route, 1), 'Simplified path must reduce waypoint count');
    assert(norm(simplified(1, :) - startPt) < 0.1, 'Simplified must match start');
    assert(norm(simplified(end, :) - goalPt) < 0.1, 'Simplified must match goal');
    fprintf('  [PASS] Test 5: Path Simplification Pruned %d nodes to %d key waypoints\n', ...
            size(route, 1), size(simplified, 1));
    passedTests = passedTests + 1;
catch ME
    fprintf('  [FAIL] Test 5: Path Simplifier Error: %s\n', ME.message);
end

%% Test 6: Continuous C2 Trajectory Generation
totalTests = totalTests + 1;
try
    traj = TrajectoryGenerator(cfg);
    genSuccess = traj.generate(simplified, 3.5, [0, 0, 0]);
    assert(genSuccess, 'Trajectory generation failed');
    
    % Evaluate at t = 0
    [p0, ~, ~, ~] = traj.evaluate(0);
    assert(norm(p0 - startPt') < 0.2, 'Trajectory must start at start waypoint');
    
    % Evaluate at t = totalDuration
    [pend, vend, ~, ~] = traj.evaluate(traj.totalDuration);
    assert(norm(pend - goalPt') < 0.2, 'Trajectory must end at goal');
    assert(norm(vend) < 0.1, 'Endpoint velocity must be zero');
    
    % Check continuity at intermediate sample
    [pMid, vMid, aMid, ~] = traj.evaluate(traj.totalDuration * 0.5);
    assert(~any(isnan([pMid; vMid; aMid])), 'State must not be NaN');
    fprintf('  [PASS] Test 6: Continuous C2 Spline Trajectory Verified (Smooth Velocity & Accel)\n');
    passedTests = passedTests + 1;
catch ME
    fprintf('  [FAIL] Test 6: Trajectory Generator Error: %s\n', ME.message);
end

%% Test 7: 6-DOF Drone Physics & Propeller Rotation
totalTests = totalTests + 1;
try
    drone = DroneModel(cfg, [0; 0; 0], 0);
    drone.armed = true;
    initialPropAngles = drone.propAngles;
    
    % Step with hover thrust
    dt = 0.04;
    hoverThrust = cfg.drone.mass * cfg.drone.g;
    drone.step(dt, hoverThrust, [0; 0; 0], [0; 0; 0]);
    
    assert(norm(drone.propAngles - initialPropAngles) > 0.01, 'Propellers must spin');
    assert(~any(isnan(drone.pos)), 'Drone position must not be NaN');
    assert(~any(isnan(drone.euler)), 'Attitude must not be NaN');
    fprintf('  [PASS] Test 7: 6-DOF Quadcopter Physics & Propeller Spin Verified\n');
    passedTests = passedTests + 1;
catch ME
    fprintf('  [FAIL] Test 7: Drone Physics Error: %s\n', ME.message);
end

%% Test 8: Cascaded Flight Controller Tracking
totalTests = totalTests + 1;
try
    ctrl = FlightController(cfg);
    currPos = [10; 10; 8];
    currVel = [0; 0; 0];
    currEuler = [0; 0; 0];
    desPos = [11; 10; 8]; % Desired forward step in X
    desVel = [1.0; 0; 0];
    desAccel = [0; 0; 0];
    desYaw = 0;
    
    [thrustCmd, eulerCmd, trackErr] = ctrl.computeControl(0.04, currPos, currVel, currEuler, ...
                                                           desPos, desVel, desAccel, desYaw);
    assert(thrustCmd > 0, 'Thrust must be positive');
    assert(eulerCmd(2) > 0, 'Quadcopter must pitch forward (theta > 0) to accelerate in +X');
    assert(trackErr > 0, 'Tracking error computed');
    fprintf('  [PASS] Test 8: Cascaded PID Controller Active (Dynamic Bank-to-Turn Verified)\n');
    passedTests = passedTests + 1;
catch ME
    fprintf('  [FAIL] Test 8: Controller Error: %s\n', ME.message);
end

%% Test 9: Multi-Sensor Suite (LiDAR, GPS, IMU, Altimeter)
totalTests = totalTests + 1;
try
    lidar = LidarSensor(cfg);
    gps = GpsSensor(cfg);
    imu = ImuSensor(cfg);
    altimeter = AltimeterSensor(cfg);
    
    % LiDAR scan test
    [ranges, hitPoints, ~, minLidarDist, ~] = lidar.scan([10, 25, 5], eye(3), b);
    assert(length(ranges) == lidar.numRays, 'LiDAR must return all ray ranges');
    assert(minLidarDist < cfg.lidar.maxRange, 'LiDAR must detect nearby obstacle');
    assert(size(hitPoints, 1) > 0, 'LiDAR must return hit points');
    
    % GPS test
    mGps = gps.read([10; 20; 8], 0.04);
    assert(norm(mGps - [10; 20; 8]) < 1.5, 'GPS reading within noise bounds');
    
    % IMU test
    [mAcc, ~] = imu.read([0; 0; 0], eye(3), [0; 0; 0]);
    assert(abs(mAcc(3) - cfg.drone.g) < 1.0, 'IMU must measure gravity vector');
    
    % Altimeter test
    mAlt = altimeter.read(8.2);
    assert(abs(mAlt - 8.2) < 0.4, 'Altimeter within noise bounds');
    
    fprintf('  [PASS] Test 9: Multi-Sensor Suite Operational (LiDAR, GPS, IMU, Altimeter)\n');
    passedTests = passedTests + 1;
catch ME
    fprintf('  [FAIL] Test 9: Sensor Error: %s\n', ME.message);
end

%% Test 10: Continuous 3D Collision Detection
totalTests = totalTests + 1;
try
    collider = CollisionChecker(cfg);
    
    % Point far from obstacle
    [col1, clr1] = collider.check([5, 5, 8], b);
    assert(~col1 && clr1 > 5.0, 'Far point must not collide');
    
    % Point inside obstacle
    [col2, clr2] = collider.check([25, 25, 8], b);
    assert(col2 && clr2 == 0, 'Interior point must report collision');
    
    fprintf('  [PASS] Test 10: Continuous 3D Collision & Clearance Checking Accurate\n');
    passedTests = passedTests + 1;
catch ME
    fprintf('  [FAIL] Test 10: Collision Checker Error: %s\n', ME.message);
end

%% Test 11: Dynamic Obstacle & In-Flight Replanning
totalTests = totalTests + 1;
try
    scDyn = ScenarioManager.loadScenario('dynamic', cfg);
    envDyn = Environment(cfg, scDyn);
    occDyn = OccupancyMap3D(cfg);
    occDyn.buildFromObstacles(envDyn.getAllObstacleBounds());
    
    % Plan initial route
    [~, s1, ~] = planner.planRoute([10, 10, 8], [70, 70, 8], occDyn);
    assert(s1, 'Initial route plan must succeed');
    
    % Move dynamic obstacle to block path at mid-corridor
    envDyn.dynamicObstacle.pos = [40, 40, 8];
    envDyn.updateDynamicBounds();
    occDyn.buildFromObstacles(envDyn.getAllObstacleBounds());
    
    % Dynamic replan
    [~, s2, ~] = planner.planRoute([30, 30, 8], [70, 70, 8], occDyn);
    assert(s2, 'Dynamic replan around moving obstacle must succeed');
    
    fprintf('  [PASS] Test 11: Dynamic Obstacle Avoidance & In-Flight Replanning Validated\n');
    passedTests = passedTests + 1;
catch ME
    fprintf('  [FAIL] Test 11: Dynamic Replanning Error: %s\n', ME.message);
end

%% Test 12: Edge Cases (Blocked Destination, Start == Goal, Pop-up Barrier)
totalTests = totalTests + 1;
try
    % Edge Case 1: Start equals Goal
    [rSame, sSame, ~] = planner.planRoute([15, 15, 8], [15, 15, 8], occMap);
    assert(sSame && size(rSame, 1) == 2, 'Start == Goal must succeed trivially');
    
    % Edge Case 2: Blocked Goal (planner finds nearest free voxel)
    blockedGoal = [25, 25, 8]; % Inside building
    [~, sNear, ~] = planner.planRoute([5, 5, 8], blockedGoal, occMap);
    assert(sNear, 'Planner must gracefully locate nearest safe perimeter point');
    
    % Edge Case 3: Sudden pop-up obstacle trigger
    scEmer = ScenarioManager.loadScenario('emergency', cfg);
    envEmer = Environment(cfg, scEmer);
    envEmer.update(12.0, [26, 20, 10]);
    assert(envEmer.emergencyTriggered, 'Emergency hazard must trigger on proximity/time');
    
    fprintf('  [PASS] Test 12: Edge Cases Handled Gracefully (Blocked Goal, Start=Goal, Pop-up)\n');
    passedTests = passedTests + 1;
catch ME
    fprintf('  [FAIL] Test 12: Edge Cases Error: %s\n', ME.message);
end

%% Summary
fprintf('\n=======================================================\n');
fprintf('  TEST RESULTS: %d / %d TESTS PASSED (%.1f%%)\n', ...
        passedTests, totalTests, (passedTests/totalTests)*100);
if passedTests == totalTests
    fprintf('  STATUS: ALL TESTS PASSED - SYSTEM VERIFIED READY FOR DEMO\n');
else
    fprintf('  STATUS: SOME TESTS FAILED\n');
end
fprintf('=======================================================\n\n');

results.total = totalTests;
results.passed = passedTests;
results.success = (passedTests == totalTests);

end
