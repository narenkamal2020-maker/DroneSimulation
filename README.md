# Autonomous 3D Drone Navigation & Dynamic Obstacle Avoidance Simulation

A state-of-the-art **3D Autonomous Drone Simulation** developed in native **MATLAB** featuring high-fidelity quadcopter physics, multi-sensor perception (3D LiDAR, GPS, IMU, Altimeter), cascaded PID flight control with dynamic banking, in-flight moving obstacle avoidance, real-time spatial replanning, and an aerospace-grade Mission Control Dashboard.

---

## 1. Executive Summary

This simulation models the complete autonomous flight lifecycle of a modern quadrotor UAV:
$$\mathbf{Takeoff} \longrightarrow \mathbf{Sense} \longrightarrow \mathbf{Plan} \longrightarrow \mathbf{Navigate} \longrightarrow \mathbf{Detect\ Hazards} \longrightarrow \mathbf{Replan} \longrightarrow \mathbf{Avoid} \longrightarrow \mathbf{Goal\ Approach} \longrightarrow \mathbf{Precision\ Landing}$$

### Architectural Highlight: Encapsulated Global Planning
The underlying global path planning engine uses **3D A\*** operating on a 26-connected spatial lattice. To provide a professional aerospace autonomous-agent presentation, all search graph internals ($f(n), g(n)$, open/closed sets) are **strictly encapsulated**. The user-facing dashboard, visual overlays, and event logs present an authentic autonomous flight system (*Autonomous Navigation, Safe Corridor Analysis, Dynamic Replanning, Trajectory Optimization*).

---

## 2. System Architecture

```
                                  ┌───────────────────────────┐
                                  │      Scenario Manager     │
                                  │   (6 Scenarios + Winds)   │
                                  └─────────────┬─────────────┘
                                                │
                                                ▼
┌───────────────────────┐         ┌───────────────────────────┐         ┌───────────────────────┐
│     Sensor Suite      │ ──────> │    3D Occupancy Grid      │ ──────> │   Autonomous Route    │
│ (LiDAR, GPS, IMU, Alt)│         │   (Safety Margin Infl.)   │         │ Planner (Internal A*) │
└───────────────────────┘         └───────────────────────────┘         └───────────┬───────────┘
            ▲                                                                       │ Waypoints
            │ State Feedback                                                        ▼
┌───────────────────────┐         ┌───────────────────────────┐         ┌───────────────────────┐
│     Drone Physics     │ <────── │   Cascaded Flight Ctrl    │ <────── │ Trajectory Generator  │
│  (6-DOF Quadcopter)   │         │ (Position & Attitude PID) │         │   (Smooth Spline)     │
└───────────┬───────────┘         └───────────────────────────┘         └───────────────────────┘
            │
            ├─────────────────────────────────────────────┐
            ▼                                             ▼
┌───────────────────────────┐               ┌───────────────────────────┐
│  High-Fidelity 3D Viewport│               │   Mission Control HUD     │
│(Animated Drone, Rays, Cam)│               │(Telemetry, Alerts, States)│
└───────────────────────────┘               └───────────────────────────┘
```

---

## 3. Mathematical Formulations

### 3.1 6-DOF Quadcopter Dynamics
The quadcopter is modeled as a rigid body with mass $m = 1.5\text{ kg}$ and gravitational acceleration $g = 9.81\text{ m/s}^2$. The translational equations of motion in the inertial world frame are:
$$\ddot{\mathbf{p}} = \frac{1}{m} \mathbf{R}_{wb} \begin{bmatrix} 0 \\ 0 \\ T \end{bmatrix} - \begin{bmatrix} 0 \\ 0 \\ g \end{bmatrix} + \frac{1}{m}\mathbf{F}_{drag}$$
where $\mathbf{R}_{wb}$ is the $Z$-$Y$-$X$ Tait-Bryan rotation matrix:
$$\mathbf{R}_{wb} = \mathbf{R}_z(\psi) \mathbf{R}_y(\theta) \mathbf{R}_x(\phi)$$
and $\mathbf{F}_{drag} = -c_{drag}(\mathbf{v} - \mathbf{w})$ accounts for aerodynamic resistance against ambient wind $\mathbf{w}$.

### 3.2 Dynamic Bank-to-Turn Aerodynamic Equilibrium
When accelerating horizontally with commanded acceleration $[a_{x, c}, a_{y, c}]$, the quadcopter must tilt into the airflow. In the yaw frame $\psi$:
$$\begin{bmatrix} a_{fwd} \\ a_{lat} \end{bmatrix} = \begin{bmatrix} \cos\psi & \sin\psi \\ -\sin\psi & \cos\psi \end{bmatrix} \begin{bmatrix} a_{x, c} \\ a_{y, c} \end{bmatrix}$$
The dynamically synthesized pitch ($\theta_d$) and roll ($\phi_d$) angles are:
$$\theta_d = \text{atan2}(a_{fwd}, g + a_{z, c}), \qquad \phi_d = \text{atan2}(-a_{lat}, g + a_{z, c})$$
Total thrust magnitude is scaled to maintain vertical equilibrium:
$$T = \frac{m(g + a_{z, c})}{\cos\phi_d \cos\theta_d}$$

### 3.3 Cascaded Flight Controller (Position PID)
Outer-loop trajectory tracking synthesizes accelerations:
$$\mathbf{a}_c = \mathbf{a}_d + \mathbf{K}_p (\mathbf{p}_d - \mathbf{p}) + \mathbf{K}_i \int (\mathbf{p}_d - \mathbf{p}) d\tau + \mathbf{K}_d (\mathbf{v}_d - \mathbf{v})$$
with anti-windup clamping on the integral accumulator.

### 3.4 3D Spatial Lattice Path Search
The global route is calculated over a 3D voxel grid. The cost function minimizes distance and changes in altitude:
$$f(n) = g(n) + h(n)$$
$$h(n) = \sqrt{(x_n - x_g)^2 + (y_n - y_g)^2 + (z_n - z_g)^2}$$
Obstacle inflation guarantees:
$$\text{Clearance Radius} \ge R_{drone} + R_{safety} = 0.40\text{ m} + 1.20\text{ m} = 1.60\text{ m}$$

### 3.5 Continuous $C^2$ Spline Trajectory Generation
Discretized grid paths are simplified using 3D line-of-sight ray marching and then parameterized into clamped cubic splines:
$$\mathbf{p}(t) = \mathbf{a}_i (t - t_i)^3 + \mathbf{b}_i (t - t_i)^2 + \mathbf{c}_i (t - t_i) + \mathbf{d}_i$$
yielding continuous velocity $\mathbf{v}(t)$ and acceleration $\mathbf{a}(t)$ with zero endpoint velocity.

---

## 4. Project Directory Structure

```
DroneSimulation/
├── main.m                      % Master simulation executable
├── config.m                    % Central system configurations
│
├── drone/                      % Quadcopter physics and 3D graphics
│   ├── DroneModel.m            % 6-DOF equations of motion & prop spin
│   ├── renderDrone3D.m         % 3D mesh model (arms, motors, props, LEDs)
│   └── updateDronePose.m       % Real-time hgtransform scene graph updates
│
├── environment/                % 3D proving ground & structures
│   ├── Environment.m           % Static and dynamic hazard manager
│   ├── createGround.m          % Ground tech grid and helipads
│   └── createObstacleGeometries.m % 3D buildings, towers, and meshes
│
├── sensors/                    % Multi-sensor perception suite
│   ├── LidarSensor.m           % 3D raycasting LiDAR scanner
│   ├── GpsSensor.m             % 3D GPS with noise and drift
│   ├── ImuSensor.m             % 3-axis Accelerometer & Gyroscope
│   └── AltimeterSensor.m       % Barometric/laser altitude sensor
│
├── mapping/                    % Spatial environment model
│   └── OccupancyMap3D.m        % 3D voxel grid with safety inflation
│
├── planning/                   % Autonomous route planning
│   ├── GlobalPlanner.m         % Encapsulated 3D spatial route engine
│   ├── PathSimplifier.m        % 3D Line-of-sight node pruning
│   └── TrajectoryGenerator.m   % C2 continuous smooth spline generation
│
├── control/                    % Flight control system
│   └── FlightController.m      % Cascaded position & attitude PID
│
├── collision/                  % Exact distance & clearance checking
│   └── CollisionChecker.m      % Continuous 3D analytical geometry clearance
│
├── scenarios/                  % Benchmark missions
│   └── ScenarioManager.m       % 6 Diverse test scenarios
│
├── visualization/              % Viewport & cinematography
│   ├── Visualizer3D.m          % 3D scene rendering, lighting, trails
│   └── CameraController.m      % Follow Drone, Top View, Overview, Free
│
├── dashboard/                  % Aerospace Mission Control HUD
│   └── MissionDashboard.m      % Real-time telemetry, alerts, camera switch
│
├── metrics/                    % Telemetry analytics & reporting
│   ├── MetricsCollector.m      % 50 Hz data logger & KPI computer
│   └── plotMissionResults.m    % 8-Panel post-flight analytical figure
│
├── tests/                      % Automated test suite
│   └── runTests.m              % Unit & integration test harness
│
└── README.md                   % Comprehensive documentation
```

---

## 5. Benchmark Scenarios

| Scenario ID | Name | Description | Key Research Feature |
| :--- | :--- | :--- | :--- |
| **`dynamic`** *(Default)* | Dynamic Moving Obstacle Avoidance | A moving transport vehicle crosses the drone's primary planned flight corridor mid-mission. | In-flight LiDAR detection, dynamic corridor blockage detection, real-time spatial replanning, and collision-free detour. |
| **`emergency`** | Sudden Pop-up Emergency Obstacle | A sudden hazard barrier pops up directly in front of the drone. | Immediate emergency braking, altitude hold, alternate route synthesis, and path resumption. |
| **`urban`** | Dense Urban Canyon | 10+ skyscrapers with narrow street corridors and tight tolerances. | Complex 3D spatial routing through urban canyons with strict clearance margins. |
| **`maze`** | 3D Spatial Complex Maze | Interlocking vertical and horizontal barriers. | 3D vertical and lateral waypoint optimization. |
| **`wind`** | Turbulent Crosswind Disturbance | Strong steady crosswinds ($4.5\text{ m/s}$) with random turbulence. | PID controller active disturbance rejection and trajectory hold. |
| **`open`** | Open Flight Arena Benchmark | Open proving ground with baseline obstacles. | Baseline performance benchmarking and speed calibration. |

---

## 6. How to Run

### 6.1 Launch Simulation
From the MATLAB Command Window:
```matlab
% Launch the Flagship Dynamic Obstacle Avoidance Demo:
main

% Or select a specific scenario:
main('dynamic')    % Flagship moving obstacle avoidance
main('emergency')  % Sudden pop-up barrier
main('urban')      % Skyscraper canyon navigation
main('maze')       % 3D spatial labyrinth
main('wind')       % Crosswind disturbance rejection
main('open')       % Open arena benchmark
```

### 6.2 Camera Controls During Flight
During simulation playback, switch camera views using the dashboard buttons:
- **[Follow Drone]**: Smooth third-person chase camera positioned behind the quadcopter.
- **[Top View]**: Overhead birds-eye tactical perspective.
- **[Overview]**: Wide isometric cinematic whole-arena view.
- **[Free Orbit]**: Interactive 3D mouse rotation and zoom.

### 6.3 Run Automated Test Suite
```matlab
runTests
```
Runs 12 comprehensive unit and integration test stages covering physics, sensors, planning, collision, replanning, and edge cases.

---

## 7. College Project & Viva Voce Defense Guide

### Q1: Why is A\* path planning encapsulated rather than displayed directly?
> **Answer**: In industrial aerospace systems (such as NASA UTM or commercial UAV ground control stations), mission control interfaces present high-level autonomous state awareness (*Safe Corridor Available, Dynamic Detour Active, Trajectory Tracking*). The discrete graph-search algorithm is an internal algorithmic engine; exposing search queues or tree nodes distracts from operational telemetry and real-time situational awareness.

### Q2: How does the system handle a dynamic moving obstacle in real time?
> **Answer**: Onboard 3D LiDAR continuously scans the environment at 15 Hz. The system forward-projects the drone's active trajectory over a 4.5-second horizon. If a detected moving obstacle intersects the safety-inflated flight corridor, the mission state machine transitions to `OBSTACLE DETECTED`. The drone halts forward acceleration, holds position/altitude, rebuilds the local 3D occupancy map with the new obstacle coordinates, synthesizes an alternate detour route, smoothly generates a new spline trajectory from its instantaneous velocity, and resumes navigation without pausing the simulation.

### Q3: How is realistic drone motion achieved without teleportation?
> **Answer**: The drone's position is governed by numerical integration of 6-DOF Newton-Euler rigid body dynamics. A cascaded PID controller commands horizontal accelerations that are mapped to physical bank-to-turn tilt angles ($\phi, \theta$) and collective thrust. The quadcopter naturally leans into turns, accelerates smoothly, rotates its heading along the flight path vector, and animates its propellers at rotational rates proportional to motor thrust.

### Q4: How is obstacle collision prevented mathematically?
> **Answer**: Collision prevention operates across two layers:
> 1. **Proactive Planning**: Obstacles in the 3D voxel grid are inflated by $R_{inflation} = R_{drone} + R_{safety} = 1.60\text{ m}$. The planner never generates waypoints within this buffer.
> 2. **Continuous Reactive Clearance**: Continuous analytical distance checks compute the Euclidean separation between the drone's bounding cylinder and all geometric boundaries. If clearance approaches zero, emergency braking and hovering are activated.

---

## 8. License
Developed for Academic Research, Autonomous Robotics Education, and Advanced UAV Engineering Demonstrations.
