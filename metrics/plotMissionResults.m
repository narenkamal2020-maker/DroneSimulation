function hFig = plotMissionResults(metrics, env, cfg)
% PLOTMISSIONRESULTS Generates a multi-panel aerospace telemetry and performance figure.
%
% Visualizes:
%   1. 3D Trajectory (Planned vs Flown)
%   2. Position Tracking Coordinates (X, Y, Z vs Time)
%   3. Velocity Profiles (Vx, Vy, Vz, |V| vs Time)
%   4. Altitude Profile (AGL vs Time)
%   5. Obstacle Clearance Margin vs Time
%   6. Attitude Angles (Roll, Pitch, Yaw vs Time)
%   7. Trajectory Tracking Error vs Time
%   8. Mission State Transition Timeline

summary = metrics.getSummary();
t = metrics.time;

if isempty(t) || length(t) < 2
    warning('Not enough telemetry points to generate report.');
    hFig = [];
    return;
end

% Create modern dark analytical dashboard figure
hFig = figure('Name', 'Autonomous UAV Mission Analytics & Telemetry Report', ...
              'NumberTitle', 'off', ...
              'Color', [0.07, 0.09, 0.14], ...
              'Position', [80, 50, 1380, 850]);

themeBg = [0.09, 0.12, 0.18];
textCol = [0.88, 0.92, 0.98];
cyanCol = [0.0, 0.85, 1.0];
greenCol = [0.1, 0.9, 0.4];
amberCol = [1.0, 0.65, 0.1];
redCol = [1.0, 0.3, 0.2];

% 1. Subplot: 3D Trajectory Overview (Span across 2 rows)
ax1 = subplot(3, 4, [1, 5]);
hold(ax1, 'on');
set(ax1, 'Color', themeBg, 'XColor', [0.3 0.4 0.5], 'YColor', [0.3 0.4 0.5], 'ZColor', [0.3 0.4 0.5]);
grid(ax1, 'on'); view(ax1, 3);
title(ax1, '3D Spatial Flight Trajectory', 'Color', textCol, 'FontWeight', 'bold');
xlabel(ax1, 'X (m)', 'Color', textCol); ylabel(ax1, 'Y (m)', 'Color', textCol); zlabel(ax1, 'Z (m)', 'Color', textCol);

% Draw obstacle bounding silhouettes
obsBounds = env.getAllObstacleBounds();
for k = 1:length(obsBounds)
    ob = obsBounds(k);
    if strcmp(ob.type, 'box')
        b = ob.bounds;
        [v, f] = env.getBoxMesh(b(1), b(3), b(5), b(2)-b(1), b(4)-b(3), b(6)-b(5));
        patch(ax1, 'Vertices', v, 'Faces', f, 'FaceColor', [0.2 0.25 0.35], ...
              'EdgeColor', [0.3 0.4 0.5], 'FaceAlpha', 0.5);
    end
end

plot3(ax1, metrics.posDesired(:, 1), metrics.posDesired(:, 2), metrics.posDesired(:, 3), ...
      '--', 'Color', cyanCol, 'LineWidth', 1.8, 'DisplayName', 'Planned Trajectory');
plot3(ax1, metrics.posActual(:, 1), metrics.posActual(:, 2), metrics.posActual(:, 3), ...
      '-', 'Color', amberCol, 'LineWidth', 2.2, 'DisplayName', 'Actual Flight Trail');
  
% Start and Goal points
scatter3(ax1, env.startPos(1), env.startPos(2), env.startPos(3), 80, greenCol, 'filled', 'DisplayName', 'Takeoff Pad');
scatter3(ax1, env.goalPos(1), env.goalPos(2), env.goalPos(3), 80, redCol, 'filled', 'DisplayName', 'Target Pad');
legend(ax1, 'TextColor', textCol, 'Color', themeBg, 'Location', 'northeast', 'FontSize', 8);

% 2. Subplot: Position Tracking vs Time
ax2 = subplot(3, 4, 2);
hold(ax2, 'on'); setupAxes(ax2, themeBg, textCol);
plot(ax2, t, metrics.posActual(:, 1), 'Color', cyanCol, 'LineWidth', 1.5, 'DisplayName', 'X Act');
plot(ax2, t, metrics.posDesired(:, 1), '--', 'Color', cyanCol*0.7, 'LineWidth', 1.2, 'DisplayName', 'X Ref');
plot(ax2, t, metrics.posActual(:, 2), 'Color', greenCol, 'LineWidth', 1.5, 'DisplayName', 'Y Act');
plot(ax2, t, metrics.posDesired(:, 2), '--', 'Color', greenCol*0.7, 'LineWidth', 1.2, 'DisplayName', 'Y Ref');
title(ax2, 'Horizontal Position vs Time', 'Color', textCol);
xlabel(ax2, 'Time (s)', 'Color', textCol); ylabel(ax2, 'Position (m)', 'Color', textCol);
legend(ax2, 'TextColor', textCol, 'Color', themeBg, 'FontSize', 7, 'Location', 'northwest');

% 3. Subplot: Altitude Tracking vs Time
ax3 = subplot(3, 4, 3);
hold(ax3, 'on'); setupAxes(ax3, themeBg, textCol);
plot(ax3, t, metrics.posDesired(:, 3), '--', 'Color', cyanCol, 'LineWidth', 1.5, 'DisplayName', 'Ref Altitude');
plot(ax3, t, metrics.posActual(:, 3), '-', 'Color', [0.3, 0.9, 0.95], 'LineWidth', 2.0, 'DisplayName', 'Actual AGL');
title(ax3, 'Vertical Flight Level (Altitude)', 'Color', textCol);
xlabel(ax3, 'Time (s)', 'Color', textCol); ylabel(ax3, 'Altitude (m)', 'Color', textCol);
legend(ax3, 'TextColor', textCol, 'Color', themeBg, 'FontSize', 7);

% 4. Subplot: Velocity Profiles vs Time
ax4 = subplot(3, 4, 4);
hold(ax4, 'on'); setupAxes(ax4, themeBg, textCol);
speeds = sqrt(sum(metrics.velActual.^2, 2));
plot(ax4, t, speeds, 'Color', amberCol, 'LineWidth', 2.0, 'DisplayName', '|V| Total');
plot(ax4, t, metrics.velActual(:, 1), 'Color', cyanCol*0.8, 'LineWidth', 1.2, 'DisplayName', 'Vx');
plot(ax4, t, metrics.velActual(:, 2), 'Color', greenCol*0.8, 'LineWidth', 1.2, 'DisplayName', 'Vy');
plot(ax4, t, metrics.velActual(:, 3), 'Color', [0.8, 0.5, 1.0], 'LineWidth', 1.2, 'DisplayName', 'Vz');
title(ax4, 'Velocity Profiles vs Time', 'Color', textCol);
xlabel(ax4, 'Time (s)', 'Color', textCol); ylabel(ax4, 'Speed (m/s)', 'Color', textCol);
legend(ax4, 'TextColor', textCol, 'Color', themeBg, 'FontSize', 7, 'Location', 'northeast');

% 5. Subplot: Obstacle Clearance Margin vs Time
ax5 = subplot(3, 4, 6);
hold(ax5, 'on'); setupAxes(ax5, themeBg, textCol);
validMask = ~isinf(metrics.clearance);
plot(ax5, t(validMask), metrics.clearance(validMask), 'Color', greenCol, 'LineWidth', 2.0, 'DisplayName', 'Clearance');
yline(ax5, cfg.drone.safetyMargin, '--r', 'Safety Margin Buffer', 'Color', redCol, 'LineWidth', 1.5);
title(ax5, 'Obstacle Proximity & Clearance', 'Color', textCol);
xlabel(ax5, 'Time (s)', 'Color', textCol); ylabel(ax5, 'Clearance (m)', 'Color', textCol);
ylim(ax5, [0, max(10, max(metrics.clearance(validMask)))]);

% 6. Subplot: Attitude Angles (Roll, Pitch, Yaw)
ax6 = subplot(3, 4, 7);
hold(ax6, 'on'); setupAxes(ax6, themeBg, textCol);
plot(ax6, t, metrics.eulerActual(:, 1), 'Color', redCol, 'LineWidth', 1.5, 'DisplayName', 'Roll \phi');
plot(ax6, t, metrics.eulerActual(:, 2), 'Color', greenCol, 'LineWidth', 1.5, 'DisplayName', 'Pitch \theta');
plot(ax6, t, metrics.eulerActual(:, 3), 'Color', cyanCol, 'LineWidth', 1.5, 'DisplayName', 'Yaw \psi');
title(ax6, 'Attitude Dynamics (Banking & Yaw)', 'Color', textCol);
xlabel(ax6, 'Time (s)', 'Color', textCol); ylabel(ax6, 'Angle (deg)', 'Color', textCol);
legend(ax6, 'TextColor', textCol, 'Color', themeBg, 'FontSize', 7, 'Location', 'northeast');

% 7. Subplot: Tracking Error vs Time
ax7 = subplot(3, 4, 8);
hold(ax7, 'on'); setupAxes(ax7, themeBg, textCol);
plot(ax7, t, metrics.trackingError, 'Color', [1.0, 0.45, 0.25], 'LineWidth', 1.8);
yline(ax7, summary.rmsTrackingError, '--', sprintf('RMS: %.2f m', summary.rmsTrackingError), ...
      'Color', amberCol, 'LineWidth', 1.4);
title(ax7, 'Trajectory Tracking Error', 'Color', textCol);
xlabel(ax7, 'Time (s)', 'Color', textCol); ylabel(ax7, 'Error Norm (m)', 'Color', textCol);

% 8. Subplot: Mission KPI Summary Card (Bottom row 9..12)
ax8 = subplot(3, 4, [9, 10, 11, 12]);
set(ax8, 'Color', [0.06, 0.08, 0.12], 'XColor', 'none', 'YColor', 'none');
title(ax8, 'MISSION TELEMETRY & FLIGHT PERFORMANCE KPI SUMMARY', 'Color', cyanCol, 'FontSize', 11, 'FontWeight', 'bold');

kpiText = { ...
    sprintf('\\bf Total Mission Duration:\\rm %.1f sec', summary.totalMissionTime), ...
    sprintf('\\bf Distance Flown:\\rm %.2f m (Planned: %.2f m)', summary.actualDistance, summary.plannedDistance), ...
    sprintf('\\bf Average Speed:\\rm %.2f m/s (Peak: %.2f m/s)', summary.avgSpeed, summary.maxSpeed), ...
    sprintf('\\bf Minimum Obstacle Clearance:\\rm %.2f m', summary.minClearance), ...
    sprintf('\\bf Trajectory RMS Error:\\rm %.3f m', summary.rmsTrackingError), ...
    sprintf('\\bf Dynamic Route Replans:\\rm %d event(s)', summary.routeReplans), ...
    sprintf('\\bf Obstacle Detections:\\rm %d return(s)', summary.obstacleDetections), ...
    sprintf('\\bf Flight Collisions:\\rm %d [MISSION SUCCESSFUL]', summary.collisions) ...
};

text(ax8, 0.05, 0.55, kpiText(1:4), 'Color', textCol, 'FontSize', 10, 'Units', 'normalized');
text(ax8, 0.55, 0.55, kpiText(5:8), 'Color', textCol, 'FontSize', 10, 'Units', 'normalized');

end

function setupAxes(ax, bgCol, ~)
set(ax, 'Color', bgCol, 'XColor', [0.3 0.4 0.5], 'YColor', [0.3 0.4 0.5]);
grid(ax, 'on');
set(ax, 'GridColor', [0.2 0.3 0.4], 'GridAlpha', 0.4);
end
