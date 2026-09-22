function handles = createGround(ax, cfg, startPos, goalPos)
% CREATEGROUND Renders a modern high-tech ground plane, grid, and helipads.
%
% Inputs:
%   ax       - MATLAB Axes handle
%   cfg      - System configuration struct
%   startPos - [x, y, z] start position
%   goalPos  - [x, y, z] destination position
%
% Outputs:
%   handles  - Struct containing graphical object handles

hold(ax, 'on');

bx = cfg.map.boundsX;
by = cfg.map.boundsY;

% 1. Dark matte base ground surface
handles.groundPatch = patch(ax, 'XData', [bx(1) bx(2) bx(2) bx(1)], ...
                                'YData', [by(1) by(1) by(2) by(2)], ...
                                'ZData', [0 0 0 0], ...
                                'FaceColor', cfg.theme.groundColor, ...
                                'EdgeColor', 'none', ...
                                'FaceAlpha', 0.98);

% 2. High-tech coordinate grid overlay
step = 5.0;
xGrid = bx(1):step:bx(2);
yGrid = by(1):step:by(2);

% Lines parallel to Y
for i = 1:length(xGrid)
    xg = xGrid(i);
    alpha = 0.25;
    lw = 0.75;
    if mod(xg, 20) == 0
        alpha = 0.55;
        lw = 1.2;
    end
    lineCol = cfg.theme.groundColor * (1 - alpha) + cfg.theme.gridColor * alpha;
    line(ax, [xg, xg], [by(1), by(2)], [0.01, 0.01], ...
         'Color', lineCol, 'LineWidth', lw);
end

% Lines parallel to X
for i = 1:length(yGrid)
    yg = yGrid(i);
    alpha = 0.25;
    lw = 0.75;
    if mod(yg, 20) == 0
        alpha = 0.55;
        lw = 1.2;
    end
    lineCol = cfg.theme.groundColor * (1 - alpha) + cfg.theme.gridColor * alpha;
    line(ax, [bx(1), bx(2)], [yg, yg], [0.01, 0.01], ...
         'Color', lineCol, 'LineWidth', lw);
end

% 3. Arena boundary perimeter frame
bX = [bx(1), bx(2), bx(2), bx(1), bx(1)];
bY = [by(1), by(1), by(2), by(2), by(1)];
bZ = [0.05, 0.05, 0.05, 0.05, 0.05];
plot3(ax, bX, bY, bZ, 'Color', [0.3, 0.5, 0.7], 'LineWidth', 2.0);

% Corner boundary beacons
corners = [bx(1), by(1); bx(2), by(1); bx(2), by(2); bx(1), by(2)];
for k = 1:4
    cx = corners(k, 1);
    cy = corners(k, 2);
    plot3(ax, [cx cx], [cy cy], [0 4], 'Color', [0.2 0.8 1.0], 'LineWidth', 2.5);
    scatter3(ax, cx, cy, 4.2, 50, [0.0 0.9 1.0], 'filled', 'MarkerEdgeColor', 'w');
end

% 4. Helipad at Start (Emerald Green Theme)
handles.startPad = renderHelipad(ax, startPos(1), startPos(2), 2.5, [0.1, 0.8, 0.3], 'START / HOME');

% 5. Helipad at Goal (Cyan / Amber Gold Theme)
handles.goalPad = renderHelipad(ax, goalPos(1), goalPos(2), 2.5, [0.0, 0.85, 1.0], 'DESTINATION');

end

function padHandles = renderHelipad(ax, x0, y0, radius, color, labelText)
% Helper to render a circular helipad with concentric rings and a central 'H'
theta = linspace(0, 2*pi, 48);

% Outer dark foundation disc
xOuter = x0 + (radius + 0.4) * cos(theta);
yOuter = y0 + (radius + 0.4) * sin(theta);
zOuter = 0.02 * ones(size(theta));
padHandles.disc = patch(ax, 'XData', xOuter, 'YData', yOuter, 'ZData', zOuter, ...
                        'FaceColor', [0.12, 0.16, 0.22], 'EdgeColor', color, 'LineWidth', 1.8);

% Inner concentric ring
xInner = x0 + (radius * 0.7) * cos(theta);
yInner = y0 + (radius * 0.7) * sin(theta);
zInner = 0.03 * ones(size(theta));
padHandles.ring = plot3(ax, xInner, yInner, zInner, 'Color', color, 'LineWidth', 1.5);

% Center 'H' symbol
hSize = radius * 0.45;
% Left vertical bar
padHandles.h1 = plot3(ax, [x0 - hSize/2, x0 - hSize/2], [y0 - hSize, y0 + hSize], [0.04, 0.04], ...
                      'Color', color, 'LineWidth', 3.0);
% Right vertical bar
padHandles.h2 = plot3(ax, [x0 + hSize/2, x0 + hSize/2], [y0 - hSize, y0 + hSize], [0.04, 0.04], ...
                      'Color', color, 'LineWidth', 3.0);
% Center bar
padHandles.h3 = plot3(ax, [x0 - hSize/2, x0 + hSize/2], [y0, y0], [0.04, 0.04], ...
                      'Color', color, 'LineWidth', 3.0);

% Pulsing beacon / label
text(ax, x0, y0, 0.8, labelText, 'Color', color, 'FontSize', 9, ...
     'FontWeight', 'bold', 'HorizontalAlignment', 'center', ...
     'BackgroundColor', [0.05, 0.07, 0.10], 'Margin', 2);
end
