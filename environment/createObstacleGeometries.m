function [obsHandles, obsBounds] = createObstacleGeometries(ax, cfg, obstacles)
% CREATEOBSTACLEGEOMETRIES Renders 3D buildings, towers, and structures.
%
% Inputs:
%   ax        - Target axes
%   cfg       - System configuration
%   obstacles - Array of obstacle definition structs:
%                 type: 'building', 'cylinder', 'tower'
%                 params: [x, y, z, dx, dy, dz] or [x, y, z, radius, height]
%                 color: [r, g, b]
%
% Outputs:
%   obsHandles - Array of graphic handles for rendered obstacles
%   obsBounds  - Struct array with geometry data for collision and sensor checks

obsHandles = [];
obsBounds = struct('type', {}, 'bounds', {}, 'center', {}, 'radius', {}, 'height', {}, 'color', {});

numObs = length(obstacles);
for k = 1:numObs
    obs = obstacles(k);
    
    switch lower(obs.type)
        case 'building'
            % Box obstacle: params = [xMin, yMin, zMin, dx, dy, dz]
            p = obs.params;
            x0 = p(1); y0 = p(2); z0 = p(3);
            dx = p(4); dy = p(5); dz = p(6);
            
            % Define 8 vertices of cuboid
            vertices = [
                x0,      y0,      z0;
                x0 + dx, y0,      z0;
                x0 + dx, y0 + dy, z0;
                x0,      y0 + dy, z0;
                x0,      y0,      z0 + dz;
                x0 + dx, y0,      z0 + dz;
                x0 + dx, y0 + dy, z0 + dz;
                x0,      y0 + dy, z0 + dz
            ];
            
            % 6 faces (counter-clockwise)
            faces = [
                1 2 6 5; % Front (+Y facing outward or -Y)
                2 3 7 6; % Right
                3 4 8 7; % Back
                4 1 5 8; % Left
                5 6 7 8; % Top
                1 4 3 2  % Bottom
            ];
            
            % Wall and roof coloring
            wallCol = obs.color;
            roofCol = wallCol * 0.7;
            
            % Only render graphics if a valid axes handle is supplied
            if ~isempty(ax) && isvalid(ax)
                % Render main building body
                hBody = patch(ax, 'Vertices', vertices, 'Faces', faces(1:4, :), ...
                              'FaceColor', wallCol, 'EdgeColor', [0.15 0.20 0.28], ...
                              'LineWidth', 1.0, 'FaceAlpha', 0.95);
                
                % Render roof
                hRoof = patch(ax, 'Vertices', vertices, 'Faces', faces(5, :), ...
                              'FaceColor', roofCol, 'EdgeColor', [0.2 0.8 1.0], ...
                              'LineWidth', 1.2, 'FaceAlpha', 0.98);
                
                % Add subtle illuminated architectural window lines
                hWindows = renderBuildingWindows(ax, x0, y0, z0, dx, dy, dz, cfg.theme.windowGlow);
                
                obsHandles = [obsHandles; hBody; hRoof; hWindows]; %#ok<AGROW>
            end
            
            % Store analytical bounding box: [xMin, xMax, yMin, yMax, zMin, zMax]
            obsBounds(k).type = 'box';
            obsBounds(k).bounds = [x0, x0 + dx, y0, y0 + dy, z0, z0 + dz];
            obsBounds(k).center = [x0 + dx/2, y0 + dy/2, z0 + dz/2];
            obsBounds(k).radius = sqrt((dx/2)^2 + (dy/2)^2);
            obsBounds(k).height = dz;
            obsBounds(k).color = wallCol;
            
        case 'cylinder'
            % Cylindrical tower: params = [centerX, centerY, baseZ, radius, height]
            p = obs.params;
            cx = p(1); cy = p(2); cz = p(3);
            r = p(4); h = p(5);
            
            nSides = 24;
            [Xcyl, Ycyl, Zcyl] = cylinder(r, nSides);
            Xcyl = Xcyl + cx;
            Ycyl = Ycyl + cy;
            Zcyl = Zcyl * h + cz;
            
            if ~isempty(ax) && isvalid(ax)
                hSide = surf(ax, Xcyl, Ycyl, Zcyl, 'FaceColor', obs.color, ...
                             'EdgeColor', [0.15 0.22 0.30], 'FaceAlpha', 0.92);
                
                % Top cap
                theta = linspace(0, 2*pi, nSides + 1);
                xCap = cx + r * cos(theta);
                yCap = cy + r * sin(theta);
                zCap = (cz + h) * ones(size(theta));
                hCap = patch(ax, 'XData', xCap, 'YData', yCap, 'ZData', zCap, ...
                             'FaceColor', obs.color * 0.75, 'EdgeColor', [0.2 0.8 1.0], ...
                             'LineWidth', 1.2);
                
                obsHandles = [obsHandles; hSide; hCap]; %#ok<AGROW>
            end
            
            obsBounds(k).type = 'cylinder';
            obsBounds(k).bounds = [cx - r, cx + r, cy - r, cy + r, cz, cz + h];
            obsBounds(k).center = [cx, cy, cz + h/2];
            obsBounds(k).radius = r;
            obsBounds(k).height = h;
            obsBounds(k).color = obs.color;
    end
end

end

function hLines = renderBuildingWindows(ax, x0, y0, z0, dx, dy, dz, glowColor)
% Adds illuminated architectural horizontal bands to buildings
hLines = [];
floorHeight = 3.0;
numFloors = floor(dz / floorHeight);

if numFloors < 2
    return;
end

zLevels = (z0 + floorHeight):floorHeight:(z0 + dz - 1.0);
for i = 1:length(zLevels)
    zl = zLevels(i);
    % Front facade band
    hl1 = line(ax, [x0 + 0.5, x0 + dx - 0.5], [y0 - 0.02, y0 - 0.02], [zl, zl], ...
               'Color', [glowColor, 0.45], 'LineWidth', 1.2);
    % Right facade band
    hl2 = line(ax, [x0 + dx + 0.02, x0 + dx + 0.02], [y0 + 0.5, y0 + dy - 0.5], [zl, zl], ...
               'Color', [glowColor, 0.45], 'LineWidth', 1.2);
    % Back facade band
    hl3 = line(ax, [x0 + 0.5, x0 + dx - 0.5], [y0 + dy + 0.02, y0 + dy + 0.02], [zl, zl], ...
               'Color', [glowColor, 0.45], 'LineWidth', 1.2);
    % Left facade band
    hl4 = line(ax, [x0 - 0.02, x0 - 0.02], [y0 + 0.5, y0 + dy - 0.5], [zl, zl], ...
               'Color', [glowColor, 0.45], 'LineWidth', 1.2);
    hLines = [hLines; hl1; hl2; hl3; hl4]; %#ok<AGROW>
end
end
