classdef PathSimplifier < handle
    % PATHSIMPLIFIER Prunes redundant nodes using 3D Line-of-Sight String Pulling.
    
    methods (Static)
        function simplified = simplify(rawWaypoints, occMap)
            % Reduces dense grid waypoints to key turning waypoints
            %
            % Inputs:
            %   rawWaypoints - [N x 3] Array of dense spatial waypoints
            %   occMap       - OccupancyMap3D instance with safety inflation
            %
            % Output:
            %   simplified   - [M x 3] Minimal essential waypoint sequence (M << N)
            
            N = size(rawWaypoints, 1);
            if N <= 2
                simplified = rawWaypoints;
                return;
            end
            
            simplified = rawWaypoints(1, :);
            currIdx = 1;
            
            while currIdx < N
                furthestIdx = currIdx + 1;
                
                % Probe backwards from end to find furthest reachable node
                for testIdx = N:-1:(currIdx + 1)
                    p1 = rawWaypoints(currIdx, :);
                    p2 = rawWaypoints(testIdx, :);
                    
                    if occMap.checkLineOfSight(p1, p2, 0.4)
                        furthestIdx = testIdx;
                        break;
                    end
                end
                
                simplified = [simplified; rawWaypoints(furthestIdx, :)]; %#ok<AGROW>
                currIdx = furthestIdx;
            end
        end
    end
end
