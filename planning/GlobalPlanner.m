classdef GlobalPlanner < handle
    % GLOBALPLANNER High-performance 3D Spatial Route Calculation Engine.
    %
    % Encapsulated 3D discrete grid path planning using Euclidean heuristic,
    % 26-connected spatial lattice, and binary min-heap priority queue.
    
    properties
        cfg
        connectivity = 26
        moves3D
        moveCosts
    end
    
    methods
        function obj = GlobalPlanner(cfg)
            obj.cfg = cfg;
            
            % Generate 26 neighbor moves in 3D (excluding [0, 0, 0])
            [dx, dy, dz] = ndgrid(-1:1, -1:1, -1:1);
            allMoves = [dx(:), dy(:), dz(:)];
            zeroIdx = (allMoves(:,1) == 0 & allMoves(:,2) == 0 & allMoves(:,3) == 0);
            obj.moves3D = allMoves(~zeroIdx, :);
            
            % Precompute Euclidean transition step costs
            obj.moveCosts = sqrt(sum(obj.moves3D.^2, 2));
        end
        
        function [route, success, planMetrics] = planRoute(obj, startPos, goalPos, occMap)
            % Computes an optimal collision-free 3D route from start to goal
            tStartTime = tic;
            
            % Map world coordinates to discrete grid indices
            [sx, sy, sz] = occMap.posToGrid(startPos);
            [gx, gy, gz] = occMap.posToGrid(goalPos);
            
            % If goal is blocked in map, search nearby free voxel
            if occMap.isGridBlocked(gx, gy, gz)
                [gx, gy, gz] = obj.findNearestFreeGrid(gx, gy, gz, occMap);
            end
            
            % If start is blocked, search nearby free voxel
            if occMap.isGridBlocked(sx, sy, sz)
                [sx, sy, sz] = obj.findNearestFreeGrid(sx, sy, sz, occMap);
            end
            
            startIdx = sub2ind([occMap.nx, occMap.ny, occMap.nz], sx, sy, sz);
            goalIdx  = sub2ind([occMap.nx, occMap.ny, occMap.nz], gx, gy, gz);
            
            if startIdx == goalIdx
                route = [startPos; goalPos];
                success = true;
                planMetrics.calcTime = toc(tStartTime);
                planMetrics.pathLength = norm(goalPos - startPos);
                planMetrics.nodesExpanded = 1;
                return;
            end
            
            totalNodes = occMap.nx * occMap.ny * occMap.nz;
            
            % Cost matrices
            gCost = inf(totalNodes, 1, 'single');
            parent = zeros(totalNodes, 1, 'int32');
            closedSet = false(totalNodes, 1);
            
            % Goal coordinates for heuristic
            goalCoord = [gx, gy, gz];
            
            % Initialize start
            gCost(startIdx) = 0;
            hStart = norm([sx, sy, sz] - goalCoord);
            
            % Binary Min-Heap preallocated arrays
            maxHeapCapacity = min(200000, totalNodes);
            heapNodes = zeros(maxHeapCapacity, 1, 'int32');
            heapKeys  = zeros(maxHeapCapacity, 1, 'single');
            heapCount = 1;
            
            heapNodes(1) = startIdx;
            heapKeys(1)  = hStart;
            
            found = false;
            iterations = 0;
            maxIterations = 150000;
            
            dimX = occMap.nx;
            dimY = occMap.ny;
            dimZ = occMap.nz;
            moves = obj.moves3D;
            numMoves = size(moves, 1);
            costs = obj.moveCosts;
            currIdx = startIdx;
            
            while heapCount > 0 && iterations < maxIterations
                iterations = iterations + 1;
                
                % Pop minimum element from binary heap
                currIdx = heapNodes(1);
                
                % Sift down replacement
                heapNodes(1) = heapNodes(heapCount);
                heapKeys(1)  = heapKeys(heapCount);
                heapCount = heapCount - 1;
                
                % Heapify down
                hIdx = 1;
                while true
                    left = bitshift(hIdx, 1);
                    right = left + 1;
                    smallest = hIdx;
                    
                    if left <= heapCount && heapKeys(left) < heapKeys(smallest)
                        smallest = left;
                    end
                    if right <= heapCount && heapKeys(right) < heapKeys(smallest)
                        smallest = right;
                    end
                    
                    if smallest ~= hIdx
                        % Swap parent with smallest child
                        tN = heapNodes(hIdx); tK = heapKeys(hIdx);
                        heapNodes(hIdx) = heapNodes(smallest); heapKeys(hIdx) = heapKeys(smallest);
                        heapNodes(smallest) = tN; heapKeys(smallest) = tK;
                        hIdx = smallest;
                    else
                        break;
                    end
                end
                
                % Lazy deletion: skip if already settled
                if closedSet(currIdx)
                    continue;
                end
                closedSet(currIdx) = true;
                
                if currIdx == goalIdx
                    found = true;
                    break;
                end
                
                % Convert 1D index to 3D grid subscripts
                [cx, cy, cz] = ind2sub([dimX, dimY, dimZ], currIdx);
                currG = gCost(currIdx);
                
                % Evaluate all 26 spatial neighbors
                for m = 1:numMoves
                    nx = cx + moves(m, 1);
                    ny = cy + moves(m, 2);
                    nz = cz + moves(m, 3);
                    
                    % Boundary check
                    if nx < 1 || nx > dimX || ny < 1 || ny > dimY || nz < 1 || nz > dimZ
                        continue;
                    end
                    
                    % Collision / occupancy check
                    if occMap.grid(nx, ny, nz) == 1
                        continue;
                    end
                    
                    neighborIdx = sub2ind([dimX, dimY, dimZ], nx, ny, nz);
                    if closedSet(neighborIdx)
                        continue;
                    end
                    
                    % Edge cost + slight vertical bias for stable flight level
                    edgeDist = costs(m);
                    vertPenalty = 0.15 * abs(moves(m, 3));
                    tentativeG = currG + edgeDist + vertPenalty;
                    
                    if tentativeG < gCost(neighborIdx)
                        parent(neighborIdx) = currIdx;
                        gCost(neighborIdx) = tentativeG;
                        
                        % Euclidean heuristic
                        hVal = norm([nx, ny, nz] - goalCoord);
                        fVal = tentativeG + hVal;
                        
                        % Push to binary heap
                        if heapCount < maxHeapCapacity
                            heapCount = heapCount + 1;
                            sIdx = heapCount;
                            heapNodes(sIdx) = neighborIdx;
                            heapKeys(sIdx)  = fVal;
                            
                            % Sift up
                            while sIdx > 1
                                pIdx = bitshift(sIdx, -1);
                                if heapKeys(sIdx) < heapKeys(pIdx)
                                    tN = heapNodes(pIdx); tK = heapKeys(pIdx);
                                    heapNodes(pIdx) = heapNodes(sIdx); heapKeys(pIdx) = heapKeys(sIdx);
                                    heapNodes(sIdx) = tN; heapKeys(sIdx) = tK;
                                    sIdx = pIdx;
                                else
                                    break;
                                end
                            end
                        end
                    end
                end
            end
            
            calcTime = toc(tStartTime);
            
            if found
                % Reconstruct route
                pathIndices = currIdx;
                while pathIndices(1) ~= startIdx
                    pNode = parent(pathIndices(1));
                    if pNode == 0
                        break;
                    end
                    pathIndices = [pNode; pathIndices]; %#ok<AGROW>
                end
                
                numPts = length(pathIndices);
                rawWaypoints = zeros(numPts, 3);
                for k = 1:numPts
                    [px, py, pz] = ind2sub([dimX, dimY, dimZ], pathIndices(k));
                    rawWaypoints(k, :) = occMap.gridToPos([px, py, pz]);
                end
                
                % Replace endpoints with exact continuous positions
                rawWaypoints(1, :) = startPos;
                rawWaypoints(end, :) = goalPos;
                
                route = rawWaypoints;
                success = true;
                
                % Calculate metric path length
                diffs = diff(route, 1, 1);
                planMetrics.pathLength = sum(sqrt(sum(diffs.^2, 2)));
                planMetrics.calcTime = calcTime;
                planMetrics.nodesExpanded = iterations;
            else
                % Direct fallback if unreachable
                route = [startPos; goalPos];
                success = false;
                planMetrics.pathLength = norm(goalPos - startPos);
                planMetrics.calcTime = calcTime;
                planMetrics.nodesExpanded = iterations;
            end
        end
        
        function [fx, fy, fz] = findNearestFreeGrid(~, gx, gy, gz, occMap)
            % Breadth-first search to find nearest free voxel if goal/start is blocked
            fx = gx; fy = gy; fz = gz;
            for r = 1:10
                for dx = -r:r
                    for dy = -r:r
                        for dz = -r:r
                            nx = gx + dx; ny = gy + dy; nz = gz + dz;
                            if nx >= 1 && nx <= occMap.nx && ny >= 1 && ny <= occMap.ny && nz >= 1 && nz <= occMap.nz
                                if occMap.grid(nx, ny, nz) == 0
                                    fx = nx; fy = ny; fz = nz;
                                    return;
                                end
                            end
                        end
                    end
                end
            end
        end
    end
end
