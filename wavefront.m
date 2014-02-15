function [value_map, trajectory] = wavefront(map, start_row, start_column)
%WAVEFRONT Planner algorithm to compute the optimal path towards the goal
%   Uses 8-point connectivity.

    MAP_GOAL_VALUE = 2; % GOAL VALUE SET TO: 2

    value_map = buildValueMap(map, MAP_GOAL_VALUE);
    
    trajectory = buildTrajectory(value_map, start_row, start_column, MAP_GOAL_VALUE);

end


%%% BUILDING WAVE FRONT MAP FUNCTIONS

function value_map = buildValueMap(map, goalValue)

    [goalX, goalY] = findValueMapGoalPosition(map, goalValue);

    import java.util.ArrayDeque
    neighborList = ArrayDeque();

    % initiating 8-neighbor list
    map = add8NeighborToList(neighborList, goalX, goalY, goalValue, map);
    
    while ~neighborList.isEmpty()
        neighborData = neighborList.pop(); % or poll(), removeFirst(), remove(), pollFirst()
        % neighborData(1): x coordinate
        % neighborData(2): y coordinate
        % neighborData(3): value of cell who added this cell as neighbor
        map = add8NeighborToList(neighborList, neighborData(1), neighborData(2), neighborData(3), map);
    end
    
    value_map = map;
    
end

function [goalX, goalY] = findValueMapGoalPosition(map, goalValue)

    [goalX, goalY] = find(map == goalValue);
    
    if isempty(goalX) == 1 || isempty(goalY) == 1
        error(['No goal index found in map with value: ', num2str(goalValue)]);
    end
    
    goalX = goalX(1); % We ignore other goal positions if there is multiple exist
    goalY = goalY(1); % We ignore other goal positions if there is multiple exist
    
end

function [changedMap] = add4NeighborToList(neighborList, x, y, currentValue, map)
    
    %left neighbor
    map = addNeighborToList(neighborList, x - 1, y, currentValue, map);
    
    %top neighbor
    map = addNeighborToList(neighborList, x, y + 1, currentValue, map);
    
    %right neighbor
    map = addNeighborToList(neighborList, x + 1, y, currentValue, map);
    
    %bottom neighbor
    map = addNeighborToList(neighborList, x, y - 1, currentValue, map);
    
    changedMap = map;
    
end

function [changedMap] = add8NeighborToList(neighborList, x, y, currentValue, map)

    map = add4NeighborToList(neighborList, x, y, currentValue, map);

    % left top neighbor
    map = addNeighborToList(neighborList, x - 1, y + 1, currentValue, map);
    
    % right top neighbor
    map = addNeighborToList(neighborList, x + 1, y + 1, currentValue, map);
    
    % right bottom neighbor
    map = addNeighborToList(neighborList, x + 1, y - 1, currentValue, map);
    
    % left bottom neighbor
    map = addNeighborToList(neighborList, x - 1, y - 1, currentValue, map);
    
    changedMap = map;
    
end

function [changedMap] = addNeighborToList(neighborList, neighborX, neighborY, currentValue, map)

    changedMap = map;

    [mapWidth, mapHeight] = size(changedMap);

    if neighborX < 1 || neighborX > mapWidth % If x exceed size of map, we don't add to list
        return;
    end

    if neighborY < 1 || neighborY > mapHeight % If y exceed size of map, we don't add to list
        return;
    end

    if changedMap(neighborX, neighborY) ~= 0 % If value is not equal to 0, we don't add to neighbor list
        return;
    end

    changedMap(neighborX, neighborY) = currentValue + 1;
    neighborList.add([neighborX neighborY currentValue + 1]);

end

%%% BUILDING TRAJECTORY FUNCTIONS

function [trajectory] = buildTrajectory(value_map, start_row, start_column, goalValue)
    
    [goalX, goalY] = findValueMapGoalPosition(value_map, goalValue);
    
    trajectory = [start_row start_column];
    
    
    
end


