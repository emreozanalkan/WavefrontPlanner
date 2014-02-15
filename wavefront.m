function [value_map, trajectory] = wavefront(map, start_row, start_column)
%WAVEFRONT Planner algorithm to compute the optimal path towards the goal
%   Uses 8-point connectivity.

    MAP_GOAL_VALUE = 2; % GOAL VALUE SET TO: 2

    value_map = buildValueMap(map, MAP_GOAL_VALUE);
    
    trajectory = buildTrajectory(value_map, start_row, start_column, MAP_GOAL_VALUE);

end

%%% BUILD WAVEFRONT VALUE MAP FUNCTIONS

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
    
    % left neighbor
    map = addNeighborToList(neighborList, x - 1, y, currentValue, map);
    
    % top neighbor
    map = addNeighborToList(neighborList, x, y + 1, currentValue, map);
    
    % right neighbor
    map = addNeighborToList(neighborList, x + 1, y, currentValue, map);
    
    % bottom neighbor
    map = addNeighborToList(neighborList, x, y - 1, currentValue, map);
    
    changedMap = map;
    
end

function [changedMap] = add8NeighborToList(neighborList, x, y, currentValue, map)

    map = add4NeighborToList(neighborList, x, y, currentValue, map);

    % top left neighbor
    map = addNeighborToList(neighborList, x - 1, y + 1, currentValue, map);
    
    % top right neighbor
    map = addNeighborToList(neighborList, x + 1, y + 1, currentValue, map);
    
    % bottom right neighbor
    map = addNeighborToList(neighborList, x + 1, y - 1, currentValue, map);
    
    % bottom left neighbor
    map = addNeighborToList(neighborList, x - 1, y - 1, currentValue, map);
    
    changedMap = map;
    
end

function [changedMap] = addNeighborToList(neighborList, neighborX, neighborY, currentValue, map)

    changedMap = map;

    [mapWidth, mapHeight] = size(changedMap);

    if neighborX < 1 || neighborX > mapWidth || neighborY < 1 || neighborY > mapHeight  % If x or y exceed size of map, we don't add to list
        return;
    end

    if changedMap(neighborX, neighborY) ~= 0 % If value is not equal to 0, we don't add to neighbor list
        return;
    end

    changedMap(neighborX, neighborY) = currentValue + 1;
    neighborList.add([neighborX neighborY currentValue + 1]);

end

%%% BUILD TRAJECTORY FUNCTIONS

function [trajectory] = buildTrajectory(value_map, start_row, start_column, goalValue)
    
    [goalX, goalY] = findValueMapGoalPosition(value_map, goalValue);
    
    robotDirection = 0; % Robot's Current Momentum Direction STRAIGHT: 0, DIAGONAL: 1
    
    trajectory = [start_row start_column]; % sortrows(matrix, column);
    
    neighborList = getAvailable8NeighborList(value_map, start_row, start_column); % init starting
    
    while ~isReachedGoal(neighborList, goalValue)
        
        [neighborX, neighborY, robotDirection] = pickNextOptimalNeighbor(neighborList, robotDirection);
        
        trajectory = [trajectory; [neighborX, neighborY]];
        
        neighborList = getAvailable8NeighborList(value_map, neighborX, neighborY);
        
    end
    
    trajectory = finalizeTrajectoryWithGoal(trajectory, neighborList);
    
end

function [isNeighborAvailableAndOptimal] = isNeighborAvailableAndOptimal(value_map, x, y, currentValue)

    isNeighborAvailableAndOptimal = 0;

    [mapWidth, mapHeight] = size(value_map);
    
    if x < 1 || x > mapWidth || y < 1 || y > mapHeight % If x or y exceed size of map
        return;
    end
    
    neighborValue = value_map(x, y);
    
    if neighborValue < 2 || neighborValue >= currentValue
        return;
    end
    
    isNeighborAvailableAndOptimal = 1;
    
end

function [neighborData] = getNeighborData(value_map, x, y, robotDirection)
    
    neighborValue = value_map(x, y);
    
    neighborData = [x y neighborValue robotDirection];
    
end

function [neighborList] = getAvailable4NeighborList(value_map, x, y)

    [mapWidth, mapHeight] = size(value_map);
    
    if x < 1 || x > mapWidth || y < 1 || y > mapHeight
        neighborList = double.empty;
        return;
    end

	neighborList = double.empty(0, 0);

    currentValue = value_map(x, y);
    
    % left neighbor
    if isNeighborAvailableAndOptimal(value_map, x - 1, y, currentValue)
        neighborList = [neighborList; getNeighborData(value_map, x - 1, y, 0)]; % STRAIGHT: 0, DIAGONAL: 1
    end
        
    % top neighbor
    if isNeighborAvailableAndOptimal(value_map, x, y + 1, currentValue)
        neighborList = [neighborList; getNeighborData(value_map, x, y + 1, 0)]; % STRAIGHT: 0, DIAGONAL: 1
    end
    
    % right neighbor
    if isNeighborAvailableAndOptimal(value_map, x + 1, y, currentValue)
        neighborList = [neighborList; getNeighborData(value_map, x + 1, y, 0)]; % STRAIGHT: 0, DIAGONAL: 1
    end
    
    % bottom neighbor
    if isNeighborAvailableAndOptimal(value_map, x, y - 1, currentValue)
        neighborList = [neighborList; getNeighborData(value_map, x, y - 1, 0)]; % STRAIGHT: 0, DIAGONAL: 1
    end
    
end

function [neighborList] = getAvailable8NeighborList(value_map, x, y)

    [mapWidth, mapHeight] = size(value_map);
    
    if x < 1 || x > mapWidth || y < 1 || y > mapHeight
        neighborList = double.empty;
        return;
    end

    neighborList = getAvailable4NeighborList(value_map, x, y);
    
    currentValue = value_map(x, y);
    
    % top left neighbor
    if isNeighborAvailableAndOptimal(value_map, x - 1, y + 1, currentValue)
        neighborList = [neighborList; getNeighborData(value_map, x - 1, y + 1, 1)]; % STRAIGHT: 0, DIAGONAL: 1
    end
    
    % top right neighbor
    if isNeighborAvailableAndOptimal(value_map, x + 1, y + 1, currentValue)
        neighborList = [neighborList; getNeighborData(value_map, x + 1, y + 1, 1)]; % STRAIGHT: 0, DIAGONAL: 1
    end
    
    % bottom right neighbor
    if isNeighborAvailableAndOptimal(value_map, x + 1, y - 1, currentValue)
        neighborList = [neighborList; getNeighborData(value_map, x + 1, y - 1, 1)]; % STRAIGHT: 0, DIAGONAL: 1
    end
    
    % bottom left neighbor
    if isNeighborAvailableAndOptimal(value_map, x - 1, y - 1, currentValue)
        neighborList = [neighborList; getNeighborData(value_map, x - 1, y - 1, 1)]; % STRAIGHT: 0, DIAGONAL: 1
    end

end

function [isReachedGoal] = isReachedGoal(neighborList, goalValue)
    
    neighborListSorted = sortrows(neighborList, 3);
    
    isReachedGoal = (neighborListSorted(1, 3) == goalValue);
    
end

function [trajectory] = finalizeTrajectoryWithGoal(trajectory, neighborList)

    neighborListSorted = sortrows(neighborList, 3);
    
    trajectory = [trajectory; [neighborListSorted(1, 1) neighborListSorted(1, 2)]];
    
end

function [neighborX, neighborY, robotDirection] = pickNextOptimalNeighbor(neighborList, robotDirection)

    neighborX = 0;
    neighborY = 0;
    robotDirection = 0;
    
end




