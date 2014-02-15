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


%%% BUILD TRAJECTORY FUNCTIONS

function [trajectory] = buildTrajectory(value_map, start_row, start_column, goalValue)
    
    [goalX, goalY] = findValueMapGoalPosition(value_map, goalValue);
    
    robotCurrentMomentumDirection = 0; % STRAIGHT: 0, DIAGONAL: 1
    
    trajectory = [start_row start_column]; % sortrows(matrix, column);
    
    neighborList = getAvailable8NeighborList(value_map, start_row, start_column); % init starting
    
    if isReachedGoal(neighborList, goalValue)
        trajectory = finalizeTrajectory(trajectory, neighborList);
        return;
    end
    
    
    
    
    
    
end

function [neighborList] = getAvailable4NeighborList(value_map, x, y)

	neighborList = double.empty(0, 0);

    currentValue = value_map(x, y);
    
    %left neighbor
    leftNeighbor = {x - 1, y};
    leftNeighborValue = value_map(leftNeighbor{1}, leftNeighbor{2});
    
    if leftNeighborValue > 1 && leftNeighborValue < currentValue
        neighborList = [neighborList; [leftNeighbor{1} leftNeighbor{2} leftNeighborValue 0]]; % STRAIGHT: 0, DIAGONAL: 1
    end
    
    %top neighbor
    topNeighbor = {x, y + 1};
    topNeighborValue = value_map(topNeighbor{1}, topNeighbor{2});
    
    if topNeighborValue > 1 && topNeighborValue < currentValue
        neighborList = [neighborList; [topNeighbor{1} topNeighbor{2} topNeighborValue 0]]; % STRAIGHT: 0, DIAGONAL: 1
    end
    
    %right neighbor
    rightNeighbor = {x + 1, y};
    rightNeighborValue = value_map(rightNeighbor{1}, rightNeighbor{2});
    
    if rightNeighborValue > 1 && rightNeighborValue < currentValue
        neighborList = [neighborList; [rightNeighbor{1} rightNeighbor{2} rightNeighborValue 0]]; % STRAIGHT: 0, DIAGONAL: 1
    end
    
    %bottom neighbor
    bottomNeighbor = {x + 1, y};
    bottomNeighborValue = value_map(bottomNeighbor{1}, bottomNeighbor{2});
    
    if bottomNeighborValue > 1 && bottomNeighborValue < currentValue
        neighborList = [neighborList; [bottomNeighbor{1} bottomNeighbor{2} bottomNeighborValue 0]]; % STRAIGHT: 0, DIAGONAL: 1
    end    
    
end

function [neighborList] = getAvailable8NeighborList(value_map, x, y)

    neighborList = getAvailable4NeighborList(value_map, x, y);
    
    currentValue = value_map(x, y);
    
    % top left neighbor
    topLeftNeighbor = {x - 1, y + 1};
    topLeftNeighborValue = value_map(topLeftNeighbor{1}, topLeftNeighbor{2});
    
    if topLeftNeighborValue > 1 && topLeftNeighborValue < currentValue
        neighborList = [neighborList; [topLeftNeighbor{1} topLeftNeighbor{2} topLeftNeighborValue 1]]; % STRAIGHT: 0, DIAGONAL: 1
    end
    
    % top right neighbor
    topRightNeighbor = {x + 1, y + 1};
    topRightNeighborValue = value_map(topRightNeighbor{1}, topRightNeighbor{2});
    
    if topRightNeighborValue > 1 && topRightNeighborValue < currentValue
        neighborList = [neighborList; [topRightNeighbor{1} topRightNeighbor{2} topRightNeighborValue 1]]; % STRAIGHT: 0, DIAGONAL: 1
    end
    
    % bottom right neighbor
    bottomRightNeighbor = {x + 1, y - 1};
    bottomRightNeighborValue = value_map(bottomRightNeighbor{1}, bottomRightNeighbor{2});
    
    if bottomRightNeighborValue > 1 && bottomRightNeighborValue < currentValue
        neighborList = [neighborList; [bottomRightNeighbor{1} bottomRightNeighbor{2} bottomRightNeighborValue 1]]; % STRAIGHT: 0, DIAGONAL: 1
    end
    
    % bottom left neighbor
    bottomLeftNeighbor = {x - 1, y - 1};
    bottomLeftNeighborValue = value_map(bottomLeftNeighbor{1}, bottomLeftNeighbor{2});
    
    if bottomLeftNeighborValue > 1 && bottomLeftNeighborValue < currentValue
        neighborList = [neighborList; [bottomLeftNeighbor{1} bottomLeftNeighbor{2} bottomLeftNeighborValue 1]]; % STRAIGHT: 0, DIAGONAL: 1
    end

end

function [isReachedGoal] = isReachedGoal(neighborList, goalValue)
    isReachedGoal = 0; % 0: FALSE, 1: TRUE
    
    neighborListSorted = sortrows(neighborList, 3);
    
    
end

function [trajectory] = finalizeTrajectory(trajectory, neighborList)
    trajectory = zeros;
end





















