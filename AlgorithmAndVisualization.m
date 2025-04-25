% Multi UAV Obstacle Avoidance, Formation-Aware and Communication-Aware Path-finding Algorithm in a Known Environment
% This algorithm is for navigating $N$ UAVs  from point $A$ to $B$ in a 2D map with known environment, 
% while satisfying safety, formation, and communication-connectivity constraints. 
% The algorithm modifies the classical A* search to include swarm-specific metrics like 
% formation feasibility and algebraic connectivity.

% Authors: Asif Rizwan and Sampurak Talukdar
%
% ALGORTHIM VISUALIZATION
%___________________________________________________________________________________________________________________



clc;        % Clears the command window.
clear;      % Clears all variables from the workspace.
close all;  % Closes all open figures.
%% PARAMETERS
map_size = 100;     % Defines the size of the square map (100x100 units).
grid_size = 1;      % Defines the resolution of the map grid (1 unit per grid cell).
N = 6;              % Sets the number of UAVs in the swarm to 6.
r_min = 2;          % Sets the minimum allowed distance between any two UAVs (safety radius) to 2 units.
r_comm = 7;         % Sets the communication range between UAVs to 7 units.
goal_radius = 4;    % Sets the radius around the goal within which the swarm is considered to have arrived to 4 units.
D = 3;              % Sets the inter-UAV spacing parameter used in formation definitions to 3 units.
start = [10, 10];   % Defines the starting position [x, y] of the virtual leader (center of the formation) as [10, 10].
goal  = [90, 90];   % Defines the goal position [x, y] of the virtual leader as [90, 90].
%% Generate Formations for arbitrary N
% F1: Regular polygon (preferred formation)
theta = linspace(0, 2*pi, N+1); % Creates a vector of N+1 evenly spaced angles from 0 to 2*pi.
theta(end) = [];               % Removes the last angle from the 'theta' vector (to avoid the 0 and 2*pi being the same).
F1 = D * [cos(theta') sin(theta')]; % Calculates the x and y offsets for each UAV in a regular polygon formation of radius D.
% F2: Two-line formation (default)
F2 = zeros(N, 2);             % Initializes an N x 2 matrix 'F2' with zeros to store the UAV positions for the two-line formation.
mid = N/2;                   % Calculates the middle index for dividing the UAVs into two lines.
for i = 1:mid                 % Loop that iterates from 1 to the middle index.
    F2(i,:) = [-1.2*r_min * (i - 1), -D/2]; % Assigns the [x, y] coordinates for the i-th UAV in the first line.
end
for i = 1:mid                 % Loop that iterates from 1 to the middle index again.
    F2(mid + i,:) = [-1.2*r_min * (i - 1), D/2]; % Assigns the [x, y] coordinates for the i-th UAV in the second line.
end
% F3: Single line formation
F3 = -1.2*r_min * (0:N-1)';    % Calculates the x-coordinates for UAVs in a horizontal line with a spacing of 1.2*r_min.
F3 = [F3 zeros(N,1)];        % Appends a column of zeros to 'F3' to set the y-coordinates of all UAVs in the line to 0.
formations = {F1, F2, F3};   % Creates a cell array 'formations' containing the matrices defining the three formations.
formation_labels = ["F1", "F2", "F3"]; % Creates a string array 'formation_labels' providing names for each formation.
formation_priority = [5, 10, 20]; % Creates an array 'formation_priority' assigning a priority value to each formation (lower is preferred).
% Weights for heuristic function
w_safety = 75;               % Sets the weight for the safety component in the heuristic function to 75.
w_form = 15;                 % Sets the weight for the formation priority component in the heuristic function to 15.
w_conn = 9;                  % Sets the weight for the communication connectivity component in the heuristic function to 9.
w_dist = 1;                  % Sets the weight for the distance to the goal component in the heuristic function to 1.
%% OBSTACLES
obstacles = [... % [x, y, radius] entries
    65 87 10; 80 65 6; 40 60 3; 60 60 5; 45 45 6; 40 70 4;
    30 80 5; 30 50 5; 60 30 5; 90 60 4; 10 70 2; 60 40 4;
    20 70 4; 20 30 5; 30 45 4; 10 60 4; 70 75 5; 20 30 5;
    45 10 4; 40 20 4; 50 20 5]; % Defines a matrix 'obstacles' where each row represents an obstacle with [x-coordinate, y-coordinate, radius].
map = zeros(map_size);       % Initializes a 'map' matrix of size map_size x map_size with all elements set to 0 (representing free space).
[X, Y] = meshgrid(1:grid_size:map_size); % Creates coordinate matrices 'X' and 'Y' based on the map size and grid size.
for i = 1:size(obstacles,1)   % Loop that iterates through each defined obstacle.
    dist = sqrt((X - obstacles(i,1)).^2 + (Y - obstacles(i,2)).^2); % Calculates the Euclidean distance from each point in the grid to the center of the i-th obstacle.
    map(dist <= obstacles(i,3)+r_min) = 1; % Sets the corresponding cells in the 'map' to 1 if they are within the radius (obstacle radius + safety radius) of the i-th obstacle (representing an obstacle).
end
%% A* Initialization
open = [start 0 heuristic(start, goal) 0 1]; % Initializes the 'open' list with the starting node [x, y, g_cost, h_cost, f_cost, formation_id].
closed = [];                                % Initializes an empty 'closed' list to store visited nodes.
path_cost = containers.Map;                 % Initializes a 'path_cost' map to store the cost (g) to reach each node.
came_from = containers.Map;                 % Initializes a 'came_from' map to store the parent node in the path for each node.
formation_at_node = containers.Map;          % Initializes a 'formation_at_node' map to store the formation ID used at each node.
cost_map = inf(map_size);                   % Initializes a 'cost_map' matrix of size map_size x map_size with all elements set to infinity to store the total cost (f) to reach each grid cell.
switch_locations = [];                      % Initializes an empty array 'switch_locations' to store the coordinates where formation switches occur.
all_uav_trajs = {};                         % Initializes an empty cell array to store the trajectories of all UAVs along the found path.
%% A* MAIN LOOP
while ~isempty(open) % Continues the loop as long as there are nodes in the 'open' list to explore.
    [~, idx] = min(open(:,5)); % Finds the index of the node in 'open' with the lowest f_cost (5th column).
    current = open(idx,:);    % Retrieves the node with the lowest f_cost from 'open'.
    open(idx,:) = [];        % Removes the current node from the 'open' list.
    cx = current(1); cy = current(2); % Extracts the x and y coordinates of the current node.
    curr_key = key(cx,cy);    % Generates a unique string key for the current node's coordinates.
    if norm([cx cy] - goal) <= goal_radius % Checks if the current node is within the goal radius.
        path = reconstruct_path(came_from, curr_key); % If the goal is reached, reconstruct the path using the 'came_from' map.
        break;                                     % Exit the while loop as the path is found.
    end
    closed = [closed; cx cy]; % Adds the coordinates of the current node to the 'closed' list.
    for dx = -1:1              % Loop that iterates through possible changes in x-coordinate (-1, 0, 1).
        for dy = -1:1          % Loop that iterates through possible changes in y-coordinate (-1, 0, 1).
            if dx==0 && dy==0, continue; end % Skips the case where both dx and dy are 0 (the current node itself).
            nx = cx + dx; ny = cy + dy;     % Calculates the x and y coordinates of the neighbor node.
            if nx<1 || ny<1 || nx>map_size || ny>map_size, continue; end % Checks if the neighbor node is within the map boundaries.
            if map(ny,nx)==1 || ismember([nx ny], closed, 'rows'), continue; end % Checks if the neighbor node is an obstacle or has already been visited (is in the 'closed' list).
            for f_id = 1:length(formations) % Loop that iterates through each defined formation.
                f = formations{f_id};      % Gets the formation definition (coordinates relative to the leader).
                valid = true;              % Initializes a flag 'valid' to true, assuming the formation is initially valid.
                uav_pos = zeros(N,2);      % Initializes an N x 2 matrix 'uav_pos' to store the absolute positions of the UAVs in the current formation.
                for i = 1:N               % Loop that iterates through each UAV in the formation.
                    pt = [nx ny] + f(i,:); % Calculates the absolute position of the i-th UAV based on the neighbor node's position and the formation offset.
                    if any(pt<1) || any(pt>map_size) || map(round(pt(2)),round(pt(1)))==1 % Checks if any UAV in the formation is out of bounds or in an obstacle.
                        valid = false; break; % If any UAV is invalid, set 'valid' to false and break the inner loop (for UAVs).
                    end
                    uav_pos(i,:) = pt;     % Stores the calculated absolute position of the i-th UAV.
                end
                if ~valid || any(pdist(uav_pos)<r_min), continue; end % If the formation is not valid (due to bounds or obstacles) or if any two UAVs are closer than r_min, skip to the next formation.
                L = build_laplacian(uav_pos, r_comm); % Builds the Laplacian matrix for the communication graph of the UAVs in the current formation.
                eigsL = sort(eig(L));            % Calculates the eigenvalues of the Laplacian matrix and sorts them in ascending order.
                lambda2 = eigsL(2);             % Extracts the second smallest eigenvalue (algebraic connectivity).
                if lambda2 < 0.01, continue; end  % If the algebraic connectivity is below a threshold (poor connectivity), skip to the next formation.
                g = current(3) + norm([dx dy]);    % Calculates the cost (g) to reach the neighbor node from the start.
                h = w_form * formation_priority(f_id) + ... % Calculates the heuristic (h) cost based on formation priority,
                    w_conn * (1/lambda2) + ...           % communication connectivity (inverse of algebraic connectivity),
                    w_dist * heuristic([nx ny], goal);   % and the Euclidean distance to the goal.
                f_total = g + h;                % Calculates the total cost (f) to reach the neighbor node (f = g + h).
                next_key = key(nx, ny);        % Generates a unique string key for the neighbor node.
                if ~isKey(path_cost, next_key) || g < path_cost(next_key) % If the neighbor node has not been visited or if the current path to it is better than the previous one.
                    came_from(next_key) = curr_key;       % Update the parent node of the neighbor to the current node.
                    path_cost(next_key) = g;             % Update the cost to reach the neighbor node.
                    formation_at_node(next_key) = f_id;  % Store the formation ID used to reach the neighbor node.
                    open = [open; nx ny g h f_total f_id]; % Add the neighbor node to the 'open' list.
                    cost_map(ny,nx) = f_total;           % Store the total cost to reach the neighbor node in the 'cost_map'.
                    if isKey(formation_at_node, curr_key) && formation_at_node(curr_key) ~= f_id % Checks if the formation has switched from the previous node to the current neighbor.
                        switch_locations(end+1,:) = [nx, ny]; % If the formation switched, record the location of the switch.
                    end
                end
                break; % Breaks the inner loop (for formations) after finding a valid formation for the neighbor.
            end
        end
    end
end
%% RECONSTRUCT PATH (UPDATED)
function path = reconstruct_path(came_from, curr_key)
    % Reconstructs the path from the 'came_from' map, starting from the goal node.
    % came_from: Map containing the parent node for each node in the path.
    % curr_key: Key of the current node (initially the goal node).
    path = []; % Initializes an empty array to store the reconstructed path.
    while isKey(came_from, curr_key) % Continues as long as the current node has a parent in the 'came_from' map.
        coords = sscanf(curr_key, '%d_%d'); % Extracts the x and y coordinates from the current node's key.
        path = [coords'; path]; % Prepends the coordinates of the current node to the 'path'.
        curr_key = came_from(curr_key); % Moves to the parent node by updating 'curr_key'.
    end
    coords = sscanf(curr_key, '%d_%d'); % Extracts the coordinates of the starting node.
    path = [coords'; path]; % Prepends the coordinates of the starting node to the 'path'.
    path = reshape(path, [], 2); % Reshapes the 'path' array into a column vector of coordinate pairs (Nx2 matrix).
end
%% VISUALIZATION
figure('Name','Swarm A* with Formation, Paths, Heatmap'); % Creates a new figure with a specified title.
subplot(1,2,1); hold on; axis equal; grid on; % Creates the first subplot (1 row, 2 columns, first plot), enables hold for plotting multiple items, sets equal scaling for axes, and displays the grid.
imagesc(cost_map'); % Displays the transpose of the 'cost_map' as an image, where color intensity represents the cost.
% colormap(flipud(hot));
colorbar; % Adds a colorbar to the image, showing the mapping of colors to cost values.
colormap hot; % Sets the colormap to 'hot' (warmer colors indicate higher costs).
plot(start(1), start(2), 'go', 'MarkerSize', 3, 'LineWidth', 2); % Plots the starting position with a green circle.
plot(goal(1), goal(2), 'rx', 'MarkerSize', 3, 'LineWidth', 2); % Plots the goal position with a red cross.
% % Plot obstacles
% if ~isempty(obstacles)
%     for i = 1:size(obstacles,1)
%         viscircles(obstacles(i,1:2), obstacles(i,3)+r_min, 'Color','k','LineStyle','--','LineWidth',0.5);
%     end
% end
% Plot path and UAV formations
for i = 1:size(path,1) % Iterates through each point in the found path of the virtual leader.
    pos = path(i,:); % Gets the coordinates of the current point on the path.
    path_key = key(pos(1), pos(2)); % Generates the key for the current path point.
    if isKey(formation_at_node, path_key) % Checks if a formation ID is associated with the current path point.
        f_id = formation_at_node(path_key); % Retrieves the formation ID used at this point.
        f = formations{f_id}; % Gets the formation definition corresponding to the retrieved ID.
        uav_pos = zeros(N,2); % Initializes a matrix to store the absolute positions of the UAVs.
        valid = true; % Assume the formation placement is valid initially.
        for k = 1:N % Iterates through each UAV in the formation.
            pt = pos + f(k,:); % Calculates the absolute position of the k-th UAV.
            if any(pt < 1) || any(pt > map_size) % Checks if the UAV's position is within the map boundaries.
                valid = false; % If any UAV is out of bounds, mark the placement as invalid.
                break; % Exit the inner loop.
            end
            uav_pos(k,:) = pt; % Store the calculated UAV position.
             end
        if valid
            all_uav_trajs{i} = uav_pos;
            plot(uav_pos(:,1), uav_pos(:,2), 'bo-', 'MarkerSize', 5, 'LineWidth', 1.5);
        else
            all_uav_trajs{i} = []; % Placeholder to avoid indexing issues
        end
    else
        all_uav_trajs{i} = []; % Placeholder for missing formation info
    end
end

% % Highlight switch locations
% if ~isempty(switch_locations)
%     scatter(switch_locations(:,1), switch_locations(:,2), 60, 'm', 'filled');
% end
xlim([0 map_size]); ylim([0 map_size]);

legend('Start','Goal','Obstacles','UAV Path','Switch Points');
title('Final Path with Formations');

% Animation
subplot(1,2,2); axis([0 map_size 0 map_size]); axis equal; grid on; hold on;
title('Multi-UAV Trajectory Animation');
xlim([0 map_size]); ylim([0 map_size]);

for t = 1:length(all_uav_trajs)
    cla;
    % Obstacles
theta = linspace(0, 2*pi, 50);                    % Generate points for drawing circles
for i = 1:size(obstacles,1)                    % Iterate through obstacles
    fill(obstacles(i,1)+obstacles(i,3)*cos(theta), ... % Draw filled circle for each obstacle
         obstacles(i,2)+obstacles(i,3)*sin(theta), ...
         'r','FaceAlpha',0.3);
end
    pos = all_uav_trajs{t};
    if isempty(pos) || size(pos,2) ~= 2
        continue;
    end
    plot(pos(:,1), pos(:,2), 'b.-','MarkerSize',20);
    drawnow;
    pause(0.05);
end


% Left subplot: Map with obstacles, start/goal, path, and formation switches
subplot(1,2,1); hold on; axis equal; grid on;

title('Swarm Path with Formations');
xlabel('X'); ylabel('Y');

% xlim([0 map_size]); ylim([0 map_size]);

% % Plot obstacles
% for i = 1:size(obstacles,1)
%     viscircles(obstacles(i,1:2), obstacles(i,3)+r_min, 'Color','r','LineWidth',1);
% end

% Plot start and goal
plot(start(1), start(2), 'go', 'MarkerSize', 8, 'LineWidth', 2);
plot(goal(1), goal(2), 'rx', 'MarkerSize', 8, 'LineWidth', 2);

% % Plot path
% if exist('path','var')
%     plot(path(:,1), path(:,2), 'b-', 'LineWidth', 2);
% end

% Plot formation switch points
% if ~isempty(switch_locations)
%     scatter(switch_locations(:,1), switch_locations(:,2), 50, 'm', 'filled');
%     legend('Obstacles','Start','Goal','Path','Formation Switch');
% 
% else
%     legend('Obstacles','Start','Goal','Path');
% end

% Right subplot: Heatmap
subplot(1,2,2);

title('Heatmap of Total Cost');
xlabel('X'); ylabel('Y');
xlim([0 map_size]); ylim([0 map_size]);

% Generate coordinates
[X, Y] = meshgrid(1:map_size, 1:map_size);
xlim([0 map_size]); ylim([0 map_size]);

contourf(X, Y, cost_map, 10, 'LineColor','none');
colorbar;
axis equal tight;

trail = [];
for t = 1:length(all_uav_trajs)
    cla;
    xlim([0 map_size]); ylim([0 map_size]);
% Obstacles
theta = linspace(0, 2*pi, 50);                    % Generate points for drawing circles
for i = 1:size(obstacles,1)                    % Iterate through obstacles
    fill(obstacles(i,1)+obstacles(i,3)*cos(theta), ... % Draw filled circle for each obstacle
         obstacles(i,2)+obstacles(i,3)*sin(theta), ...
         'r','FaceAlpha',0.3);
end
    pos = all_uav_trajs{t};
    if isempty(pos) || size(pos,2) ~= 2
        continue;
    end
    plot(pos(:,1), pos(:,2), 'b.-','MarkerSize',20);
    drawnow;
    pause(0.01);
end

%% FUNCTIONS
% This section defines the custom functions used in the main A* search algorithm.

function h = heuristic(p, goal)
    % Define a function named 'heuristic' that takes two arguments:
    %   p: The current position (a [1x2] array representing [x, y] coordinates).
    %   goal: The goal position (a [1x2] array representing [x, y] coordinates).
    %
    % This function calculates an estimate of the cost from the current position 'p'
    % to the 'goal' position. In this case, it uses the Euclidean distance, which
    % is a common and admissible heuristic for pathfinding on a grid.

    h = norm(p - goal); % Calculates the Euclidean distance between the current position 'p'
                         % and the 'goal' position using the 'norm' function. The result
                         % is stored in the variable 'h'. This 'h' value serves as the
                         % heuristic estimate in the A* algorithm.
end

function L = build_laplacian(pos, r_comm)
    % Define a function named 'build_laplacian' that takes two arguments:
    %   pos: A matrix of UAV positions (an Nx2 array, where N is the number of UAVs,
    %        and each row represents the [x, y] coordinates of a UAV).
    %   r_comm: The communication range between the UAVs (a scalar value).
    %
    % This function constructs the Laplacian matrix of the communication graph
    % formed by the UAV swarm. The Laplacian matrix is used to analyze the
    % connectivity of the graph.

    N = size(pos,1);           % Gets the number of UAVs (the number of rows in the 'pos' matrix)
                                % and stores it in the variable 'N'.
    A = zeros(N);             % Initializes an NxN matrix 'A' with all elements set to zero.
                                % This matrix will represent the adjacency matrix of the communication graph.
    for i = 1:N                % Outer loop that iterates through each UAV (from 1 to N).
        for j = i+1:N          % Inner loop that iterates through the remaining UAVs (from i+1 to N)
                                % to avoid redundant checks (since communication is bidirectional) and
                                % self-loops.
            if norm(pos(i,:) - pos(j,:)) <= r_comm % Calculates the Euclidean distance between the i-th and j-th UAVs
                                                % using their positions from the 'pos' matrix. If this distance
                                                % is less than or equal to the communication range 'r_comm'.
                A(i,j) = 1;      % Sets the element (i, j) of the adjacency matrix 'A' to 1, indicating
                                % that there is a communication link between the i-th and j-th UAVs.
                A(j,i) = 1;      % Since communication is bidirectional, it also sets the element (j, i)
                                % of 'A' to 1.
            end
        end
    end
    L = diag(sum(A,2)) - A; % Calculates the Laplacian matrix 'L'. The Laplacian matrix is defined as
                                % L = D - A, where 'D' is the degree matrix (a diagonal matrix where
                                % the diagonal elements are the degree of each node, i.e., the number
                                % of connections each UAV has). 'sum(A,2)' calculates the degree of each
                                % UAV (sum of each row in the adjacency matrix), and 'diag()' creates
                                % the degree matrix 'D'. Then, the adjacency matrix 'A' is subtracted
                                % from 'D' to obtain the Laplacian matrix 'L'.
end

function s = key(x, y)
    % Define a function named 'key' that takes two arguments:
    %   x: The x-coordinate of a grid node (a scalar value).
    %   y: The y-coordinate of a grid node (a scalar value).
    %
    % This function generates a unique string key for a given (x, y) coordinate pair.
    % This key is used to store and retrieve information associated with grid nodes
    % in data structures like MATLAB's 'containers.Map'.

    s = sprintf('%d_%d', x, y); % Uses the 'sprintf' function to create a string 's' by concatenating
                                % the integer values of 'x' and 'y' separated by an underscore '_'.
                                % This string serves as a unique identifier or key for the grid node
                                % with coordinates (x, y).
end