% Multi UAV Obstacle Avoidance, Formation-Aware and Communication-Aware Path-finding Algorithm in a Known Environment
% This algorithm is for navigating $N$ UAVs  from point $A$ to $B$ in a 2D map with known environment, 
% while satisfying safety, formation, and communication-connectivity constraints. 
% The algorithm modifies the classical A* search to include swarm-specific metrics like 
% formation feasibility and algebraic connectivity.

% Authors: Asif Rizwan and Sampurak Talukdar
%
% ALGORTHIM 
%___________________________________________________________________________________________________________________



clc; clear; close all;
%% PARAMETERS
map_size = 100;            % Size of the square map (100x100)
grid_size = 1;             % Resolution of the map grid (not directly used in A*, but good for conceptualization)
N = 6;                     % Number of UAVs in the swarm
r_min = 2;                 % Minimum allowed distance between UAVs (safety radius)
r_comm = 10;                % Communication range between UAVs
goal_radius = 4;           % Radius around the goal where UAVs are considered arrived
D = 3;                     % Inter-UAV spacing in the formation
start = [10, 10];          % Start position [x, y] of the virtual leader (center of formation)
goal  = [90, 90];          % Goal position [x, y] of the virtual leader
%% Generate Formations for arbitrary N
% F1: Regular polygon (preferred formation)
theta = linspace(0, 2*pi, N+1); % Evenly spaced angles around a circle
theta(end) = [];               % Remove the last angle (which is equal to 2*pi)
F1 = D * [cos(theta') sin(theta')]; % Calculate x, y offsets for each UAV in the polygon
% F2: Two-line formation
F2 = zeros(N, 2);             % Initialize N x 2 matrix for UAV positions
mid = floor(N / 2);           % Find the middle index
for i = 1:mid
    F2(i,:) = [-D * (i - 1), -D/2]; % Positions for the first line of UAVs
end
for i = 1:(N - mid)
    F2(mid + i,:) = [-D * (i - 1), D/2]; % Positions for the second line of UAVs
end
if mod(N, 2) == 1
    F2(end,:) = [D, 0];      % If N is odd, push the last UAV ahead
end
% F3: Single line formation
F3 = -D * (0:N-1)';          % Calculate x-positions for UAVs in a line
F3 = [F3 zeros(N,1)];        % Append zeros for y-positions (all in a horizontal line)
formations = {F1, F2, F3};    % Cell array containing the formation matrices
formation_labels = ["F1", "F2", "F3"]; % String labels for the formations
formation_priority = [0, 10, 20];    % Priority of formations (lower is preferred). F1 is preferred.
% Weights for heuristic function
w_safety = 50;               % Weight for safety (obstacle and inter-UAV avoidance) - Not directly used in this simplified A*
w_form = 30;                 % Weight for formation priority
w_conn = 15;                 % Weight for communication connectivity
w_dist = 5;                  % Weight for distance to goal
%% OBSTACLES
obstacles = [65 87 10; 80 65 6; 40 60 3; 60 60 1; 67 60 1; 45 45 6; 40 70 4;
             30 80 5; 30 50 5; 60 30 5; 90 60 4; 10 70 2; 60 40 4;
             20 70 4; 20 30 5; 30 45 4; 10 60 4; 70 75 5; 20 30 5;
             45 10 4; 40 20 4; 50 20 5]; % [x, y, radius] for each obstacle
map = zeros(map_size);        % Initialize the map as a 2D matrix of zeros
[X, Y] = meshgrid(1:grid_size:map_size); % Create meshgrid for map coordinates
for i = 1:size(obstacles,1)    % Iterate through each obstacle
    dist = sqrt((X - obstacles(i,1)).^2 + (Y - obstacles(i,2)).^2); % Calculate distance from each point to obstacle center
    map(dist <= obstacles(i,3)+r_min) = 1;           % Set map cells within obstacle radius to 1 (obstacle)
end
%% A* Initialization
open = [start 0 heuristic(start, goal) 0 1]; % [x y g h f formID].  g=cost from start, h=heuristic, f=g+h, formID=formation ID
closed = [];                                 % List of visited nodes
path_cost = containers.Map;                  % Map to store the cost (g) to reach each node
came_from = containers.Map;                  % Map to store the parent node in the path
formation_at_node = containers.Map;           % Map to store the formation ID used at each node
cost_map = inf(map_size);                    % Map to store the total cost (f) of reaching each node (for visualization)
switch_locations = [];                       % Array to store the locations where formation switches occur
%% A* MAIN LOOP
while ~isempty(open)
    [~, idx] = min(open(:,5));       % Find the node in 'open' with the lowest f cost
    current = open(idx,:);          % Get the node with the lowest f cost
    open(idx,:) = [];               % Remove the current node from 'open'
    cx = current(1); cy = current(2);   % Current node coordinates
    curr_key = key(cx,cy);          % Generate unique key for current node
    if norm([cx cy] - goal) <= goal_radius % Check if we've reached the goal
        path = reconstruct_path(came_from, curr_key); % Reconstruct the path from 'came_from'
        break;                                      % Exit the loop
    end
    closed = [closed; cx cy];       % Add the current node to 'closed'
    for dx = -1:1                   % Iterate through possible neighbor offsets
        for dy = -1:1
            if dx==0 && dy==0, continue; end % Skip the current node itself
            nx = cx + dx; ny = cy + dy;     % Calculate neighbor coordinates
            if nx<1 || ny<1 || nx>map_size || ny>map_size, continue; end % Check if neighbor is within map bounds
            if map(ny,nx)==1 || ismember([nx ny], closed, 'rows'), continue; end % Check if neighbor is an obstacle or already visited
            for f_id = 1:3           % Iterate through each formation
                f = formations{f_id};    % Get the formation definition
                valid = true;           % Assume the formation is valid initially
                uav_pos = zeros(N,2);    % Initialize UAV positions
                for i = 1:N           % Iterate through each UAV in the formation
                    pt = [nx ny] + f(i,:); % Calculate UAV position relative to the neighbor
                    if any(pt<1) || any(pt>map_size) || map(round(pt(2)),round(pt(1)))==1 % Check bounds and obstacles
                        valid = false; break; % If any UAV is out of bounds or in obstacle, formation is invalid
                    end
                    uav_pos(i,:) = pt;     % Store the UAV position
                end
                if ~valid || any(pdist(uav_pos)<r_min), continue; end % Skip if formation is invalid or UAVs too close
                L = build_laplacian(uav_pos, r_comm); % Build Laplacian matrix for connectivity check
                eigsL = sort(eig(L));            % Get eigenvalues of Laplacian
                lambda2 = eigsL(2);             % Algebraic connectivity (second smallest eigenvalue)
                if lambda2 < 0.01, continue; end  % Skip if connectivity is poor
                g = current(3) + norm([dx dy]);    % Calculate cost (g) to reach the neighbor
                h = w_form * formation_priority(f_id) + ... % Heuristic (h) calculation
                    w_conn * (1/lambda2) + ...
                    w_dist * heuristic([nx ny], goal);
                f_total = g + h;                % Total cost (f)
                next_key = key(nx, ny);        % Generate key for neighbor
                if ~isKey(path_cost, next_key) || g < path_cost(next_key) % If neighbor not in 'path_cost' or new path is better
                    came_from(next_key) = curr_key;       % Update parent node
                    path_cost(next_key) = g;             % Update cost to reach neighbor
                    formation_at_node(next_key) = f_id;  % Store the formation ID used
                    open = [open; nx ny g h f_total f_id]; % Add neighbor to 'open'
                    cost_map(ny,nx) = f_total;           % Store total cost for visualization
                    % Log switch location
                    if isKey(formation_at_node, curr_key) && formation_at_node(curr_key) ~= f_id
                        switch_locations(end+1,:) = [nx, ny]; % Store location
                    end
                end
                break; % Break after finding a valid formation
            end
        end
    end
end
%% VISUALIZATION
figure('Name','Swarm A* with Formation, Paths, Heatmap'); % Create a figure
subplot(1,2,1); hold on; axis equal; grid on;       % Create subplot for paths
xlim([0 map_size]); ylim([0 map_size]);           % Set axis limits
xlabel('X'); ylabel('Y'); title('Swarm Path with Formation Colors'); % Set labels and title
% Obstacles
theta = linspace(0, 2*pi, 50);                    % Generate points for drawing circles
for i = 1:size(obstacles,1)                    % Iterate through obstacles
    fill(obstacles(i,1)+obstacles(i,3)*cos(theta), ... % Draw filled circle for each obstacle
         obstacles(i,2)+obstacles(i,3)*sin(theta), ...
         'r','FaceAlpha',0.3);
end
% UAV Paths
colors = lines(N);                                 % Get distinct colors for each UAV
last_fid = -1;
uav_paths = cell(N,1); for i = 1:N, uav_paths{i} = []; end % Initialize cell array to store UAV paths
for k = 1:size(path,1)                         % Iterate through each node in the path
    pos = path(k,:);                           % Get the node coordinates
    k_str = key(pos(1), pos(2));               % Generate key for the node
    if isKey(formation_at_node, k_str)          % Get formation ID at this node
        f_id = formation_at_node(k_str);
    else
        f_id = 1;                                 % Default to formation 1 if not found
    end
    f = formations{f_id};                      % Get formation definition
    color_switch = f_id ~= last_fid;          % Check if formation changed
    last_fid = f_id;
    for i = 1:N                             % Iterate through each UAV
        pt = pos + f(i,:);                     % Calculate UAV position
        uav_paths{i}(end+1,:) = pt;           % Append to UAV's path
        c = colors(i,:);                       % Get UAV color
        if color_switch, c = [1 0 0]; end      % If formation switch, make UAV color red
        plot(pt(1), pt(2), 'o', 'MarkerFaceColor', c, 'MarkerEdgeColor', 'k', 'MarkerSize', 6); % Plot UAV position
    end
    pause(0.01);                               % Pause for visualization
end
% Continuous UAV trails
for i = 1:N
    plot(uav_paths{i}(:,1), uav_paths{i}(:,2), '-', 'Color', colors(i,:), 'LineWidth', 1.2); % Plot the path
end
% Start and Goal
plot(start(1), start(2), 'gs', 'MarkerSize', 10, 'LineWidth', 2); % Plot start
plot(goal(1), goal(2), 'kx', 'MarkerSize', 10, 'LineWidth', 2);  % Plot goal
% Formation Switch Locations
% if ~isempty(switch_locations)
%     plot(switch_locations(:,1), switch_locations(:,2), 'rs', 'MarkerSize', 10, 'LineWidth', 1.5); % Plot switch points
%     legend('UAVs', 'Switch Point');
% end
%% COST HEATMAP
subplot(1,2,2);                             % Create subplot for cost heatmap
imagesc(cost_map); colorbar;                 % Display the cost map as an image with a colorbar
title('Total Cost Heatmap (f = g + h)');    % Set title
xlabel('X'); ylabel('Y');                     % Set labels
set(gca,'YDir','normal');                   % Correct the y-axis direction
colormap hot;                                 % Use a heatmap color scheme
%% FUNCTIONS
function h = heuristic(p, goal)
    % Calculate the Euclidean distance between a point p and the goal.
    h = norm(p - goal);
end
function L = build_laplacian(pos, r_comm)
    % Build the Laplacian matrix representing the communication graph of the UAV swarm.
    % pos: N x 2 matrix of UAV positions
    % r_comm: Communication range
    N = size(pos,1);           % Number of UAVs
    A = zeros(N);             % Initialize adjacency matrix
    for i = 1:N
        for j = i+1:N
            if norm(pos(i,:) - pos(j,:)) <= r_comm % Check if distance is within communication range
                A(i,j) = 1;  % If within range, there is a connection.
                A(j,i) = 1;  % The communication link is bidirectional
            end
        end
    end
    L = diag(sum(A,2)) - A; % Calculate Laplacian matrix: L = D - A, where D is the degree matrix
end
function path = reconstruct_path(came_from, key_curr)
    % Reconstruct the path from the 'came_from' map, starting from the goal node.
    % came_from: Map containing parent nodes
    % key_curr: Key of the current node (starting from the goal)
    path = [];
    while isKey(came_from, key_curr)       % While the current node has a parent
        xy = sscanf(key_curr, '%d_%d');   % Extract x, y coordinates from the key
        path = [xy'; path];             % Prepend the current node to the path (building it backward)
        key_curr = came_from(key_curr);     % Get the key of the parent node
    end
    path = [sscanf(key_curr, '%d_%d')'; path]; % Add the start node to the path
end
function s = key(x, y)
    % Generate a unique string key for a node (x, y) coordinate.
    % Used for storing nodes in maps.
    s = sprintf('%d_%d', x, y);
end
