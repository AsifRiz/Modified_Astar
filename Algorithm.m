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
map_size = 100;
grid_size = 1;
N = 6;
r_min = 2;
r_comm = 10;
goal_radius = 4;
D = 3;
start = [10, 10];
goal  = [90, 90];
theta = linspace(0, 2*pi, N+1);
theta(end) = [];
F1 = D * [cos(theta') sin(theta')];
F2 = zeros(N, 2);
mid = floor(N / 2);
for i = 1:mid
    F2(i,:) = [-D * (i - 1), -D/2];
end
for i = 1:(N - mid)
    F2(mid + i,:) = [-D * (i - 1), D/2];
end
if mod(N, 2) == 1
    F2(end,:) = [D, 0];
end
F3 = -D * (0:N-1)';
F3 = [F3 zeros(N,1)];
formations = {F1, F2, F3};
formation_labels = ["F1", "F2", "F3"];
formation_priority = [0, 10, 20];
w_safety = 50;
w_form = 30;
w_conn = 15;
w_dist = 5;
obstacles = [65 87 10; 80 65 6; 40 60 3; 60 60 1; 67 60 1; 45 45 6; 40 70 4;
             30 80 5; 30 50 5; 60 30 5; 90 60 4; 10 70 2; 60 40 4;
             20 70 4; 20 30 5; 30 45 4; 10 60 4; 70 75 5; 20 30 5;
             45 10 4; 40 20 4; 50 20 5];
map = zeros(map_size);
[X, Y] = meshgrid(1:grid_size:map_size);
for i = 1:size(obstacles,1)
    dist = sqrt((X - obstacles(i,1)).^2 + (Y - obstacles(i,2)).^2);
    map(dist <= obstacles(i,3)+r_min) = 1;
end
open = [start 0 heuristic(start, goal) 0 1];
closed = [];
path_cost = containers.Map;
came_from = containers.Map;
formation_at_node = containers.Map;
cost_map = inf(map_size);
switch_locations = [];
while ~isempty(open)
    [~, idx] = min(open(:,5));
    current = open(idx,:);
    open(idx,:) = [];
    cx = current(1); cy = current(2);
    curr_key = key(cx,cy);
    if norm([cx cy] - goal) <= goal_radius
        path = reconstruct_path(came_from, curr_key);
        break;
    end
    closed = [closed; cx cy];
    for dx = -1:1
        for dy = -1:1
            if dx==0 && dy==0, continue; end
            nx = cx + dx; ny = cy + dy;
            if nx<1 || ny<1 || nx>map_size || ny>map_size, continue; end
            if map(ny,nx)==1 || ismember([nx ny], closed, 'rows'), continue; end
            for f_id = 1:3
                f = formations{f_id};
                valid = true;
                uav_pos = zeros(N,2);
                for i = 1:N
                    pt = [nx ny] + f(i,:);
                    if any(pt<1) || any(pt>map_size) || map(round(pt(2)),round(pt(1)))==1
                        valid = false; break;
                    end
                    uav_pos(i,:) = pt;
                end
                if ~valid || any(pdist(uav_pos)<r_min), continue; end
                L = build_laplacian(uav_pos, r_comm);
                eigsL = sort(eig(L));
                lambda2 = eigsL(2);
                if lambda2 < 0.01, continue; end
                g = current(3) + norm([dx dy]);
                h = w_form * formation_priority(f_id) + ...
                    w_conn * (1/lambda2) + ...
                    w_dist * heuristic([nx ny], goal);
                f_total = g + h;
                next_key = key(nx, ny);
                if ~isKey(path_cost, next_key) || g < path_cost(next_key)
                    came_from(next_key) = curr_key;
                    path_cost(next_key) = g;
                    formation_at_node(next_key) = f_id;
                    open = [open; nx ny g h f_total f_id];
                    cost_map(ny,nx) = f_total;
                    if isKey(formation_at_node, curr_key) && formation_at_node(curr_key) ~= f_id
                        switch_locations(end+1,:) = [nx, ny];
                    end
                end
                break;
            end
        end
    end
end
figure('Name','Swarm A* with Formation, Paths, Heatmap');
subplot(1,2,1); hold on; axis equal; grid on;
xlim([0 map_size]); ylim([0 map_size]);
xlabel('X'); ylabel('Y'); title('Swarm Path with Formation Colors');
theta = linspace(0, 2*pi, 50);
for i = 1:size(obstacles,1)
    fill(obstacles(i,1)+obstacles(i,3)*cos(theta), ...
         obstacles(i,2)+obstacles(i,3)*sin(theta), ...
         'r','FaceAlpha',0.3);
end
colors = lines(N);
last_fid = -1;
uav_paths = cell(N,1); for i = 1:N, uav_paths{i} = []; end
for k = 1:size(path,1)
    pos = path(k,:);
    k_str = key(pos(1), pos(2));
    if isKey(formation_at_node, k_str)
        f_id = formation_at_node(k_str);
    else
        f_id = 1;
    end
    f = formations{f_id};
    color_switch = f_id ~= last_fid;
    last_fid = f_id;
    for i = 1:N
        pt = pos + f(i,:);
        uav_paths{i}(end+1,:) = pt;
        c = colors(i,:);
        if color_switch, c = [1 0 0]; end
        plot(pt(1), pt(2), 'o', 'MarkerFaceColor', c, 'MarkerEdgeColor', 'k', 'MarkerSize', 6);
    end
    pause(0.01);
end
for i = 1:N
    plot(uav_paths{i}(:,1), uav_paths{i}(:,2), '-', 'Color', colors(i,:), 'LineWidth', 1.2);
end
plot(start(1), start(2), 'gs', 'MarkerSize', 10, 'LineWidth', 2);
plot(goal(1), goal(2), 'kx', 'MarkerSize', 10, 'LineWidth', 2);
if ~isempty(switch_locations)
    plot(switch_locations(:,1), switch_locations(:,2), 'rs', 'MarkerSize', 10, 'LineWidth', 1.5);
    legend('UAVs', 'Switch Point');
end
subplot(1,2,2);
imagesc(cost_map); colorbar;
title('Total Cost Heatmap (f = g + h)');
xlabel('X'); ylabel('Y');
set(gca,'YDir','normal');
colormap hot;
function h = heuristic(p, goal)
    h = norm(p - goal);
end
function L = build_laplacian(pos, r_comm)
    N = size(pos,1);
    A = zeros(N);
    for i = 1:N
        for j = i+1:N
            if norm(pos(i,:) - pos(j,:)) <= r_comm
                A(i,j) = 1;
                A(j,i) = 1;
            end
        end
    end
    L = diag(sum(A,2)) - A;
end
function path = reconstruct_path(came_from, key_curr)
    path = [];
    while isKey(came_from, key_curr)
        xy = sscanf(key_curr, '%d_%d');
        path = [xy'; path];
        curr_key = came_from(key_curr);
    end
    path = [sscanf(key_curr, '%d_%d')'; path];
    path = reshape(path, [], 2);
end
function s = key(x, y)
    s = sprintf('%d_%d', x, y);
end
