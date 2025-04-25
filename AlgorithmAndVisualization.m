% Multi UAV Obstacle Avoidance, Formation-Aware and Communication-Aware Path-finding Algorithm in a Known Environment
% This algorithm is for navigating $N$ UAVs  from point $A$ to $B$ in a 2D map with known environment, 
% while satisfying safety, formation, and communication-connectivity constraints. 
% The algorithm modifies the classical A* search to include swarm-specific metrics like 
% formation feasibility and algebraic connectivity.

% Authors: Asif Rizwan and Sampurak Talukdar
%
% ALGORTHIM VISUALIZATION
%___________________________________________________________________________________________________________________


clc;
clear;
close all;
map_size = 100;
grid_size = 1;
N = 8;
r_min = 2;
r_comm = 7;
goal_radius = 4;
D = 3;
start = [10, 10];
goal  = [90, 90];
theta = linspace(0, 2*pi, N+1);
theta(end) = [];
F1 = D * [cos(theta') sin(theta')];
F2 = zeros(N, 2);
mid = N/2;
for i = 1:mid
    F2(i,:) = [-1.2*r_min * (i - 1), -D/2];
end
for i = 1:mid
    F2(mid + i,:) = [-1.2*r_min * (i - 1), D/2];
end
F3 = -1.2*r_min * (0:N-1)';
F3 = [F3 zeros(N,1)];
formations = {F1, F2, F3};
formation_labels = ["F1", "F2", "F3"];
formation_priority = [5, 10, 20];
w_safety = 75;
w_form = 15;
w_conn = 9;
w_dist = 1;
obstacles = [
    65 87 10; 80 65 6; 40 60 3; 60 60 1; 67 60 1; 45 45 6; 40 70 4;
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
all_uav_trajs = {};
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
            for f_id = 1:length(formations)
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
function path = reconstruct_path(came_from, curr_key)
    path = [];
    while isKey(came_from, curr_key)
        coords = sscanf(curr_key, '%d_%d');
        path = [coords'; path];
        curr_key = came_from(curr_key);
    end
    coords = sscanf(curr_key, '%d_%d');
    path = [coords'; path];
    path = reshape(path, [], 2);
end
figure('Name','Swarm A* with Formation, Paths, Heatmap');
subplot(1,2,1); hold on; axis equal; grid on;
imagesc(cost_map');
colorbar;
colormap hot;
plot(start(1), start(2), 'go', 'MarkerSize', 3, 'LineWidth', 2);
plot(goal(1), goal(2), 'rx', 'MarkerSize', 3, 'LineWidth', 2);
for i = 1:size(path,1)
    pos = path(i,:);
    path_key = key(pos(1), pos(2));
    if isKey(formation_at_node, path_key)
        f_id = formation_at_node(path_key);
        f = formations{f_id};
        uav_pos = zeros(N,2);
        valid = true;
        for k = 1:N
            pt = pos + f(k,:);
            if any(pt < 1) || any(pt > map_size)
                valid = false;
                break;
            end
            uav_pos(k,:) = pt;
        end
        if valid
            all_uav_trajs{i} = uav_pos;
            plot(uav_pos(:,1), uav_pos(:,2), 'bo-', 'MarkerSize', 5, 'LineWidth', 1.5);
        else
            all_uav_trajs{i} = [];
        end
    else
        all_uav_trajs{i} = [];
    end
end
xlim([0 map_size]); ylim([0 map_size]);
legend('Start','Goal','Obstacles','UAV Path','Switch Points');
title('Final Path with Formations');
subplot(1,2,2); axis([0 map_size 0 map_size]); axis equal; grid on; hold on;
title('Multi-UAV Trajectory Animation');
xlim([0 map_size]); ylim([0 map_size]);
for t = 1:length(all_uav_trajs)
    cla;
theta = linspace(0, 2*pi, 50);
for i = 1:size(obstacles,1)
    fill(obstacles(i,1)+obstacles(i,3)*cos(theta),
         obstacles(i,2)+obstacles(i,3)*sin(theta),
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
subplot(1,2,1); hold on; axis equal; grid on;
title('Swarm Path with Formations');
xlabel('X'); ylabel('Y');
plot(start(1), start(2), 'go', 'MarkerSize', 8, 'LineWidth', 2);
plot(goal(1), goal(2), 'rx', 'MarkerSize', 8, 'LineWidth', 2);
subplot(1,2,2);
title('Heatmap of Total Cost');
xlabel('X'); ylabel('Y');
xlim([0 map_size]); ylim([0 map_size]);
[X, Y] = meshgrid(1:map_size, 1:map_size);
xlim([0 map_size]); ylim([0 map_size]);
contourf(X, Y, cost_map, 10, 'LineColor','none');
colorbar;
axis equal tight;
trail = [];
for t = 1:length(all_uav_trajs)
    cla;
    xlim([0 map_size]); ylim([0 map_size]);
theta = linspace(0, 2*pi, 50);
for i = 1:size(obstacles,1)
    fill(obstacles(i,1)+obstacles(i,3)*cos(theta),
         obstacles(i,2)+obstacles(i,3)*sin(theta),
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
function s = key(x, y)
    s = sprintf('%d_%d', x, y);
end
