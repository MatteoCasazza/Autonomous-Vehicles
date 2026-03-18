clc
clear
close all

map_rgb = imread('house_map_full1.png');
BW = im2bw(map_rgb, 0.95);
 
res = 0.05;
rows = size(BW, 1);
cols = size(BW, 2);
total_nodes = rows * cols;

free_nodes = find(BW == 1);
num_free = length(free_nodes);

adjacency_list = cell(total_nodes, 1);
directions = [-1, 0; 1, 0; 0, -1; 0, 1; 1 1; 1 -1; -1 1; -1 -1];

for idx = 1:length(free_nodes)
    node = free_nodes(idx);
    [i, j] = ind2sub([rows, cols], node);
    
    neighbors = [];
    for d = 1:size(directions, 1)
        ni = i + directions(d, 1);
        nj = j + directions(d, 2);
        
        if ni >= 1 && ni <= rows && nj >= 1 && nj <= cols
            % save only effective neighbours 
            if BW(ni, nj) == 1
                nn = sub2ind([rows, cols], ni, nj);
                neighbors = [neighbors, nn];
            end
        end
    end
    % for each free node there is a cell with neighbours
    adjacency_list{node} = neighbors;
end

%% PATHS
spawn = [119,110];
pairs = {
    spawn,   [163,280];
    [163, 281], [82 184]
    [82, 183], [57 97]
    [57 96], [164 38]};

path_x=[];
path_y=[];
trajectory.n=zeros(size(pairs,1),1);

for jj = 1:size(pairs, 1)
    start_pos = pairs{jj, 1};
    goal_pos = pairs{jj, 2};
    start = sub2ind([rows, cols], start_pos(1), start_pos(2));
    goal = sub2ind([rows, cols], goal_pos(1), goal_pos(2));

    tic;
    
    % A*
    [path, dist_to_goal, nodes_explored, nodes] = aStar_optimized(adjacency_list, start, goal, goal_pos, total_nodes, rows, BW);
    elapsed_time = toc;

    [nx, ny] = node_to_coords(nodes, rows, BW);
    figure;
    plot_map2(BW,rows,cols);
    title(sprintf('Trajectory n.%d: Explored Nodes = %d', jj, nodes_explored));
    hold on
    plot(ny, nx, '*')
    filename = sprintf('explored_nodes_%d.eps', jj);
    print(gcf, filename, '-depsc', '-r300');

    fprintf('Path: %d steps\n', length(path) - 1);
    fprintf('Distance: %.1f m\n', dist_to_goal*res);
    fprintf('Nodes explored: %d / %d (%.1f%%)\n', nodes_explored, num_free, ...
        100*nodes_explored/num_free);
    fprintf('Execution time: %.4f s\n\n', elapsed_time);

    [path_rows, path_cols] = ind2sub([rows, cols], path);
    path_coords = [path_rows; path_cols]';
    figure;
    plot_map2(BW,rows,cols);
    hold on;
    plot(start_pos(2), start_pos(1), 'sg', 'MarkerFaceColor', 'g', 'MarkerSize', 10);
    plot(goal_pos(2), goal_pos(1), 'sr', 'MarkerFaceColor', 'r', 'MarkerSize', 10);
    plot(path_cols, path_rows, 'b-', 'LineWidth', 2);
    title(sprintf('Trajectory n.%d', jj));
    hold off;
    filename = sprintf('ideal_trajectory_%d.eps', jj);
    print(gcf, filename, '-depsc', '-r300');

    % trajectories
    path_x=[path_x (path_cols-spawn(2))*res];
    path_y=[path_y (path_rows-spawn(1))*res];
    % number of nodes for each path
    trajectory.n(jj)=length(path_x);

end

trajectory.x = path_x;
trajectory.y = path_y;

save('trajectory_data.mat', 'trajectory');
waypoints = [path_x', path_y'];

figure;
plot_map2(BW,rows,cols);
title('Map of the House')
print(gcf, 'map_house', '-depsc', '-r300');