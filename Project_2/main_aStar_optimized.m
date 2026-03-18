function [path, dist_to_goal, nodes_explored, nodes] = dijkstra_optimized(adjacency_list, start, goal, goal_pos, total_nodes, rows, BW)

    g = inf(total_nodes, 1); % cost matrix
    pre = zeros(total_nodes, 1);  % previous node
    visited = false(total_nodes, 1);  % avoid already explored nodes
    
    g(start) = 0;
    
    [start_x, start_y] = node_to_coords(start, rows, BW);
    h_start = sqrt((start_x - goal_pos(1))^2 + (start_y - goal_pos(2))^2);
    fq = [h_start, start]; % frontier queue 
    
    nodes_explored = 0;
    nodes = start;
    
    % continue untill frontier queue is full
    while ~isempty(fq)
        [~, min_idx] = min(fq(:, 1));
        act_node = fq(min_idx, 2);
        [cn_x, cn_y] = node_to_coords(act_node, rows, BW);
    
        fq(min_idx, :) = [];

        % avoid already visited nodes
        if visited(act_node)
            continue;
        end
        visited(act_node) = true;

        nodes_explored = nodes_explored + 1;
        nodes = [nodes; act_node];
    
        if act_node == goal
            break;
        end
    
        neighbors = adjacency_list{act_node};
    
        % evaluation each neighbours
        for nn = neighbors
            if visited(nn)
                continue;
            end
    
            [nx, ny] = node_to_coords(nn, rows, BW);
            step_cost = sqrt((nx - cn_x)^2 + (ny - cn_y)^2);
            tentative_g = g(act_node) + step_cost;
    
            % if better cost, update
            if tentative_g < g(nn)
                g(nn) = tentative_g;
                h = sqrt((nx - goal_pos(1))^2 + (ny - goal_pos(2))^2);
                f = tentative_g + h;
                pre(nn) = act_node;
                fq = [fq; f, nn];
            end
        end
    end
    
    dist_to_goal = g(goal);
    
    if isinf(dist_to_goal)
        path = [];
        return;
    end
    
    path = goal;
    current = goal;
    
    while current ~= start
        current = pre(current);
        path = [current, path];
    end

end

