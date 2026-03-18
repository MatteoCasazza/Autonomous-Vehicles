function [T, cost] = Rewire(T, cost, nn_ind, rows, BW)
    
    [ni, nj] = node_to_coords(nn_ind, rows, BW);
    [Ti, Tj] = node_to_coords(T(:,1), rows, BW);
    dists = sqrt((ni - Ti).^2 + (nj - Tj).^2);
    total_nodes = length(T);
    
    % raggio should depend in the number of nodes to guarantee asymptotic
    % optimality, but since the map is discretized it is forces to sqrt(2)
    gamma = min(20, max(3, round(2 * log(total_nodes)/total_nodes * rows)));
    gamma=sqrt(2);
    
    % selection near nodes
    near_inds = find(dists <= gamma & T(:,1) ~= nn_ind);
    ind=nn_ind==T(:,1);
    node_cost = cost(ind);
    
    % rewiring cycle
    for idx = near_inds'
        [vi, vj] = node_to_coords(T(idx,1), rows, BW);
        step_cost = sqrt((ni - vi)^2 + (nj - vj)^2);
        total_cost = node_cost + step_cost;
        old_cost = cost(idx);
        % upload parent
        if total_cost < old_cost
            T(idx, 2) = nn_ind;
            cost(idx) = total_cost;
        end
    end
end