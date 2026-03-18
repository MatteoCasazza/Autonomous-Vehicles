function [best_parent_ind, best_cost] = parent_function(T, cost, nn_ind, rows, BW)
    [ni, nj] = node_to_coords(nn_ind, rows, BW);
    [Ti, Tj] = node_to_coords(T(:,1), rows, BW);
    dists = sqrt((ni - Ti).^2 + (nj - Tj).^2);
    
    total_nodes = length(T);
    % raggio should depend in the number of nodes to guarantee asymptotic
    % optimality, but since the map is discretized it is forces to sqrt(2)
    gamma = min(20, max(3, round(2 * log(total_nodes)/total_nodes * rows)));
    gamma=sqrt(2);
    
    % selection near nodes
    near_inds = (dists <= gamma).*(dists>0);
    near_inds = find(near_inds);
    % if not possible parent, node not add
    if isempty(near_inds)
        best_parent_ind = [];
        best_cost = Inf;
        return;
    end
    
    min_cost = Inf;
    best_ind = [];
    % selection best parent
    for ii = 1:length(near_inds)
        idx=near_inds(ii);
        step_cost = dists(idx);
        tot_cost = cost(idx) + step_cost;
        if tot_cost < min_cost
            min_cost = tot_cost;
            best_ind = idx;
        end
    end
    
    best_parent_ind = T(best_ind, 1);
    best_cost = min_cost;
end