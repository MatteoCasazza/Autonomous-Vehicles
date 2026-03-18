clear
close all
clc

rows = 30;
cols = 30;

% map_type = 't';
% map_type = '1';
% map_type = '2';
% map_type = '3';
map_type = '4';

MT = ['t','1','2','3','4'];

for jj=1:length(MT)
    map_type=MT(jj);
    switch map_type
        case 't'
            map_rgb = imread('mappa_test_red.png');
        case '1'
            map_rgb = imread('map_1_reduced.png');
        case '2'
            map_rgb = imread('map_2_reduced.png');
        case '3'
            map_rgb = imread('map_3_reduced.png');
        case '4'
            map_rgb = imread('map_4_reduced.png');
    end
    BW = im2bw(map_rgb,0.95);
    plot_map(BW)

    %%% initialize system:
    switch map_type
        case 't'
            start_pos = [1,1];
            goal_pos = [20,30];
        case '1'
            start_pos = [1,12];
            goal_pos = [28,12];
        case '2'
            start_pos = [1,8];
            goal_pos = [27,28];
        case '3'
            start_pos = [1,1];
            goal_pos = [1,30];
        case '4'
            start_pos = [1,8];
            goal_pos = [20,18];
    end
    start= (start_pos(2)-1)*size(BW,1)+start_pos(1);
    goal= (goal_pos(2)-1)*size(BW,1)+goal_pos(1);

    %%% visualize map start & goal
    hold on
    plot(start_pos(1),start_pos(2),'sg','MarkerFaceColor','g')
    plot(goal_pos(1),goal_pos(2),'sr','MarkerFaceColor','r')
    xlabel('Column (j)');
    ylabel('Row (i)');
    title(sprintf('Map %s', map_type));
    axis equal
    xlim([1 cols]);
    ylim([1 rows]);

    T = [start NaN];
    cost = 0;
    count = 0;
    delta_q = sqrt(2);
    max_iter=5000;
    max_rew=20;
    flag_goal=0;

    t_Start=tic;
    while flag_goal==0 && count<max_iter
        % while iter<max_iter
        count = count + 1;

        % casual node
        act_node=[rand(1)*rows rand(1)*cols];
        i = act_node(1);
        j = act_node(2);

        % control limits
        i=min(30, max(1, i));
        j=min(30, max(1, j));
        % control obstacles
        if BW(round(j),round(i)) == 0
            continue;
        end

        % select x near
        [Ti, Tj] = node_to_coords(T(:,1), rows, BW);
        dist = sqrt((i-Ti).^2 + (j-Tj).^2);
        [~, min_ind] = min(dist);
        if dist(min_ind) == 0
            continue;
        end
        near_node = T(min_ind,1);

        near_coord = [Ti(min_ind) Tj(min_ind)];
        dist_near = dist(min_ind);

        dist_x = (i - near_coord(1)) / dist_near;
        dist_y = (j - near_coord(2)) / dist_near;
        new_coord = near_coord + round(delta_q * [dist_x dist_y]);

        if BW(new_coord(2), new_coord(1)) == 0
            continue;
        end
        nn_ind = coords_to_node(new_coord(1), new_coord(2), rows, BW);

        if any(T(:,1) == nn_ind)
            continue;
        end

        % select best parent for x new
        [parent_ind, new_cost] = parent_function(T, cost, nn_ind, rows, BW);
        if isempty(parent_ind)
            continue;
        end

        % upload Tree and cost
        T = [T; nn_ind parent_ind];
        cost = [cost; new_cost];


        % check if some nodes near could have lower cost passing through x
        % new
        [T, cost] = Rewire(T, cost, nn_ind, rows, BW);

        if nn_ind==goal
            flag_goal=1;
        end
        % plot_runtime(nn_ind, BW);
    end

    [Tx, Ty] = node_to_coords(T(:,1),rows,BW);
    plot(Tx, Ty, 'xb','LineWidth',2)
    idx_cost=zeros(length(cost),1);

    count=0;
    diff_idx_cost=inf;

    %%% global rewiring
    while diff_idx_cost~=0 && count<max_rew
        % reorder nodes for cost
        idx_cost_old=idx_cost;
        [cost,idx_cost]=sort(cost);
        T=T(idx_cost,:);

        count=count+1;
        % rewiring over all nodes
        for kk=1:size(T,1)
            nn_ind=T(kk,1);
            [T, cost] = Rewire(T, cost, nn_ind, rows, BW);
        end
        % measure how many nodes change position, if =0-->end
        diff_idx_cost=sum(abs(idx_cost-idx_cost_old));
    end

    %% path
    if flag_goal==0
        disp('No solution found')
    else
        sol_id=goal;
        path=[];
        while(sol_id~=start)
            path=[sol_id path];
            ind=find(T(:,1)==sol_id);
            % sol_id=T(ind,2);
            [sol_id, ~] = parent_function(T, cost, sol_id, rows, BW);
        end
        path=[sol_id path];

        for k = 1:length(path)
            [dr_j, dr_i] = node_to_coords(path(k), rows, BW);
            plot(dr_j, dr_i, 'or', 'MarkerFaceColor', 'r');
        end

        comp_time=toc(t_Start);
        [path_x, path_y] = node_to_coords(path,rows,BW);
        travelled_dist = sum(sqrt((diff(path_x)).^2+(diff(path_y)).^2));
        explored_nodes=size(T,1);

        fprintf('Computation time: %.4f s\n', comp_time);
        fprintf('Travelled distance: %.3f m\n', travelled_dist);
        fprintf('Explored nodes: %d\n', explored_nodes);

        filename = sprintf('map_%s_rrtStar.eps', map_type);
        print(gcf, filename, '-depsc', '-r300');
    end

end