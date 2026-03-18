%% 
clear
close all
clc

rows = 30;
cols = 30;

map_type = 't';
% map_type = '1';
% map_type = '2';
% map_type = '3';
% map_type = '4';

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

    % matrix pixel black and white
    BW = im2bw(map_rgb,0.95);

    plot_map(BW)

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

    plot_map(BW)
    xlabel('Column (j)');
    ylabel('Row (i)');
    title(sprintf('Map %s', map_type));
    axis equal
    xlim([1 cols]);
    ylim([1 rows]);
    hold on

    plot(start_pos(1),start_pos(2),'sg','MarkerFaceColor','g')
    plot(goal_pos(1),goal_pos(2),'sr','MarkerFaceColor','r')

    % matrix Tree = [node, parent]
    T=[start start];
    % max movement per iteration
    delta_q=sqrt(2);

    count=1;
    control=randi(30)+20;
    max_iter=5000;
    flag_goal=0;
    t_Start=tic;

    while flag_goal==0 && count<max_iter
        
        % goal biasing: very fixed nember of iterations the random sample
        % is replaced by goal position to accelerate convergence
        if mod(count,control)==0   % mod compute remainder of division
            act_node=goal;
            i = goal_pos(1);
            j = goal_pos(2);
        else
            % casual node
            act_node=[rand(1)*rows rand(1)*cols];  
            i = act_node(1);
            j = act_node(2);
        end

        count=count+1;
        
        % check on the coordinates: force point in the limits of the map
        i=min(30, max(1, i)); 
        j=min(30, max(1, j));
        
        % discard point if obstacle
        if BW(round(j),round(i))==0
            continue;
        end
        
        % selection nearest point
        [Ti, Tj] = node_to_coords(T(:,1), rows, BW);
        dist = sqrt((i-Ti).^2+(j-Tj).^2);
        [min_dist, min_ind]=min(dist);
        if min_dist==0
            continue;
        end
        near_node=T(min_ind,1);
        near_coord=[Ti(min_ind) Tj(min_ind)];
        
        % direction to x new
        dist_x=(i-near_coord(1))/min_dist;
        dist_y=(j-near_coord(2))/min_dist;
        % select x new (round required to get the pixel that is closest to the desired value)
        new_node = near_coord + round(delta_q * [dist_x, dist_y]);
        % check if not obstacle
        if BW(new_node(2),new_node(1))==0
            continue;
        end

        nn_ind=coords_to_node(new_node(1), new_node(2),rows,BW);

        if any(T(:,1) == nn_ind)
            continue;
        end

        % plot_runtime(nn_ind,BW)
        
        % Upload tree
        T = [T;
            nn_ind near_node];

        if nn_ind==goal
            flag_goal=1;
        end

    end

    [Tx, Ty] = node_to_coords(T(:,1),rows,BW);
    plot(Tx, Ty, 'xb','LineWidth',2)
    
    %% path
    if flag_goal==0
        disp('No solution found')
    else
        % backtracking from goal to start
        sol_id=goal;
        path=[];
        while(sol_id~=start)
            path=[sol_id path];
            ind=find(T(:,1)==sol_id);
            sol_id=T(ind,2);
        end
        path=[sol_id path];

        %%% plot

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

        filename = sprintf('map_%s_rrtBase.eps', map_type);
        print(gcf, filename, '-depsc', '-r300');
    end
end