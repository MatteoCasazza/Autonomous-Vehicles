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


    %%% visualize map start & goal (pcolor?)

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
    
    % two trees
    Ta=[start start];
    Tb=[goal goal];
    % max movements per iteration
    delta_q=sqrt(2);
    count=1;
    max_iter=5000;
    flag_goal=0;

    t_Start=tic;
    while flag_goal==0 && count<max_iter

        % alternation 2 trees (1 explorer, 1 try to connect them)
        if mod(count,2)~=0
            T=Ta;
            T_connect=Tb;
        else
            T=Tb;
            T_connect=Ta;
        end

        %%% EXPLORE %%%

        % casual node
        act_node=[rand(1)*rows rand(1)*cols];
        i = act_node(1);
        j = act_node(2);

        % control not obastacle
        if BW(round(j),round(i)) == 0
            continue;
        end
        
        % select x near
        [Ti, Tj] = node_to_coords(T(:,1), rows, BW);
        dist = sqrt((i-Ti).^2+(j-Tj).^2);
        [min_dist, min_ind]=min(dist);
        if min_dist==0
            continue;
        end
        near_node=T(min_ind,1);
        near_coord=[Ti(min_ind) Tj(min_ind)];

        % select x new
        dist_x=(i-near_coord(1))/min_dist;
        dist_y=(j-near_coord(2))/min_dist;

        new_node = near_coord + round(delta_q * [dist_x, dist_y]);
        if BW(new_node(2),new_node(1))==0
            continue;
        end

        nn_ind=coords_to_node(new_node(1), new_node(2),rows,BW);
        % plot_runtime(nn_ind,BW)

        T = [T;
            nn_ind near_node];

        %%% CONNECT %%%

        nn_ind_c=0;

        while T(end,1)~=nn_ind_c   % continue untill it reach last node of the explored tree

            % target = last node of T
            act_node= T(end,1);
            [i, j] = node_to_coords(act_node, rows, BW);

            [Ti, Tj] = node_to_coords(T_connect(:,1), rows, BW);
            dist = sqrt((i-Ti).^2+(j-Tj).^2);
            [min_dist, min_ind]=min(dist);
            if min_dist==0
                break;
            end
            near_node=T_connect(min_ind,1);
            near_coord=[Ti(min_ind) Tj(min_ind)];

            dist_x=(i-near_coord(1))/min_dist;
            dist_y=(j-near_coord(2))/min_dist;

            new_node = near_coord + round(delta_q * [dist_x, dist_y]);
            if BW(new_node(2),new_node(1))==0
                break;
            end

            nn_ind_c=coords_to_node(new_node(1), new_node(2),rows,BW);
            % plot_runtime(nn_ind_c,BW)

            % upload T connect
            T_connect = [T_connect;
                nn_ind_c near_node];
        end
        %%% SWAP %%%

        if mod(count,2)~=0
            Ta=T;
            Tb=T_connect;
        else
            Tb=T;
            Ta=T_connect;
        end

        count=count+1;

        if Ta(end,1)==Tb(end,1)
            flag_goal=1;
        end

    end

    [Tx, Ty] = node_to_coords(Ta(:,1),rows,BW);
    plot(Tx, Ty, 'xb','LineWidth',2)
    [Tx, Ty] = node_to_coords(Tb(:,1),rows,BW);
    plot(Tx, Ty, 'xb','LineWidth',2)

    %% path
    if flag_goal==0
        disp('No solution found')
    else
        % da connection a start
        sol_id=Tb(end,1);
        path=[];
        while(sol_id~=start)
            path=[sol_id path];
            ind=find(Ta(:,1)==sol_id);
            sol_id=Ta(ind,2);
        end

        % da connection a goal
        path=[sol_id path];
        sol_id = Tb(end,1);
        while sol_id~=goal
            path=[path sol_id];
            ind=find(Tb(:,1)==sol_id);
            ind=max(ind);  % nodo di connessione puo apparire piu volte
            sol_id=Tb(ind,2);
        end
        path=[path sol_id];

        %%% plot
        for k = 1:length(path)
            [dr_j, dr_i] = node_to_coords(path(k), rows, BW);
            plot(dr_j, dr_i, 'or', 'MarkerFaceColor', 'r');
        end

        comp_time=toc(t_Start);
        [path_x, path_y] = node_to_coords(path,rows,BW);
        travelled_dist = sum(sqrt((diff(path_x)).^2+(diff(path_y)).^2));
        explored_nodes=size(Ta,1)+size(Tb,1);

        fprintf('Computation time: %.4f s\n', comp_time);
        fprintf('Travelled distance: %.3f m\n', travelled_dist);
        fprintf('Explored nodes: %d\n', explored_nodes);

        filename = sprintf('map_%s_rrtConnected.eps', map_type);
        print(gcf, filename, '-depsc', '-r300');
    end
end