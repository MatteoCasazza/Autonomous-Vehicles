clear
clc
close all

type = 'v2';
% type = 'Euclidian';
% type = 'Manhattan';
% type = 'Chebishev';
% type = 'Diagonal';

TT = ['v', 'E', 'M', 'C', 'D'];

map_type = 'test';
% map_type = '1';
% map_type = '2';
% map_type = '3';
% map_type = '4';

MT = ['t','1','2','3','4'];

for ii=5
    type=TT(ii);
    for jj=1:5
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

        % grid construction, 0.95 threshold for obstacle
        BW = im2bw(map_rgb,0.95);  % matrice in bianco e nero (1=bianco=libero)
        rows = 30;
        cols = 30;
         
        plot_map(BW)
        xlabel('Column (j)');
        ylabel('Row (i)');
        title(sprintf('Map %s', map_type));
        axis equal
        xlim([1 cols]);
        ylim([1 rows]);
        filename = sprintf('map_%s.eps', map_type);
        print(gcf, filename, '-depsc', '-r300');

        % initialize variables
        G=-1*ones(size(BW,1)*size(BW,2)); % tree dependencies
        dist=inf*ones(size(BW,1)*size(BW,2),1); % C(q)
        prec=inf*ones(size(BW,1)*size(BW,2),1); % parenting 
        nodelist=-1*ones(size(BW,1)*size(BW,2),1); % frontier queue (nodes to visit)
        % -1=never discovered, 0=already visited, 1=to visit

        % create edge matrix

        for i=1:size(BW,1)
            for j=1:size(BW,2)
                if BW(i,j)==1
                    %%% itself
                    G((i-1)*size(BW,1)+j,(i-1)*size(BW,1)+j)=0;

                    %%% 4 directions
                    if i+1 >0 && i+1<=size(BW,1) && BW(i+1,j)==1
                        G((i-1)*size(BW,1)+j,(i+1-1)*size(BW,1)+j)=1;
                    end
                    if i-1 >0 && i-1<=size(BW,1) && BW(i-1,j)==1
                        G((i-1)*size(BW,1)+j,(i-1-1)*size(BW,1)+j)=1;
                    end
                    if j+1 >0 && j+1<=size(BW,2) && BW(i,j+1)==1
                        G((i-1)*size(BW,1)+j,(i-1)*size(BW,1)+j+1)=1;
                    end
                    if j-1 >0 && j-1<=size(BW,2) && BW(i,j-1)==1
                        G((i-1)*size(BW,1)+j,(i-1)*size(BW,1)+j-1)=1;
                    end

                    %%% Diagonal directions
                    if i+1 >0 && i+1<=size(BW,1) && j+1 >0 && j+1<=size(BW,2) && BW(i+1,j+1)==1
                        G((i-1)*size(BW,1)+j,(i+1-1)*size(BW,1)+j+1)=2;
                    end
                    if i+1 >0 && i+1<=size(BW,1) && j-1 >0 && j-1<=size(BW,2) && BW(i+1,j-1)==1
                        G((i-1)*size(BW,1)+j,(i+1-1)*size(BW,1)+j-1)=2;
                    end
                    if i-1 >0 && i-1<=size(BW,1) && j+1 >0 && j+1<=size(BW,2) && BW(i-1,j+1)==1
                        G((i-1)*size(BW,1)+j,(i-1-1)*size(BW,1)+j+1)=2;
                    end
                    if i-1 >0 && i-1<=size(BW,1) && j-1 >0 && j-1<=size(BW,2) && BW(i-1,j-1)==1
                        G((i-1)*size(BW,1)+j,(i-1-1)*size(BW,1)+j-1)=2;
                    end
                end
            end
        end


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
        
        % serve numerazione lineare dei nodi
        start= (start_pos(2)-1)*size(BW,1)+start_pos(1);
        goal= (goal_pos(2)-1)*size(BW,1)+goal_pos(1);

        %%% visualize map start & goal (pcolor?)

        plot_map(BW)

        plot(start_pos(1),start_pos(2),'sg','MarkerFaceColor','g')
        plot(goal_pos(1),goal_pos(2),'sr','MarkerFaceColor','r')
        xlabel('Column (j)');
        ylabel('Row (i)');
        title('Trajectory')
        axis equal
        xlim([1 cols]);
        ylim([1 rows]);


        %%%% Dijkstra %%%%%%%%%%%%%%%%%%%%
        %%% initialize actual node = start

        dist(start)=0; % inital distance C(q)=0
        act_node=start; %actual node

        [~,con_nodes]=find(G(act_node,:)>0); % def connected vertex
        % (con_nodes=vettore degli indici dei nodi adiacenti al nodo
        % attuale)
        nodelist(con_nodes,1) = 1; % add to visit
        nodelist(act_node,1)=0; % visited, removed q from frontier queue
       
        %%% further steps

        t_Start=tic; % per misurare tempo computazione

        while any(nodelist(:,1)==1) && act_node~=goal % ciclo continua finche esistono nodi da esplorare
            i_con = length(con_nodes);
            while i_con > 0      % avail & not visited

                if G(act_node,con_nodes(i_con)) == 2 
                    step_cost = sqrt(2); % se movimento diagonale
                else
                    step_cost = 1; % se ortogonale
                end
                
                % nuovo costo di arrivo
                g_new = dist(act_node) + step_cost;

                % aggiornamento se percorso migliore
                if g_new < dist(con_nodes(i_con)) 

                    dist(con_nodes(i_con)) = g_new;        % salva g
                    prec(con_nodes(i_con)) = act_node;   % e salvi padre
                end

                i_con = i_con - 1;
            end

            % plot_runtime(act_node,BW)
            %%% evaluate new candidate node & new neighbours

            % select the next act_node considering f = g + h,
            cand = find(nodelist(:,1)==1);   % candidati non ancora visitati
            f_cand = inf(size(cand));
            for k = 1:length(cand)
                [x_c, y_c] = node_to_coords(cand(k),rows,BW);

                switch type
                    case 'v'
                        h_c = 0;
                    case 'E'
                        h_c = sqrt((goal_pos(1)-x_c).^2 + (goal_pos(2)-y_c).^2);
                    case 'M'
                        h_c = abs(goal_pos(1) - x_c) + abs(goal_pos(2) - y_c);
                    case 'C'
                        h_c = max([abs(goal_pos(1) - x_c) abs(goal_pos(2) - y_c)]);
                    case 'D'
                        h_c = abs(goal_pos(1) - x_c) + abs(goal_pos(2) - y_c) + (sqrt(2)-2)*min([abs(goal_pos(1) - x_c) abs(goal_pos(2) - y_c)]);
                end

                f_cand(k) = dist(cand(k)) + h_c;  % f = g + h
            end

            [~, idx_min] = min(f_cand);
            act_node = cand(idx_min);        % nuovo nodo con f minimo

            nodelist(act_node,1) = 0;        % visited
            [~,con_nodes] = find(G(act_node,:)>0); % nuovi vicini

            i_con = length(con_nodes);
            while i_con > 0
                if nodelist(con_nodes(i_con),1) ~= 0   
                    nodelist(con_nodes(i_con),1) = 1;  
                end
                i_con = i_con - 1;
            end
        end

        nodes=1:900;
        % visualizzazione dei nodi esplorati
        [Tx, Ty] = node_to_coords(nodes(nodelist==0),rows,BW);
        plot(Tx, Ty, 'xb','LineWidth',2)

        dr_i=start_pos(2);
        dr_j=start_pos(1);
        travelled_dist=0;

        %%% reconstruction of the shortest path
        if dist(goal)<inf
            % parti da goal, risali i padri fino a start, ottieni cammino
            % minimo
            sol_id=goal;
            path=[];
            while(sol_id~=start)
                path=[sol_id path];
                sol_id=prec(sol_id);
            end
            path=[sol_id path];

            %%% plot shortest path
            for i=1:length(path)
                dr_i_old=dr_i;
                dr_j_old=dr_j;
                dr_i=0;
                dr_j=path(i);
                while size(BW,1)<dr_j
                    dr_i=dr_i+1;
                    dr_j=dr_j-size(BW,1);
                end
                dr_i=(dr_i)+1;
                if i<=length(path)
                    % calcolo travel distance
                    travelled_dist=travelled_dist+sqrt((dr_j-dr_j_old).^2+(dr_i-dr_i_old)^2);
                end
                plot(dr_j,dr_i,'or','MarkerFaceColor','r');
            end
        else
            disp('no solution found')
        end

        % metriche finali
        comp_time=toc(t_Start);
        explored_nodes=sum(nodelist==0);
        dist(goal)

        fprintf('Computation time: %.4f s\n', comp_time);
        fprintf('Travelled distance: %.3f m\n', travelled_dist);
        fprintf('Explored nodes: %d\n', explored_nodes);

        filename = sprintf('%s_%s.eps', map_type, type);
        print(gcf, filename, '-depsc', '-r300');
    end
end