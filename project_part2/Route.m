classdef Route < handle
    
    properties
        config_space
        theta_res
        start
        dest
        map
        route
    end
    
    methods
        function obj = Route(map, start, dest)
            obj.map = map; % intilaized Map object
            obj.start = start;
            obj.dest = dest; % 3D-point (x,y,theta)
            obj.theta_res = 76;
        end
        
        function calc_config_space(obj)
            dt= 360/obj.theta_res;
            obj.config_space = zeros([30*obj.map.size_factor 32*obj.map.size_factor obj.theta_res]);
            i=0;
            
            for theta=0:dt:360-dt
                i=i+1;
                obj.map.robot.set_theta(theta);
                obj.map.calc_c_obstacles;
                obj.map.update_matrix_map;
                obj.config_space(:,:,i) = obj.map.matrix_map;
            end
        end
        
        function A_Star(obj, saveDir)
            % A star algorithm
            % Input: obj:
            % M- metrix [30,32,51]. 0==free space, 1== obstacle (closed list)
            % start [1*3]- x,y,theta
            % target [1*3]- x,y,theta
            start = obj.start+1;
            target = obj.dest+1;

            path = 1;
            key= distance([start,target], obj.theta_res);
            O = [key,start,0,start]'; %#ok<*PROP> 
            C = obj.config_space;
            
            fig = figure('visible', 'on', 'Units', 'normalized', 'position', [0 0 1 1]); hold on;
            with_norms = false; %set to true will print red lines to mark the normals from each line.
            
            for z=1:length(obj.map.obstacls)
                obj.map.obstacls{z}.print_obstacle(with_norms);
            end
            
            axis equal;
            fig.Children.XLim = [0 obj.map.map_size(2)];
            fig.Children.YLim = [0 obj.map.map_size(1)];
            grid minor; title(['calculated robot route']);  
            l=1;
                       
            while (path == 1)
                x_best_i = find(O(1,:) == min(O(1,:)), 1);
                x_best = O(2:4,x_best_i);
                g_x_best = O(5,x_best_i);
                plot_A_Star(obj, O, x_best(1:2), saveDir);
                pause(0.1);
                C(x_best(2), x_best(1), x_best(3)) = 1;
                potentional_routh(1:6,l) = O([2:4,6:8],x_best_i);
                l=l+1;
                O(:,x_best_i) = [];

                if (all(x_best' == target))
                    break;
                end
                
                neighbors = FindNeighbors(x_best, obj.theta_res);
                for i=1:size(neighbors,2)
                    r = neighbors(1:3,i);
                    r_type = neighbors(4,i);
                    if i ==1
                        g=0;
                    end
                    dist = distance([r',target], obj.theta_res);
                    k = is_x_not_in_O(r);
                    if isempty(k)
                        O(5,end+1) = g_x_best + neighbor_cost(r_type);
                        O(1,end) = O(5,end) + dist;
                        O(2:4,end) = r;
                        O(6:8,end) = x_best;
                    elseif g_x_best+neighbor_cost(r_type)<O(5,k)
                        O(5,k)= g_x_best+neighbor_cost(r_type) ;
                        O(1,k)= O(5,k)+ dist;
                        O(6:8,k) = x_best;
                    end
                end
                if isempty(O)
                    path = 0;
                end
            end
            
            save([saveDir '\potentional_routh.mat'], 'potentional_routh');
            calc_route(obj, potentional_routh)

            function y = is_x_not_in_O(r)
                y = find(all(O(2:4,:) == r), 1);
            end
            
            function plot_A_Star(obj, O, x_best, saveDir)
                persistent n
                
                if isempty(n)
                    n = 1;
                end
                
                if mod(n,100) == 0
                    open_list = O(2:3,:);
                    scatter(open_list(1,:)-0.5, open_list(2,:)-0.5, 'MarkerFaceColor',[1 1 1],'MarkerEdgeColor',[0 0 .0]);
%                     scatter(x_best(1)-0.5, x_best(2)-0.5, 'MarkerFaceColor',[0 .7 .7],'MarkerEdgeColor',[0 0 .0]);
                    imwrite(frame2im(getframe(gcf)), [saveDir '\' num2str(n) '.jpg']);
                end
                n = n+1;
            end

            function neighbors = FindNeighbors(r_vec, theta_res)
                xi = r_vec(1);
                yi = r_vec(2);
                zi = r_vec(3);
                neighbors_class = cat(3, [4 3 4; 3 2 3; 4 3 4], ones(3,3), [4 3 4; 3 2 3; 4 3 4]); % helps to define the cost for each neigbor
                if zi==1
                    neighborhood = C(yi-1:yi+1, xi-1:xi+1, [theta_res 1 2]);
                elseif zi==theta_res
                    neighborhood = C(yi-1:yi+1, xi-1:xi+1,[theta_res-1 theta_res 1]);
                else
                    neighborhood = C(yi-1:yi+1, xi-1:xi+1,zi-1:zi+1);
                end
                ind = find(neighborhood == 0);
                [y,x,z] = ind2sub([3 3 3],ind);
                neighbors = [xi+x'-2;yi+y'-2;zi+z'-2;neighbors_class(ind)']; % the '-2' is to center the neighborhood at [xi, yi, zi]
                
                neighbors_zeros = neighbors(3,:)==0;
                neighbors(3,neighbors_zeros) = theta_res;
                
                neighbors_zeros = neighbors(3,:)==theta_res+1;
                neighbors(3,neighbors_zeros) = 1;
                
            end
            
            function cost = neighbor_cost(type)
                p = [1 , 1, 1, 1]/100;
                cost = p(type);
            end
            
            function dist = distance(r,theta_res)
                %This function calculates the distance between any two cartesian coordinates
                dist=sqrt((r(1)-r(4))^2 + (r(2)-r(5))^2 + (min(r(3)-r(6),(theta_res + 1 - r(3))-r(6)))^2);
            end           
        end
        
        function calc_route(obj, potentional_routh)
            j=1;
            i=length(potentional_routh);
            while i>1
                obj.route(j,:) = potentional_routh(1:3,i);
                t(j)=find( potentional_routh(1,1:i-1) == potentional_routh(4,i) &...
                    potentional_routh(2,1:i-1) ==potentional_routh(5,i) & ...
                    potentional_routh(3,1:i-1) == potentional_routh(6,i));
                i=t(j);
                j=j+1;
            end
            disp(['route length: ' num2str(j) ]);
        end
            
        function plot_config_space(obj)
            [x,y,z] = ind2sub(size(obj.config_space),find(obj.config_space == 0));
            figure;
            scatter3(x,y,z, 40, 'o', 'MarkerEdgeColor','k','MarkerFaceColor',[0 .75 .75]);
        end
        
        function plot_route(obj, saveDir)
            with_norms = false; %set to true will print red lines to mark the normals from each line.
            
            fig = figure('visible', 'on', 'Units', 'normalized', 'position', [0 0 1 1]); hold on;
            
            for i=1:length(obj.map.obstacls)
                obj.map.obstacls{i}.print_obstacle(with_norms);
            end
            
%             obj.map.robot.org_vertices = [0 0; 8 0; 8 1; 0 1]*obj.map.size_factor;
            for i=size(obj.route,1):-1:1
                obj.map.robot.set_location(obj.route(i, 1:2));
                obj.map.robot.set_theta((obj.route(i, 3)-1)*(360/obj.theta_res));
                obj.map.robot.print_obstacle(with_norms);
            end
            
            axis equal;
            fig.Children.XLim = [0 obj.map.map_size(2)];
            fig.Children.YLim = [0 obj.map.map_size(1)];
            grid minor; title(['calculated robot route']);
            
            imwrite(frame2im(getframe(gcf)), [saveDir '\final route.jpg']);
            savefig('final route.fig')
        end
    end
    
    methods (Static)
        function obj_route = main()
            saveDir = datestr(datetime, 'mm-dd-yy HH-MM-SS');
            mkdir(saveDir);
            
            obj_map = Map.init_question_map();
            obj_route = Route(obj_map, [4 24 0]*obj_map.size_factor, [4 8 0]*obj_map.size_factor);
            obj_route.calc_config_space()
%             Route.walk_configuration_space(obj_route.config_space, obj_route.theta_res)
%             Route.walk_configuration_space2(obj_route, obj_route.config_space, obj_route.theta_res)
%             obj_route.plot_config_space()
            obj_route.A_Star(saveDir)
%             calc_route(obj_route, potentional_routh)
            obj_route.plot_route(saveDir)
        end
        
        function walk_configuration_space(C, theta_res)
            close all
            theta = 1;
            f=0.5;
            g = figure;
            
            while(1)
                [x,y]= ind2sub([size(C,1),size(C,2)],find(C(:,:,theta) == 1));                        
                scatter(y+f,x+f,'MarkerFaceColor',[1 1 1],'MarkerEdgeColor',[0 0 .0]); xlim([1, size(C,2)]); ylim([1 size(C,1)]); grid minor;
                title(['\theta = ' num2str(theta)]);
                w = waitforbuttonpress;
                v = double(get(gcf, 'CurrentCharacter'));
                switch v
                    case 29 %right
                        theta = theta + 1;
                        if theta == theta_res +1
                            theta = 1;
                        end
                    case 28 %left
                        theta = theta - 1;
                        if theta == 0
                            theta = theta_res;
                        end
                    case 113 %q
                        close all
                        break;
                end
            end
            disp('END!');
        end
        
        function walk_configuration_space2(obj, C, theta_res)
            close all
            dt = 360/theta_res;
            theta = 0;
            theta_i = 1;
            f=0.5;
            g = figure; hold on;
            [a,b] = size(obj.map.matrix_map);
            grid minor; axis equal; xlim([0 b]); ylim([0 a]);

            
            
            while(1)
                obj.map.robot.set_theta(theta);
                obj.map.calc_c_obstacles;
                obj.map.update_matrix_map;
                title(['\theta = ' num2str(theta)]);
                print(obj.map);
                w = waitforbuttonpress;
                v = double(get(gcf, 'CurrentCharacter'));
                switch v
                    case 29 %right
                        theta = theta + dt;
                        theta_i = theta_i + 1;
                        if theta >= 360
                            theta = 0;
                            theta_i = 1;
                        end
                    case 28 %left
                        theta = theta - dt;
                        theta_i = theta_i - 1;
                        if theta <= 0
                            theta = 360 - dt;
                            theta_i = theta_res;
                        end
                    case 113 %q
                        close all
                        break;
                end
                cla(g.Children);
                grid minor; axis equal; xlim([0 b]); ylim([0 a]);
            end
            disp('END!');
            
            function print(obj)
                with_norms = false;
                for i=1:length(obj.c_obstacls)
                    obj.c_obstacls{i}.print_obstacle(with_norms);
                end
                for i=1:length(obj.obstacls)
                    obj.obstacls{i}.print_obstacle(with_norms);
                end
                for i=1:length(obj.c_obstacls)
                    obj.c_obstacls{i}.add_label_to_plot;
                end
            
                imagesc(0.5,0.5, obj.matrix_map , 'AlphaData', 0.3); ax = gca; ax.YDir = 'normal';
                
                [x,y]= ind2sub([size(C,1),size(C,2)],find(C(:,:,theta_i) == 1));
                scatter(y+f,x+f,'MarkerFaceColor',[1 1 1],'MarkerEdgeColor',[0 0 0]);

            end
        end
    end
end


