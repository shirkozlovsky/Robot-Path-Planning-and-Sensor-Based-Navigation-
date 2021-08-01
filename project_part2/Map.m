classdef Map < handle
    properties
        map_size
        size_factor
        
        matrix_map
        obstacls
        c_obstacls
        robot
        
        non_obstacle_i
        obstacle_i
        robot_i
        wall_i
    end
    
    methods
        function obj = Map(robot, obstacls)
            %UNTITLED Construct an instance of this class
            %   Detailed explanation goes here
            obj.size_factor = 1.5;
            obj.map_size = [30 32]*obj.size_factor; % detarmine in the question
            
            %inside map index
            obj.obstacle_i = 1;
            obj.non_obstacle_i = 0;
            obj.robot_i = 2;
                        
            % create the robot
            obj.robot = Robot(robot{1}*obj.size_factor, robot{2});

            % creat the map obstacles and c-obstacles
            obj.obstacls = cell(1,length(obstacls));
            
            % saving the original obstacles objects and the c_obstacls list
            % for print_map
            for i=1:length(obstacls)
                obj.obstacls{i} = Map_Object(obstacls{i}{1}*obj.size_factor,1, obstacls{i}{2});
            end
        end       
                
        function mat = init_matrix_map(obj)
            mat = zeros(obj.map_size(1), obj.map_size(2));
        end
        
        function update_matrix_map(obj)
            % the 'imfill' works on binery images. thus we fill the matrix
            % map with the robot (index '2') only after we activate the 'imfill'.
            obj.matrix_map = obj.init_matrix_map();
            
            for i=1:length(obj.obstacls)
                tmp_mat = obj.draw_object_on_matrix_map(obj.c_obstacls{i}.vertices, obj.obstacle_i, obj.c_obstacls{i}.norms);
                obj.matrix_map = min((obj.matrix_map + imfill(tmp_mat)),1);
            end
        end
        
        function mat = get_obstacle_matrix_map(obj)
            % the 'imfill' works on binery images. thus we fill the matrix
            % map with the robot (index '2') only after we activate the 'imfill'.
            mat = obj.init_matrix_map();
            
            for i=1:length(obj.obstacls)
                tmp_mat = obj.draw_object_on_matrix_map(obj.obstacls{i}.vertices, obj.obstacle_i);
                mat = min((obj.matrix_map + imfill(tmp_mat)),1);
            end
        end
        
        %%%% print functions %%%
        function fig = get_map(obj, show)
            with_norms = false; %set to true will print red lines to mark the normals from each line.
            
            fig = figure('visible', show); hold on;            
            obj.robot.print_obstacle(with_norms);
            for i=1:length(obj.c_obstacls)
                obj.c_obstacls{i}.print_obstacle(with_norms);
            end
            for i=1:length(obj.obstacls)
                obj.obstacls{i}.print_obstacle(with_norms);
            end
            for i=1:length(obj.c_obstacls)
                obj.c_obstacls{i}.add_label_to_plot;
            end
            axis equal;
            fig.Children.XLim = [0 obj.map_size(2)];
            fig.Children.YLim = [0 obj.map_size(1)];
            grid minor; title(['Configuration space for fixes-\theta = ' num2str(obj.robot.theta, '%.3f') '^o']);
        end
        
        function matrix_map = get_map_matrix(obj, show)
            update_matrix_map(obj);
           
            c = [1 1 1; 0 1 0; 1 0 0]; %obstacles in red, wall in black and robot in blue;

            matrix_map = figure('visible', show);
            [a,b] = size(obj.matrix_map);
            imagesc(0.5,0.5, obj.matrix_map); ax = gca; ax.YDir = 'normal';
            grid minor; axis equal; xlim([0 b]); ylim([0 a]); title(['Discrete configuration space for fixes-\theta = ' num2str(obj.robot.theta, '%.3f') '^o']);
            colormap(c);
        end
            
        function print_maps(obj)
            get_map(obj, 'on');
            get_map_matrix(obj, 'on'); 
        end
        
        %%%% draw objects and lines %%%%
        function mat = draw_object_on_matrix_map(obj, map_obj, val, norms)
            mat = init_matrix_map(obj);
            %the objects vertices are initilized so the last vertix is the
            %same as the first so a line will be draw between them in this
            %loop - thats why the loop is with '-1';
            max_x = max(map_obj(:,1))-1;
            max_y = max(map_obj(:,2))-1;
            min_x = min(map_obj(:,1))+1;
            min_y = min(map_obj(:,2))+1;

            for i=1:length(map_obj)-1
                if norms(i) >= 0 && norms(i) <= 90
                    x = 1; y=1;
                elseif norms(i) > 90 && norms(i) <= 180
                    x = -1; y=1;
                elseif norms(i) > -90 && norms(i) < 0
                    x = 1; y=-1;
                elseif norms(i) >= -180 && norms(i) <= -90
                    x =-1; y=-1;
                end
                mat = obj.drwa_line(mat, map_obj(i,:)+1, map_obj(i+1,:)+1, val, [max_x max_y min_x min_y], x, y);
            end
        end
        
        function mat = drwa_line(obj, mat, start_point, end_point, val, max_val, x ,y)
            start_point = convret_point_to_map_size_point(obj, start_point, max_val);
            end_point = convret_point_to_map_size_point(obj, end_point, max_val);
            nPoints = 100;

            switch y
                case 1
                    rIndex = ceil(linspace(start_point(2), end_point(2), nPoints));  % Row indices
                case -1
                    rIndex = floor(linspace(start_point(2), end_point(2), nPoints));  % Row indices
                otherwise
                    rIndex = round(linspace(start_point(2), end_point(2), nPoints));  % Row indices
            end
            switch x
                case 1
                    cIndex = ceil(linspace(start_point(1), end_point(1), nPoints));  % Column indices
                case -1
                    cIndex = floor(linspace(start_point(1), end_point(1), nPoints));  % Column indices
                otherwise
                    cIndex = round(linspace(start_point(1), end_point(1), nPoints));  % Column indices
            end
            
            index = sub2ind(size(mat), rIndex, cIndex);     % Linear indices
            mat(index) = val;
        end
        
        function map_point = convret_point_to_map_size_point(obj, point, max_val)
            x = max([min([point(1) obj.map_size(2) max_val(1)]), max_val(3),1]);
            y = max([min([point(2) obj.map_size(1) max_val(2)]), max_val(4),1]);
            map_point = [x y];
%             map_point = max([min([point(1) obj.map_size(2) max_val(1)]) min([point(2) obj.map_size(1) max_val(2)])],1);
        end
        
        %%%% c obstacles %%%%
        function calc_c_obstacles(obj)
            obj.c_obstacls = cell(1,length(obj.obstacls));
            for i=1:length(obj.obstacls)
                obj.c_obstacls{i} = Map_Object(obj.calc_c_obstacle(obj.obstacls{i}), 1, obj.obstacls{i}.name);
            end
        end
        
        function c_obstacle = calc_c_obstacle(obj, obstacle)
            robot_dim = obj.robot.vertices; %the method works with the robot relative vertices (the robot dims)
%             robot_dim = [0 0; 7.5 0; 7.5 0.5; 0 0.5]*obj.size_factor;
            m = size(obj.robot.norms,1);
            n = size(obstacle.norms,1);
            
            var_axes = [1:m 1:n; ones(1,m), ones(1,n)*2; obj.robot.norms(:,1)', obstacle.norms(:,1)']; %help matrix for the method implementation.
            [sort_var_axes(3,:), i] = sort(var_axes(3,:));
            sort_var_axes(1:2,:) = var_axes(1:2,i);
            
            x = find(sort_var_axes(2,:) == 2,1);
            y = find(sort_var_axes(2,:) == 1,1);
            sort_var_axes = [sort_var_axes, sort_var_axes(:,1:x+1) + [0 0 360]'];
            
            c_obstacle = [];
            tmp = [];
            
            for i=y:length(sort_var_axes)-1
                if sort_var_axes(2,i) == 1
                    tmp = [tmp; robot_dim(sort_var_axes(1,i),:)];
                else
                    if sort_var_axes(3,i) == sort_var_axes(3,i-1)
                        c_obstacle = [c_obstacle; obstacle.vertices(sort_var_axes(1,i),:) - tmp];
                    else
                        tmp = [tmp; robot_dim(sort_var_axes(1,i+1),:)];
                        c_obstacle = [c_obstacle; obstacle.vertices(sort_var_axes(1,i),:) - tmp];
                    end
                    tmp = [];
                end
            end
        end  
    end
    
    methods (Static)
        function test_mapInit_and_printMap(factor)
            obj = Map(factor);
            print_map(obj)
        end
        
        function test_insert_robot_and_obstacles()
            robot = {[0 0; 0 5; 2 5; 2 0], [0 0]};
            obstacles = {{[0 0; 0 10; 10 10; 10 0], [-10 5]}, {[-5 0; 0 5; 5 0; 0 -5], [20 20]}};
            obj = Map(1,robot, 45, obstacles);
            print_map(obj)
        end
        
        function test_draw_line()
            obj = Map(1);
            drwa_line(obj, [6 10], [21 21], obj.obstacle)
            drwa_line(obj, [10 10], [10 21], obj.obstacle)
            drwa_line(obj, [4 10], [10 12], obj.obstacle)
            print_map(obj);
        end
        
        function test_c_obstacle()
            robot = {[0 0; 2 0; 2 5; 0 5], [0 0]};
            obstacles = {{[0 0; 10 0; 10 10; 0 10], [-10 5]}, {[-5 0; 0 -5; 5 0; 0 5], [20 20]}};
            theta0 = -90;
            obj = Map(1,robot, theta0, obstacles);
            c_obstacle = calc_c_obstacle(obj, obj.obstacls{1});
            obj.add_c_obstacle(c_obstacle)
            c_obstacle = calc_c_obstacle(obj, obj.obstacls{2});
            obj.add_c_obstacle(c_obstacle)
            print_map(obj, 'on')
        end
        
        function obj = init_question_map()
            robot = {[0 0; 8 0; 8 1; 0 1], 'robot'};
            B0_1 = {[0 0; 32 0; 32 1; 0 1]+[0 29], 'B_{01}'};
            B0_2 = {[0 0; 1 0; 1 30; 0 30]+[0 0], 'B_{02}'};
            B0_3 = {[0 0; 32 0; 32 1; 0 1]+[0 0], 'B_{03}'};
            B0_4 = {[0 0; 1 0; 1 30; 0 30]+[31 0], 'B_{04}'};
%             
            B1 = {[0 0; 10 0; 10 1; 0 1]+[0 18], 'B_1'};
            B2 = {[0 0; 1 0; 1 12; 0 12]+[17 17], 'B_2'};
            B3 = {[0 0; 7 0; 7 1; 0 1]+[25 18], 'B_3'};
            B4 = {[0 0; 19 0; 19 1; 0 1]+[0 14], 'B_4'};
            B5 = {[0 0; 8 0; 8 2; 0 2]+[24 13], 'B_5'};
            B6 = {[0 0; 2 0; 2 1; 0 1]+[10 19], 'B_6'};
            B7 = {[0 0; 2 0; 2 1; 0 1]+[23 19], 'B_7'};
            
            obstacles = {B0_1, B0_2, B0_3, B0_4, B1, B2, B3, B4, B5, B6, B7};
%             obstacles = {B2};
            
            obj = Map(robot, obstacles);
        end
        
        function main()
            obj = Map.init_question_map();
            print_map(obj)
        end
    end
end

