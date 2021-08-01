classdef Map_Object < handle
    properties
        vertices
        norms
        InOut
        name
    end
    
    methods
        function obj = Map_Object(vertices, InOut, name)
            % assumse the vertices are counter-clock wish order
            set_vertices(obj, vertices);
            obj.InOut = InOut;
            calc_norms(obj);
            obj.name = name;
        end
        
        function set_vertices(obj, vertices)
            obj.vertices = vertices;
            obj.vertices(end+1,:) = vertices(1,:);
        end
        
        function calc_norms(obj)
            % Inout = 1 means out mormals.
            % Inout = -1 means in mormals.
            num = size(obj.vertices,1);
            obj.norms = zeros(num-1, 5);
            for i=1:(num-1)
                x = obj.vertices(i:i+1,1);
                y = obj.vertices(i:i+1,2);
                obj.norms(i,:) = get_norm(x,y,obj.InOut);
            end
            
            function obj_norm = get_norm(x,y,InOut)
                dy = diff(y);
                dx = diff(x);
                m = (dy/dx);
                % Slope of new line
                L = 0.3*sqrt(dy^2+dx^2);
                minv = -1/m;
                if abs(minv) == Inf
                    obj_norm = [atan2d(L*sign(minv)*sign(InOut),0) , mean(x), mean(x), mean(y), mean(y)-L*sign(minv)*sign(InOut)];
                else
                    obj_norm = [atan2d(L*minv*sign(dy)*sign(InOut),L*sign(dy)*sign(InOut)), mean(x), mean(x)-L*sign(dy)*sign(InOut), mean(y), mean(y)-L*minv*sign(dy)*sign(InOut)]; %#ok<*CPROP>
                end
            end
        end
        
        function add_label_to_plot(obj)
            ver_x = obj.vertices(:,1);
            ver_y = obj.vertices(:,2);
            text(mean(ver_x), mean(ver_y), obj.name , 'HorizontalAlignment' , 'center' , 'VerticalAlignment' , 'middle')
        end
        
        function print_obstacle(obj, with_norms)
            ver_x = obj.vertices(:,1);
            ver_y = obj.vertices(:,2);

            if obj.InOut == -1
                fill(ver_x,ver_y,'b');
                line(ver_x, ver_y, 'Color', 'k');
            elseif obj.InOut == 1
                fill(ver_x,ver_y,'r');
                line(ver_x, ver_y, 'Color', 'k');
            else
                fill(ver_x,ver_y,'g', 'facealpha',.3);
                line(ver_x, ver_y, 'Color', 'g', 'LineStyle', '--');
            end
            
            if with_norms
                for i=1:size(obj.norms,1)
                    line(obj.location(1) + [obj.norms(i,2); obj.norms(i,3)], obj.location(2) + [obj.norms(i,4); obj.norms(i,5)], 'Color', 'r');
                end
            end
        end
    end
    
    methods (Static)
        function test_calc_norm()
            figure; hold on;
            vertices = [-5 0; 0 5; 5 0; 0 -5];
            location = [20 20];
            obj = Map_Object(vertices, location, 1);
            print_obstacle(obj, true)
            axis equal
            
            vertices = [0 0; 0 10; 10 10; 10 0];
            location = [-10 5];
            obj = Map_Object(vertices, location, 1);
            print_obstacle(obj, false)
            axis equal
        end
    end
end

