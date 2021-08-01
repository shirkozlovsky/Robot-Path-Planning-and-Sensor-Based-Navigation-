classdef Robot < Map_Object
    properties
        theta
        org_vertices
    end
    
    methods
        function obj = Robot(vertices, theta, name)
            % theta needs to be in degree
            % theta > 0 - counterclockwise
            obj@Map_Object(vertices, -1, name);
            obj.org_vertices = vertices;
            set_theta(obj, theta)
        end
        
        function set_theta(obj, theta)
            obj.theta = theta;
            obj.set_vertices(rotate_robot(obj.org_vertices, theta));
            calc_norms(obj)
            
            function new_vertices = rotate_robot(vertices, theta)
                R = [cosd(theta) -sind(theta); sind(theta) cosd(theta)];
                robot_dim = vertices - vertices(1,:);
                new_vertices = (R*robot_dim')' + vertices(1,:);
            end
        end
    end
    
    methods (Static)
        function test_robot()
            obj = Robot([0 0; 0 5; 2 5; 2 0], -1, -30);
            print_obstacle(obj)
        end
    end
end

