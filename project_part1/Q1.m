%% Initialization
close all; clear; clc;
warning('off' ,'images:initSize:adjustingMag');
robot = {[0 0; 8 0; 8 1; 0 1] + [4 24], 'robot'};
B1 = {[0 0; 10 0; 10 1; 0 1]+[0 18], 'CB'};

%% calc all layers
[matrix_map, maps] = Q1_(robot,B1);

%% show and save relevant layers
maps{1}.Visible = 'on';
matrix_map{1}.Visible = 'on';
maps{8}.Visible = 'on';
matrix_map{8}.Visible = 'on';
maps{16}.Visible = 'on';
matrix_map{16}.Visible = 'on';
maps{32}.Visible = 'on';
matrix_map{32}.Visible = 'on';

saveas(maps{1},'graphs\Q1\map1.jpg');
saveas(matrix_map{1},'graphs\Q1\matrix_map1.jpg');
saveas(maps{8},'graphs\Q1\map8.jpg');
saveas(matrix_map{8},'graphs\Q1\matrix_map8.jpg');
saveas(maps{16},'graphs\Q1\map16.jpg');
saveas(matrix_map{1},'graphs\Q1\matrix_map16.jpg');
saveas(maps{32},'graphs\Q1\map32.jpg');
saveas(matrix_map{32},'graphs\Q1\matrix_map32.jpg');

%%
function [matrix_map, maps] = Q1_(robot,obstacle)
dt= 360/32;
maps = cell(1,32);
matrix_map = cell(1,32);
i=0;

obstacle_obj = {obstacle};

% initilaize map object
obj = Map(robot, 0, obstacle_obj);

% calc CB for each theta and get the discrete and continues maps
for theta=0:dt:360-dt
    i = i + 1;
    obj.robot.set_theta(theta);
    obj.calc_c_obstacles;
    maps{i} = obj.get_map('off');
    matrix_map{i} = get_map_matrix(obj, 'off');
end

end

