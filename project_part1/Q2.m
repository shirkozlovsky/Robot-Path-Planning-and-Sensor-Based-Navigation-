%% Initialization
warning('off' ,'images:initSize:adjustingMag');
close all; clear; clc;
robot = {[0 0; 8 0; 8 1; 0 1] + [4 24], 'robot'};
B0_1 = {[0 0; 32 0; 32 1; 0 1]+[0 29], 'B_{01}'};
B0_2 = {[0 0; 1 0; 1 30; 0 30]+[0 0], 'B_{02}'};
B0_3 = {[0 0; 32 0; 32 1; 0 1]+[0 0], 'B_{03}'};
B0_4 = {[0 0; 1 0; 1 30; 0 30]+[31 0], 'B_{04}'};

B1 = {[0 0; 10 0; 10 1; 0 1]+[0 18], 'B_1'};
B2 = {[0 0; 1 0; 1 12; 0 12]+[17 17], 'B_2'};
B3 = {[0 0; 7 0; 7 1; 0 1]+[25 18], 'B_3'};
B4 = {[0 0; 19 0; 19 1; 0 1]+[0 14], 'B_4'};
B5 = {[0 0; 8 0; 8 3; 0 3]+[24 13], 'B_5'};
B6 = {[0 0; 2 0; 2 1; 0 1]+[10 19], 'B_6'};
B7 = {[0 0; 2 0; 2 1; 0 1]+[23 19], 'B_7'};

obstacles = {B0_1, B0_2, B0_3, B0_4, B1, B2, B3, B4, B5, B6, B7};

%% calc all layers
[matrix_map, maps] = Q2_(robot,obstacles);
maps{1}.Visible = 'on';
matrix_map{1}.Visible = 'on';
maps{8}.Visible = 'on';
matrix_map{8}.Visible = 'on';
maps{16}.Visible = 'on';
matrix_map{16}.Visible = 'on';
maps{32}.Visible = 'on';
matrix_map{32}.Visible = 'on';

%% show and save relevant layers
saveas(maps{1},'graphs\Q2\map1.jpg');
saveas(matrix_map{1},'graphs\Q2\matrix_map1.jpg');
saveas(maps{8},'graphs\Q2\map8.jpg');
saveas(matrix_map{8},'graphs\Q2\matrix_map8.jpg');
saveas(maps{16},'graphs\Q2\map16.jpg');
saveas(matrix_map{16},'graphs\Q2\matrix_map16.jpg');
saveas(maps{32},'graphs\Q2\map32.jpg');
saveas(matrix_map{32},'graphs\Q2\matrix_map32.jpg');

%%
function [matrix_map, maps] = Q2_(robot, obstacles)
dt= 360/32;
maps = cell(1,32);
matrix_map = cell(1,32);
i=0;

% initilaize map object
obj = Map(robot, 0, obstacles);

% calc CB for each theta and get the discrete and continues maps
for theta=0:dt:360-dt
    i = i + 1;
    obj.robot.set_theta(theta);
    obj.calc_c_obstacles;
    maps{i} = obj.get_map('off');
    matrix_map{i} = get_map_matrix(obj, 'off');
end

end