clear all; clc; close all;
%%
ds = 0.005;
X = 0;
Y = 0;
Yaw = 0;
Curv = 0;
map = [0 ; X ; Y ; Yaw ; Curv];

map = Map_Gen_Straight(map, .3, ds);
map = Map_Gen_Curve(map, .78, 200, 1/5, ds);
map = Map_Gen_Straight(map, .15, ds);
map = Map_Gen_Curve(map, -.65, 40, 1/3, ds);
map = Map_Gen_Straight(map, .035, ds);
map = Map_Gen_Curve(map, .76, 200, 1/5, ds);
map = Map_Gen_Straight(map, .92, ds);




%%
path = map(2:3,:);


%%
W_road = 0.3/2;
left_lane = [];
right_lane = [];
for i = 2:length(path(1,:))
    tan_vec = path(1:2,i) - path(1:2,i-1);
    nom_vec = [-tan_vec(2) ; tan_vec(1)]/norm(tan_vec);
    left_lane = [left_lane, path(1:2,i) + nom_vec*W_road];
    right_lane = [right_lane, path(1:2,i) - nom_vec*W_road];
end
%%

fig1 = figure('units','normalized','outerposition',[0 0 1 1]);
% path = path - min(right_lane,[],2);
% left_lane = left_lane - min(right_lane,[],2);
% right_lane = right_lane - min(right_lane,[],2);
plot(path(1,:), path(2,:), '--', 'color', [.95 .95 .1], 'linewidth', 1.5);  hold on
plot([left_lane(1,1), right_lane(1,1)], [left_lane(2,1), right_lane(2,1)], '-','color',[.6 .6 .6], 'linewidth', 5); 
plot(left_lane(1,:), left_lane(2,:), '-', 'color', [.9 .4 .4], 'linewidth', 4);
plot(right_lane(1,:), right_lane(2,:), '-', 'color', [.4 .4 .9], 'linewidth', 4);
axis equal 

save('path_data', 'path', 'left_lane', 'right_lane', 'map');
