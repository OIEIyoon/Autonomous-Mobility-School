clear all; clc; close all;
%%
ds = 0.005;
X = 0;
Y = 0;
Yaw = 0;
Curv = 0;
map = [0 ; X ; Y ; Yaw ; Curv];
% 
map = Map_Gen_Straight(map, .3, ds);
map = Map_Gen_Curve(map, 1, 135, 1/10, ds);
map = Map_Gen_Curve(map, .7, 75, 1/3, ds);
map = Map_Gen_Curve(map, -.7, 60, 1/3, ds);
map = Map_Gen_Curve(map, .7, 60, 1/3, ds);
map = Map_Gen_Curve(map, -.7, 60, 1/3, ds);
map = Map_Gen_Curve(map, .7, 75, 1/3, ds);
map = Map_Gen_Straight(map, .895, ds);
map = Map_Gen_Curve(map, .8, 180, 1/3, ds);
map = Map_Gen_Curve(map, -.8, 75, 1/10, ds);
map = Map_Gen_Curve(map, .8, 30, 1/10, ds);
map = Map_Gen_Straight(map, 1.6, ds);

%  map = Map_Gen_Straight(map, .5, ds);
%  map = Map_Gen_Curve(map, -.8, 30, 1/10, ds);
%  map = Map_Gen_Curve(map, .8, 30, 1/10, ds);
%  map = Map_Gen_Straight(map, 1.5, ds);
%  map = Map_Gen_Curve(map, -.8, 45, 1/10, ds);
%  map = Map_Gen_Curve(map, .8, 60, 1/10, ds);
%  map = Map_Gen_Curve(map, -.8, 30, 1/10, ds);
%  map = Map_Gen_Straight(map, 1, ds);
%  map = Map_Gen_Curve(map, .6, 90, 1/5, ds);
%  map = Map_Gen_Curve(map, .6, 90, 1/5, ds);
%  map = Map_Gen_Straight(map, 1, ds);
%  map = Map_Gen_Curve(map, -.8, 30, 1/5, ds);
%  map = Map_Gen_Straight(map, 2, ds);
%  map = Map_Gen_Curve(map, .8, 45, 1/10, ds);
%  map = Map_Gen_Straight(map, .5, ds);
%  map = Map_Gen_Curve(map, .6, 60, 1/20, ds);
%  map = Map_Gen_Curve(map, -.6, 120, 1/20, ds);
%  map = Map_Gen_Curve(map, 1, 130, 1/4, ds);
%  map = Map_Gen_Straight(map, 1.185, ds);
%  map = Map_Gen_Curve(map, .6, 110, 1/4, ds);
%  map = Map_Gen_Straight(map, 1.3, ds);


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
plot(path(1,:), path(2,:), '--', 'color', [.98 .98 .5], 'linewidth', 1.5);  hold on
plot([left_lane(1,1), right_lane(1,1)], [left_lane(2,1), right_lane(2,1)], '-','color',[.3 .3 .95], 'linewidth', 15); 
plot(left_lane(1,:), left_lane(2,:), '-', 'color', [.3 .99 .3], 'linewidth', 4);
plot(right_lane(1,:), right_lane(2,:), '-', 'color', [.99 .3 .3], 'linewidth', 4);
axis equal ; grid on; grid minor; axis tight
set(gca,'color',[0.5 0.5 0.5])
save('path_data', 'path', 'left_lane', 'right_lane', 'map');
