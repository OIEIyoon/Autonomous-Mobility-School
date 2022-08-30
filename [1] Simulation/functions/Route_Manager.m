function [e_y, e_yaw, yaw_road, curv_road, curv_max] = Route_Manager(center_local, Yaw, idx, map)
x_seq = center_local(1,:);
y_seq = center_local(2,:);
local_len = length(x_seq);
p1 = polyfit(x_seq(1:floor(local_len/4)), y_seq(1:floor(local_len/4)), 2);
e_y = -p1(3);
e_yaw = -p1(2);
yaw_road = Yaw - e_yaw;
curv_road = 2*p1(1)/(1+p1(2)^2)^1.5;

map_ext = [map map];
idx_preview = idx + 100;
curv_max = max(abs(map_ext(5,idx:idx_preview)));