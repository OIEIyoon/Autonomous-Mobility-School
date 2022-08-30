function [body, tire] = drawvehicle(X, Y, Yaw, delta)
Lveh = 0.3/2;
Wveh = 0.2/2;
Ta = 0.16/2;
Tb = 0.16/2;
Tlen = 0.07/2;
Twid = 0.03/2;
% X = X + Lveh*cos(Yaw);
% Y = Y + Lveh*sin(Yaw);
plot(X,Y,'.','color', 'm');
R = [cos(Yaw), -sin(Yaw); sin(Yaw), cos(Yaw)];
Rf = [cos(Yaw+delta), -sin(Yaw+delta); sin(Yaw+delta), cos(Yaw+delta)];
Rr = R;
veh_rot = [X;Y] + R*[- Lveh, Lveh, Lveh, -Lveh ; - Wveh, -Wveh, +Wveh, +Wveh];
tire_pos = [X;Y] + R*[- Ta-0.01, Ta, Ta, -Ta-0.01 ; - Tb, -Tb, +Tb, +Tb];
veh_poly = polyshape(veh_rot(1,:), veh_rot(2,:));
body = plot(veh_poly, 'FaceColor', [.6 .6 1], 'FaceAlpha', 1, 'Linewidth', 1, 'EdgeColor', 'k');
lf_rot = tire_pos(:,3) + Rf*[- Tlen, Tlen, Tlen, -Tlen ; - Twid, -Twid, +Twid, +Twid];
rf_rot = tire_pos(:,2) + Rf*[- Tlen, Tlen, Tlen, -Tlen ; - Twid, -Twid, +Twid, +Twid];
lr_rot = tire_pos(:,4) + R*[- Tlen, Tlen, Tlen, -Tlen ; - Twid, -Twid, +Twid, +Twid];
rr_rot = tire_pos(:,1) + R*[- Tlen, Tlen, Tlen, -Tlen ; - Twid, -Twid, +Twid, +Twid];

lf_poly = polyshape(lf_rot(1,:), lf_rot(2,:));
rf_poly = polyshape(rf_rot(1,:), rf_rot(2,:));
lr_poly = polyshape(lr_rot(1,:), lr_rot(2,:));
rr_poly = polyshape(rr_rot(1,:), rr_rot(2,:));

lf = plot(lf_poly, 'FaceColor', 'k', 'FaceAlpha', .8, 'Linewidth', 1, 'EdgeColor', 'k');
rf = plot(rf_poly, 'FaceColor', 'k', 'FaceAlpha', .8, 'Linewidth', 1, 'EdgeColor', 'k');
lr = plot(lr_poly, 'FaceColor', 'k', 'FaceAlpha', .8, 'Linewidth', 1, 'EdgeColor', 'k');
rr = plot(rr_poly, 'FaceColor', 'k', 'FaceAlpha', .8, 'Linewidth', 1, 'EdgeColor', 'k');
tire = [lf rf lr rr];
