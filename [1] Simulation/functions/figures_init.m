figure('units','normalized','outerposition',[0 0.02 1 0.98]);

sub_traj = subplot(5,4,[1,2,5,6,9,10]);
plot(path(1,:), path(2,:), '--', 'color', [.95 .95 .1], 'linewidth', 1.5);  hold on;
sub_traj.Color = [.2 .2 .2];
plot(left_lane(1,:), left_lane(2,:), '-', 'color', [.4 .9 .4], 'linewidth', 4);
plot(right_lane(1,:), right_lane(2,:), '-', 'color', [.9 .4 .4], 'linewidth', 4);
grid on;
axis equal; 
xlim([min(right_lane(1,:))-0.3, max(right_lane(1,:))+0.3]);
ylim([min(right_lane(2,:))-0.3, max(right_lane(2,:))+0.3]);
h_left_ROI = animatedline(sub_traj, 'MaximumNumPoints', N/2, 'Marker', '.', 'Color', 'w', 'MarkerSize', 20);
h_right_ROI = animatedline(sub_traj, 'MaximumNumPoints', N/2, 'Marker', '.', 'Color', 'w', 'MarkerSize', 20);


sub_ey = subplot(5,4,3);
hold on; grid on;
xlabel("시간 [s]"); ylabel("횡 오차 [m]");
yticks(-0.4:0.2:0.4)
axis([0 T -0.4 0.4]);
h_ey = animatedline(sub_ey, 'LineWidth', 1.5);


sub_eyaw = subplot(5,4,4);
hold on; grid on;
xlabel("시간 [s]"); ylabel("각 오차 [deg]");
yticks(-1:0.5:1)
axis([0 T -1 1]);
h_eyaw = animatedline(sub_eyaw, 'LineWidth', 1.5);


sub_acc = subplot(5,4,7);
hold on; grid on;
xlabel("시간 [s]"); ylabel("가속도 [m/s^2]");
title("파랑 : 종 방향 / 빨강 : 횡 방향",'fontweight', 'b')
yticks(-2:1:2)
axis([0 T -2 2]);
p_ax = plot(sub_acc, Ax(1), 'b-', 'LineWidth', 1.5);
p_ay = plot(sub_acc, Ay, 'r:', 'LineWidth', 1.5);
h_ax = animatedline(sub_acc, 'LineWidth', 1.5, 'LineStyle', '-', 'color', 'b');
h_ay = animatedline(sub_acc, 'LineWidth', 1.5, 'LineStyle', ':', 'color', 'r');
% legend([p_ax, p_ay], {"종 방향", "횡 방향"}, 'location', 'northeast'); legend('boxoff')


sub_delta = subplot(5,4,8);
hold on; grid on;
xlabel("시간 [s]"); ylabel("전륜 조향각 [deg]");
axis([0 T -30 30]);
% yticks(-30:10:30);
h_delta = animatedline(sub_delta, 'LineWidth', 1.5);


sub_vx = subplot(5,4,11);
hold on; grid on;
xlabel("시간 [s]"); ylabel("속도 [m/s]");
title("검정 : 실제 / 파랑 : 목표",'fontweight', 'b')
yticks(0:0.2:1)
axis([0 T 0 1.2]);
p_vx = plot(sub_vx, Vx(1), 'k-', 'LineWidth', 1.5);
p_vxdes = plot(sub_vx, Vx_des(1), 'b:', 'LineWidth', 1.5);
h_vx = animatedline(sub_vx, 'LineWidth', 1.5);
h_vxdes = animatedline(sub_vx, 'LineWidth', 1.5, 'Color', 'b', 'LineStyle', ':');
% legend([p_vx, p_vxdes], {"실제", "목표"}, 'location', 'northeast'); legend('boxoff')


sub_clearance = subplot(5,4,12);
hold on; grid on;
xlabel("시간 [s]"); ylabel("정지선까지 거리 [m]");
title("검정 : 실제 / 파랑 : 목표",'fontweight', 'b')
axis([0 T 0 5]);
p_cl = plot(sub_clearance,clearance(1), 'k-', 'LineWidth', 1.5);
p_cldes = plot(sub_clearance,cl_des(1), 'b:', 'LineWidth', 1.5);
h_cl = animatedline(sub_clearance, 'LineWidth', 1.5);
h_cldes = animatedline(sub_clearance,'LineWidth', 1.5, 'Color', 'b', 'LineStyle', ':');
% legend([p_cl, p_cldes], {"실제", "목표"}, 'location', 'northeast'); legend('boxoff')


sub_cam = subplot(5,4,[14,15,18,19]);
plot3(path(1,:), path(2,:), zeros(1, size(path,2)), '--', 'color', [.95 .95 .1], 'LineWidth', 1.5); hold on;
plot3(left_lane(1,:), left_lane(2,:), zeros(1, size(left_lane,2)), '.-', 'color', [.4 .9 .4], 'LineWidth', 1.5, 'markersize', 30);
plot3(right_lane(1,:), right_lane(2,:), zeros(1, size(left_lane,2)), '.-', 'color', [.9 .4 .4], 'LineWidth', 1.5, 'markersize', 30);
h_left_ROI2 = animatedline(sub_cam, 'MaximumNumPoints', N/2, 'Marker', '.', 'Color', 'w', 'MarkerSize', 50);
h_right_ROI2 = animatedline(sub_cam, 'MaximumNumPoints', N/2, 'Marker', '.', 'Color', 'w', 'MarkerSize', 50);
[xmesh, ymesh] = meshgrid(-130:160, -110:150);
zmesh = -0.1*ones(size(xmesh));
surf(xmesh, ymesh, zmesh, 'FaceColor', [.2 .2 .2], 'EdgeColor', [.3 .3 .3]);
axis equal vis3d;
camproj perspective
campos([X(1)-cos(Yaw(1))*d1, Y(1)-sin(Yaw(1))*d1, h1])
camtarget([X(1)+cos(Yaw(1))*d2, Y(1)+sin(Yaw(1))*d2, h2])
camzoom(0.5)