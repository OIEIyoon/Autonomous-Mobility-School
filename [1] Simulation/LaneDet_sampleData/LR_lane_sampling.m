
% N = 20;
% ds = 5;
% s_list = [300 500 700 900 1100 1400];
% ey_dev = [0 0.1 -0.1 0.2 -0.2 0];
% yaw_dev = [5 2 0 -3 -5 -10];
% left_local = [];
% right_local = [];
% for i = 1:length(s_list)
%     s1 = s_list(i);
%     left1 = left_lane(:, s1:ds:s1+ds*N)-path(:,s1)-[0;ey_dev(i)];
%     right1 = right_lane(:, s1:ds:s1+ds*N)-path(:,s1)-[0;ey_dev(i)];
%     yaw = map(4,s1)+yaw_dev(i)*pi/180;
%     R = [cos(-yaw) -sin(-yaw) ; sin(-yaw) cos(-yaw)];
%     left_local = [left_local ; R*left1];
%     right_local = [right_local ; R*right1];
%     
%     figure;
%     plot(left_local(2*(i-1)+1,:), left_local(2*(i-1)+2,:));hold on;
%     plot(right_local(2*(i-1)+1,:), right_local(2*(i-1)+2,:));
%     axis equal;
% end
% save('LR_lane_sample', 'left_local', 'right_local');