function [left_local, right_local, center_local] = Lane_Detection(X, Y, Yaw, left_lane_ROI, right_lane_ROI) 
R = [cos(-Yaw) -sin(-Yaw) ; sin(-Yaw) cos(-Yaw)];
left_local = R*(left_lane_ROI - [X; Y]);
right_local = R*(right_lane_ROI - [X; Y]);
center_local = (left_local + right_local)/2;