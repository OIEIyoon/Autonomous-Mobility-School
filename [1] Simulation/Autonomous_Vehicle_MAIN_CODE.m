clear; close all; clc;
%% AddPath / Loading
addpath('functions');
addpath('MapGeneration+Smoothing');
load path_data.mat

%% Parameters Initialization
paramas_init

%% Plot Initialization
figures_init

%% Initial Condition
Y(1) = 0.0;
Vx(1) = .5;
Yaw(1) = 0*pi/180;


%% Simulation
for i = 2:iter
    % Lane Detection
    [left_lane_ROI(2*i-1:2*i,:), right_lane_ROI(2*i-1:2*i,:), idx(i)] = Lane_ROI(path, left_lane, right_lane, X(i-1), Y(i-1), dn, N);
    [left_local, right_local, center_local] = Lane_Detection(X(i-1), Y(i-1), Yaw(i-1), left_lane_ROI(2*i-1:2*i,:), right_lane_ROI(2*i-1:2*i,:));
    
    % Route Manager + Error Calculator
    [e_y(i), e_yaw(i), yaw_road, curv_road, curv_max(i)] = Route_Manager(center_local, Yaw(i-1), idx(i), map);
    
    % Target Manager
    [isTarget(i), clearance(i)] = TargetManager(TargetX, TargetY, X(i-1), Y(i-1), map);
    
    % Longi + Lat Control
    [Ax(i), Vx_des(i), cl_des(i), BSC(i), ax_nom(i)] = Longitudinal_Control(Ax(i-1), Vx(i-1), dt, curv_max(i), isTarget(i), clearance(i),  BSC(i-1), ax_nom(i-1));
    delta(i) = Lateral_Control(e_y(i), e_yaw(i), Vx(i-1));
    
    % Vehicle Model
    [X(i), Y(i), Yaw(i), Vx(i)] = Vehicle_Model(X(i-1), Y(i-1), Yaw(i-1), Vx(i-1), delta(i), Ax(i), Ax(i-1), dt, L);
end

%% Plot
idx_temp = 0;
for i = 2:4:iter
    subplot(5,4,[1,2,5,6,9,10]);
    addpoints(h_left_ROI, left_lane_ROI(2*i-1,1:2:N), left_lane_ROI(2*i,1:2:N));
    addpoints(h_right_ROI, right_lane_ROI(2*i-1,1:2:N), right_lane_ROI(2*i,1:2:N));
    if isTarget(i) == 0
        plot([left_lane(1,1), right_lane(1,1)], [left_lane(2,1), right_lane(2,1)], '-','color', [.4 .4 .9], 'linewidth', 5); 
    else
        plot([left_lane(1,1), right_lane(1,1)], [left_lane(2,1), right_lane(2,1)], '-','color', 'w', 'linewidth', 5); 
    end
    [body, tire] = drawvehicle(X(i), Y(i), Yaw(i), delta(i));
    
    addpoints(h_ey, t(i), e_y(i));
    
    addpoints(h_eyaw, t(i), e_yaw(i));
    
    addpoints(h_delta, t(i), 180/pi*delta(i));
    
    addpoints(h_ax, t(i), Ax(i));
    addpoints(h_ay, t(i), -Vx(i)^2/L*tan(delta(i)));
        
    addpoints(h_cl, t(i), clearance(i));
    addpoints(h_cldes, t(i), cl_des(i));
    
    addpoints(h_vx, t(i), Vx(i));
    addpoints(h_vxdes, t(i), Vx_des(i));
    
    subplot(5,4,[14,15,18,19]);
    if isTarget(i) == 0
        plot([left_lane(1,1), right_lane(1,1)], [left_lane(2,1), right_lane(2,1)], '-','color', [.4 .4 .9], 'linewidth', 10); 
    else
        plot([left_lane(1,1), right_lane(1,1)], [left_lane(2,1), right_lane(2,1)], '-','color', 'w', 'linewidth', 10); 
    end
    addpoints(h_left_ROI2, left_lane_ROI(2*i-1,1:2:N), left_lane_ROI(2*i,1:2:N));
    addpoints(h_right_ROI2, right_lane_ROI(2*i-1,1:2:N), right_lane_ROI(2*i,1:2:N));
    campos([X(i)-cos(Yaw(i))*d1, Y(i)-sin(Yaw(i))*d1, h1])
    camtarget([X(i)+cos(Yaw(i))*d2, Y(i)+sin(Yaw(i))*d2, h2])
    drawnow limitrate
    
%     pause(1/60);
    
    if idx(i) < idx_temp || Vx(i) < 0.01
        break;
    end
    delete(body);
    delete(tire);
    idx_temp = idx(i);
end