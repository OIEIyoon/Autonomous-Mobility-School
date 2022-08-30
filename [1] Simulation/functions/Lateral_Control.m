function delta = Lateral_Control(e_y, e_yaw, Vx)



%%%%%%%% 주행 제어 (횡 제어) %%%%%%%
%%%%%% 아래 코드를 완성하세요 %%%%%% 
% 명령 조향 각 : delta
% 횡 오차 : e_y
% 각 오차 : e_yaw
% gain : k1, k2

k1 = 0;
k2 = 0;
delta = 0;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


k1 = 10;
k2 = 8;
delta = -k1*e_y - k2*e_yaw;

L = 0.16;
Ay_max = 1.2;
u_lim = min(25 * pi/180, atan(Ay_max*L/Vx^2));
delta = max(-u_lim, min(u_lim, delta));