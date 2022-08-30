path_len = length(path(1,:));
N = 40;
dn = 5;
dt = 0.05;
T = 28;
L = 0.16;
d1 = 0.4;
h1 = 0.1;
d2 = 1500;
h2 = -100;

iter = floor(T/dt)+1;
t = 0:dt:T;
X = zeros(iter,1);
Y = zeros(iter,1);
Yaw = zeros(iter,1);
Ax = zeros(iter,1);
ax_nom = zeros(iter,1);
Ay = 0;
Vx = zeros(iter,1);
Vx_des = zeros(iter,1);
curv_max = zeros(iter,1);

e_y = zeros(iter,1);
e_yaw = zeros(iter,1);
delta = zeros(iter,1);
idx = zeros(iter,1);
left_lane_ROI = zeros(iter,N);
right_lane_ROI = zeros(iter,N);

TargetX = -0.0130;
TargetY = 0;
clearance = zeros(iter,1);
cl_des = zeros(iter,1);
isTarget = zeros(iter,1);
BSC = zeros(iter,1);