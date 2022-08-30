function [X, Y, Yaw, Vx] = Vehicle_Model(X0, Y0, Yaw0, Vx0, delta, ax, ax_pre, dt, L)
ax = 0.1*ax + 0.9*ax_pre;
Vx = Vx0 + ax*dt;
X = X0 + Vx*cos(Yaw0)*dt;
Y = Y0 + Vx*sin(Yaw0)*dt;
gamma = Vx/L*tan(delta);
Yaw = Yaw0 + gamma*dt;