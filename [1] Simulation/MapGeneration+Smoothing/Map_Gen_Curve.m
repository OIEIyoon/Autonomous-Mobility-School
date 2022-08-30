function [map_out] = Map_Gen_Curve(map_in, R, corner_ang_deg, ratio, ds)
    s = map_in(1,:);
    X = map_in(2,:);
    Y = map_in(3,:);
    Yaw = map_in(4,:);
    Curv = map_in(5,:);
    
    corner_ang_rad = corner_ang_deg*pi/180;
    L = corner_ang_rad * abs(R) / (1 + ratio);
    L1 = L*ratio;
    N = floor((2*L1 + L)/ds);
    
    for i = 1:N
        if i < floor(L1/ds)
            Curv = [Curv, 1/(R*L1) * ds * i];
        elseif i > floor((L1 + L)/ds)
            Curv = [Curv, 1/R - 1/(R*L1) * ds * (i - floor((L1+L)/ds))];
        else
            Curv = [Curv, 1/R];
        end
        Yaw = [Yaw, Yaw(end) + Curv(end)*ds];
        s = [s, s(end) + ds];
        X = [X, X(end) + cos(Yaw(end))*ds];
        Y = [Y, Y(end) + sin(Yaw(end))*ds];
    end
    map_out = [s ; X ; Y ; Yaw ; Curv];
end


