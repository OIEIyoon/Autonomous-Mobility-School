function [map_out] = Map_Gen_Straight(map_in, L, ds)
    s = map_in(1,:);
    X = map_in(2,:);
    Y = map_in(3,:);
    Yaw = map_in(4,:);
    Curv = map_in(5,:);
    N = floor(L/ds);
    
    for i = 1:N
        Curv = [Curv, 0];
        Yaw = [Yaw, Yaw(end) + Curv(end)*ds];
        s = [s, s(end) + ds];
        X = [X, X(end) + cos(Yaw(end))*ds];
        Y = [Y, Y(end) + sin(Yaw(end))*ds];
    end
    map_out = [s ; X ; Y ; Yaw ; Curv];
end


