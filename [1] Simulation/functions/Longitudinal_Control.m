  function [Ax, Vx_des, d_des, BSC, ax_nom] = Longitudinal_Control(Ax_pre, Vx, dt, curv_road, isTarget, d, BSC_pre, ax_nom_pre)
Vx_max = 1;
d_min = 0.3;
ax_set = -0.5;
Init = 0;
Ay_max = .5;
curvature = abs(curv_road);



%%%%%% 곡률 기반 속도 추종 제어 %%%%%
%%%%%%  아래 코드를 완성하세요 %%%%%% 

% 명령 가속도 : a1
% 현재 속도 : Vx
% 목표 속도 : Vx_des
% 최대 속도 : Vx_max
% 최대 횡 가속도 : Ay_max (= 1)
% 도로 최대 곡률 : curvature (곡선 도로 반경의 역수)
% gain : k3
% 루트 : sqrt()
% 제곱 : ^2

Vx_des = sqrt(min(Vx_max^2, Ay_max/curvature));

k3 = 2.5;
a1 = k3*(Vx_des-Vx);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



if isTarget == 1
    if  BSC_pre == 1
        BSC = 1;
    else
        if (Vx^2/(-2*ax_set) + d_min > d)
            Init = 1;
            BSC = 1;
        else 
            BSC = 0;
        end
    end
else
    BSC = 0;
end

if Init == 1
    ax_nom = -Vx^2/2/(d-d_min);
else
    ax_nom = ax_nom_pre;
end


if BSC == 1
    
    
    %%%%%%%%%% 제동 정차 제어 %%%%%%%%%%
    %%%%%% 아래 코드를 완성하세요 %%%%%% 
    
    % 필요 가속도 : ax_nom
    % 명령 가속도 : a2
    % 현재 속도 : Vx
    % 현재 차간 거리 : d
    % 목표 차간 거리 : d_des
    % 최소 차간 거리 : d_min (= 0.3)
    % gain : k4
        
    d_des = -Vx^2/(2*ax_nom) + d_min;
    k4 = -5;
    a2 = ax_nom + k4*(d_des-d);

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    
    
    Ax = min([a1, a2, 0]);
    Vx_des = 0;
else
    d_des = NaN;
    Ax = a1;
end

% s_max = 2*dt;
% Ax = Ax_pre + max(-s_max,min(s_max,Ax-Ax_pre));