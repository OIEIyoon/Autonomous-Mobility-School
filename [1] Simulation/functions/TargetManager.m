function [isTarget, clearance] = TargetManager(TargetX, TargetY, X, Y, map)
path = map(2:3,:);
path_ext = [path, path];
[~, idx_ev] = min(vecnorm(path_ext-[X;Y],1));
[~, idx_tv] = min(vecnorm(path_ext-[TargetX;TargetY],1));
s_ev = map(1,idx_ev);
s_tv = map(1,idx_tv);

clearance = s_tv - s_ev;
isTarget = (clearance < 2);