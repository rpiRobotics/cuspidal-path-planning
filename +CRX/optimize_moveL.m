pose_0 = [0.2;0.2;0.2; 1;0;0];
kin = define_CRX;

options = optimset('PlotFcns',@optimplotfval);
optimized = fminsearch(@(pose)moveL_norm(pose(1:3), pose(4:6), kin), pose_0, options);

%%
% pose = 
%%
pose = pose_0;
%%
[q_dot_norm, Q_path, G] = moveL_norm(pose(1:3), pose(4:6), kin);
%%
plot(permute(Q_path(3,:,:), [3 2 1]))
%% 
plot(diff(unwrap(permute(Q_path(:,2,:), [3 1 2]))))

%%

function [q_dot_norm, Q_path, G] = moveL_norm(p, quat_XY, kin)
    R = quat2rotm([quat_XY' 0]); % Auto-normalize

    p_A = R*p;
    p_B = R*(p + [0;0;0.5]);


    N = 100;
    [R_path, T_path] = CRX.generate_moveL(R, R, p_A, p_B, N);
    Q_path = CRX.generate_Q_path(kin, R_path, T_path);

    G = generate_path_graph(Q_path);
    [~, q_dot_norm] = shortestpath(G, 1, 2);
end