pose_0 = [0.7013    1.8369   -0.6936   -1.6368   -0.5241    1.5935]';
kin = cuspidal_3R.get_kin;
p_path_orig = example_toolpath.helix();

options = optimset('PlotFcns',@optimplotfval);
pose = fminsearch(@(pose)path_norm(pose(1:3), pose(4:6), p_path_orig, kin), pose_0, options);
% 0.8462    2.2120    0.0234    0.1465   -2.5520   -2.4581
%%

% pose = pose_0;
pose = optimized;
[q_dot_norm, Q_path, G] = path_norm(pose(1:3), pose(4:6), p_path_orig, kin);
plot_path_graph(G, Q_path, 2);

%% Find a random feasible pose

for i = 1:100
    [~, p] = fwdkin(kin, [0 rand_angle, rand_angle]);
    axang = rand_angle*rand_normal_vec;
    [q_dot_norm, Q_path, G] = path_norm(p, axang, p_path_orig, kin);

    if q_dot_norm< inf
        disp("Got it")
        p
        axang
        break
    end
end


%%

function [q_dot_norm, Q_path, G] = path_norm(p, axang, p_path_orig, kin)
    R = rot(axang/norm(axang), norm(axang));
    p_path = p + R*p_path_orig;
    Q_path = cuspidal_3R.generate_Q_path(kin, p_path);

    G = generate_path_graph(Q_path);
    [~, q_dot_norm] = shortestpath(G, 1, 2);
end