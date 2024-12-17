kin = cuspidal_3R.get_kin;
cuspidal_3R.initial_poses;
%% moveL

pose_0 = moveL_pose_0;
[~, p_path_orig] = example_toolpath.moveL([], [], [0;0;0], [0;0;0.5], [], [], 100);

%% Helix

pose_0 = helix_pose_0;
[~, p_path_orig] = example_toolpath.helix(H=1.2, R = 0.4);

%%
options = optimset('PlotFcns',@optimplotfval, 'TolFun', 1e-3, 'TolX', 1e-3);
optimized = fminsearch(@(pose)cuspidal_3R.path_norm(pose(1:3), pose(4:6), p_path_orig, kin), pose_0, options);

%%

% pose = pose_0;
pose = optimized;
[q_dot_norm, Q_path, G] = cuspidal_3R.path_norm(pose(1:3), pose(4:6), p_path_orig, kin);
graph_path_planning.plot_path_graph(G, Q_path, 2);

%% 
Q_optimal = graph_path_planning.highlight_optimal_path(G, Q_path);
%%
plot(Q_optimal');

%% Find a random feasible pose

for i = 1:100
    [~, p] = fwdkin(kin, [0 rand_angle, rand_angle]);
    quat_XY = rand_normal_vec;
    [q_dot_norm, Q_path, G] = cuspidal_3R.path_norm(p, quat_XY, p_path_orig, kin);

    if q_dot_norm < inf
        disp("Got it")
        disp(i)
        disp([p' quat_XY'])
        pose_test = [p' quat_XY']'
        break
    end
end
