kin = define_CRX;
CRX.initial_poses;
%% moveL
pose_0 = moveL_pose_0;
[R_path_orig, p_path_orig] = example_toolpath.moveL(eye(3), eye(3), [0;0;0], [0;0;0.5], [], [], 100);

%% Helix
pose_0 = helix_pose_0_B;
[R_path_orig, p_path_orig] = example_toolpath.helix(N = 500);

%% Find a feasible initial guess

for i = 1:100
    [~, p] = fwdkin(kin, [0; rand_angle([5 1])]);
    quat_XY = rand_normal_vec;
    [q_dot_norm, Q_path, G] = CRX.path_norm(p, quat_XY, p_path_orig, R_path_orig, kin);

    if q_dot_norm< inf
        disp("Got it")
        disp(i)
        disp([p' quat_XY'])
        pose_0 = [p' quat_XY']';
        break
    end
end

%% Optimize

options = optimset('PlotFcns',@optimplotfval, 'TolFun', 1e-3, 'TolX', 1e-3);
optimized = fminsearch(@(pose)CRX.path_norm(pose(1:3), pose(4:6), p_path_orig, R_path_orig, kin), pose_0, options);

%%

% pose = pose_0;
pose = optimized;
[q_dot_norm, Q_path, G] = CRX.path_norm(pose(1:3), pose(4:6), p_path_orig, R_path_orig, kin);
graph_path_planning.plot_path_graph(G, Q_path, 2);

%% 
Q_optimal = graph_path_planning.highlight_optimal_path(G, Q_path);
%%
plot(Q_optimal');
