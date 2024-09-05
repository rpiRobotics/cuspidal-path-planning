kin = cuspidal_3R.get_kin;

%% moveL

pose_0 = [0.2;0.2;0.2; 1;0;0];
[~, p_path_orig] = example_toolpath.moveL([], [], [0;0;0], [0;0;0.5], [], [], 100);

%% Helix

pose_0 = [0.7013    1.8369   -0.6936  0.3884   -0.6435   -0.2061]';
[~, p_path_orig] = example_toolpath.helix();

% optimized =[-0.19328340028517898563364951769472, 0.15472249611146848824461130789132, -4.4200684503758260746053565526381, 1716.7136288001620414434000849724, 1277.3946329562686514691449701786, -1143.6160441060003449820214882493]'


%%
options = optimset('PlotFcns',@optimplotfval);
optimized = fminsearch(@(pose)cuspidal_3R.path_norm(pose(1:3), pose(4:6), p_path_orig, kin), pose_0, options);

%%

% pose = pose_0;
pose = optimized;
[q_dot_norm, Q_path, G] = path_norm(pose(1:3), pose(4:6), p_path_orig, kin);
graph_path_planning.plot_path_graph(G, Q_path, 2);

%% 
Q_optimal = graph_path_planning.highlight_optimal_path(G, Q_path);
%%
plot(Q_optimal');

%% Find a random feasible pose

for i = 1:100
    [~, p] = fwdkin(kin, [0 rand_angle, rand_angle]);
    quat_XY = rand_normal_vec;
    [q_dot_norm, Q_path, G] = path_norm(p, quat_XY, p_path_orig, kin);

    if q_dot_norm < inf
        disp("Got it")
        disp(i)
        disp([p' quat_XY'])
        break
    end
end
