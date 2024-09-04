pose_0 = [0.7013    1.8369   -0.6936  0.3884   -0.6435   -0.2061]';
kin = cuspidal_3R.get_kin;
p_path_orig = example_toolpath.helix();

options = optimset('PlotFcns',@optimplotfval);
optimized = fminsearch(@(pose)path_norm(pose(1:3), pose(4:6), p_path_orig, kin), pose_0, options);

%%

optimized =[-0.19328340028517898563364951769472, 0.15472249611146848824461130789132, -4.4200684503758260746053565526381, 1716.7136288001620414434000849724, 1277.3946329562686514691449701786, -1143.6160441060003449820214882493]'
%%

% pose = pose_0;
pose = optimized;
[q_dot_norm, Q_path, G] = path_norm(pose(1:3), pose(4:6), p_path_orig, kin);
plot_path_graph(G, Q_path, 2);

%% Highlight optimal path
Q_optimal = highlight_optimal_path(optimized, p_path_orig, kin)
%%
plot(Q_optimal');

%% Initial guess path
Q_initial = highlight_optimal_path(pose_0, p_path_orig, kin)
%%
plot(Q_initial');

%% Find a random feasible pose

for i = 1:100
    [~, p] = fwdkin(kin, [0 rand_angle, rand_angle]);
    quat_XY = rand_normal_vec;
    [q_dot_norm, Q_path, G] = path_norm(p, quat_XY, p_path_orig, kin);

    if q_dot_norm< inf
        disp("Got it")
        [p' quat_XY']
        break
    end
end


%%

function [q_dot_norm, Q_path, G] = path_norm(p, quat_XY, p_path_orig, kin)
    p_path = cuspidal_3R.quat_offset_path(p, quat_XY, p_path_orig);
    Q_path = cuspidal_3R.generate_Q_path(kin, p_path);

    G = generate_path_graph(Q_path);
    [~, q_dot_norm] = shortestpath(G, 1, 2);
end

function Q_optimal = highlight_optimal_path(pose, p_path_orig, kin)
    [q_dot_norm, Q_path, G] = path_norm(pose(1:3), pose(4:6), p_path_orig, kin);
    plot_path_graph(G, Q_path, 2); hold on;
    [P, q_dot_norm] = shortestpath(G, 1, 2);
    
    Q_optimal = NaN(height(Q_path), length(Q_path));
    
    W = width(Q_path);
    q_disp_ind = 2;
    for i = 2:(length(P)-1)
        soln_ind = rem(P(i), W);
        path_ind = (P(i)-soln_ind)/W;
        q_i = Q_path(q_disp_ind, soln_ind, path_ind);
        plot(path_ind, q_i, 'xk');
        Q_optimal(:,path_ind) = Q_path(:, soln_ind, path_ind);
    end
    hold off
end