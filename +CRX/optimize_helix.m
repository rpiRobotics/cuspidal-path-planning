
kin = define_CRX;
[p_path_orig, R_path_orig] = example_toolpath.helix(1000);

%% Find a feasible initial guess

for i = 1:100
    [~, p] = fwdkin(kin, [0; rand_angle([5 1])]);
    axang = rand_angle*rand_normal_vec;
    [q_dot_norm, Q_path, G] = path_norm(p, axang, p_path_orig, R_path_orig, kin);

    if q_dot_norm< inf
        disp("Got it")
        [p' axang']
        break
    end
end
%%

initial_guess = [0.1453    0.9265   -0.2172    0.4074    0.1676   -1.2581]';

%% Plot initial guess (q vs lambda)

p = initial_guess(1:3);
axang = initial_guess(4:6);
[q_dot_norm, Q_path, G] = path_norm(p, axang, p_path_orig, R_path_orig, kin);

% plot(squeeze(Q_path(4,:,:))', 'k.')
G = generate_path_graph(Q_path);

plot_path_graph(G, Q_path, 4); hold on;
[P, q_dot_norm] = shortestpath(G, 1, 2);

Q_0 = NaN(height(Q_path), length(Q_path));

W = width(Q_path);
q_disp_ind = 4;
for i = 2:(length(P)-1)
    soln_ind = rem(P(i), W);
    path_ind = (P(i)-soln_ind)/W;
    q_i = Q_path(q_disp_ind, soln_ind, path_ind);
    plot(path_ind, q_i, 'xk');
    Q_0(:,path_ind) = Q_path(:, soln_ind, path_ind);
end
hold off

%% Optimize

options = optimset('PlotFcns',@optimplotfval);
optimized = fminsearch(@(pose)path_norm(pose(1:3), pose(4:6), p_path_orig, R_path_orig, kin), initial_guess, options);

% 125 iterations
% [-0.0023   -0.0023   -1.1021   -0.0151    0.0204   -0.0889]';
%%

function [q_dot_norm, Q_path, G] = path_norm(p, quat_XY, p_path_orig, R_path_orig, kin)
    [R_path, p_path] = CRX.quat_offset_path(p, quat_XY, R_path_orig, p_path_orig);
    Q_path = CRX.generate_Q_path(kin, R_path, p_path);

    G = generate_path_graph(Q_path);
    [~, q_dot_norm] = shortestpath(G, 1, 2);
end