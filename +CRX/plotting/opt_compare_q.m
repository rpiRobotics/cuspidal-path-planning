% Plot q(k) vs k before and after optimization
kin = define_CRX;
CRX.initial_poses;
CRX.optimized_poses;

%% Intial / optimized poses and original toolpath

pose_0 = helix_pose_0_A;
optimized = helix_optimized_A;

[R_path_orig, p_path_orig] = example_toolpath.helix(N=200);

%% Q for initial pose

[R_path, p_path] = graph_path_planning.quat_offset_path(pose_0(1:3), pose_0(4:6), R_path_orig, p_path_orig);
Q_path = CRX.generate_Q_path(kin, R_path, p_path);
G = graph_path_planning.generate_path_graph(Q_path);
Q_optimal_initial = graph_path_planning.highlight_optimal_path(G, Q_path);

%% Q for optimized poes

[R_path, p_path] = graph_path_planning.quat_offset_path(optimized(1:3), optimized(4:6), R_path_orig, p_path_orig);
Q_path = CRX.generate_Q_path(kin, R_path, p_path);
G = graph_path_planning.generate_path_graph(Q_path);
Q_optimal_optimized = graph_path_planning.highlight_optimal_path(G, Q_path);

%% Plot

lambda = linspace(0, 1, width(Q_optimal_optimized))

h_fig = figure(10);
tiledlayout(1,1,'TileSpacing','none','Padding','compact');
nexttile
figure_size = 2*[2.5 2.45];
set(h_fig, "Units", "inches")
pos_old = h_fig.OuterPosition;
if ~all(pos_old(3:4) == figure_size)
set(h_fig, "OuterPosition", [pos_old(1:2)-figure_size+pos_old(3:4) figure_size])
end
set(h_fig, "Units", "pixels")
findfigs

Q_optimal_initial_unwrap = unwrap(Q_optimal_initial')';
plot(lambda, Q_optimal_initial_unwrap'); hold on
set(gca,'ColorOrderIndex',1); % reset color order
plot(lambda, Q_optimal_initial_unwrap'+2*pi); set(gca,'ColorOrderIndex',1);
plot(lambda, Q_optimal_initial_unwrap'+4*pi); set(gca,'ColorOrderIndex',1);
plot(lambda, Q_optimal_initial_unwrap'+6*pi); set(gca,'ColorOrderIndex',1);
plot(lambda, Q_optimal_initial_unwrap'+8*pi); set(gca,'ColorOrderIndex',1);
plot(lambda, Q_optimal_initial_unwrap'+10*pi); set(gca,'ColorOrderIndex',1);

Q_optimal_optimized_unwrap = unwrap(Q_optimal_optimized')';
plot(lambda, Q_optimal_optimized_unwrap', LineWidth=4); set(gca,'ColorOrderIndex',1);
plot(lambda, Q_optimal_optimized_unwrap' + -2*pi, LineWidth=4); set(gca,'ColorOrderIndex',1);
plot(lambda, Q_optimal_optimized_unwrap' + 2*pi, LineWidth=4); set(gca,'ColorOrderIndex',1);
plot(lambda, Q_optimal_optimized_unwrap' + 4*pi, LineWidth=4); set(gca,'ColorOrderIndex',1);
plot(lambda, Q_optimal_optimized_unwrap' + 6*pi, LineWidth=4); set(gca,'ColorOrderIndex',1);
plot(lambda, Q_optimal_optimized_unwrap' + 8*pi, LineWidth=4); set(gca,'ColorOrderIndex',1);
plot(lambda, Q_optimal_optimized_unwrap' + 10*pi, LineWidth=4); set(gca,'ColorOrderIndex',1);


hold off

xlabel("$\lambda/L$", Interpreter='latex');
ylabel("$q_i$", Interpreter='latex');

fontsize(2*8, 'points')
xaxisproperties= get(gca, 'XAxis');
xaxisproperties.TickLabelInterpreter = 'latex';
yaxisproperties= get(gca, 'YAxis');
yaxisproperties.TickLabelInterpreter = 'latex';

ylim([-pi, pi]);

%%  Plot only one joint angle rather than all 6

joint_display = 5;

lambda = linspace(0, 1, width(Q_optimal_optimized));

h_fig = figure(10);
tiledlayout(1,1,'TileSpacing','none','Padding','compact');
nexttile
figure_size = 2*[2.5 2.45];
set(h_fig, "Units", "inches")
pos_old = h_fig.OuterPosition;
if ~all(pos_old(3:4) == figure_size)
set(h_fig, "OuterPosition", [pos_old(1:2)-figure_size+pos_old(3:4) figure_size])
end
set(h_fig, "Units", "pixels")
findfigs
set(h_fig, 'renderer', 'painters')

Q_optimal_initial_unwrap = unwrap(Q_optimal_initial')';
plot(lambda, Q_optimal_initial_unwrap(joint_display,:)', 'k'); hold on

Q_optimal_optimized_unwrap = unwrap(Q_optimal_optimized')';
plot(lambda, Q_optimal_optimized_unwrap(joint_display,:)', 'k', LineWidth=4);
hold off

xlabel("$\lambda/L$", Interpreter='latex');
ylabel("$q_"+joint_display+"$", Interpreter='latex');

fontsize(2*8, 'points')
xaxisproperties= get(gca, 'XAxis');
xaxisproperties.TickLabelInterpreter = 'latex';
yaxisproperties= get(gca, 'YAxis');
yaxisproperties.TickLabelInterpreter = 'latex';

% ylim([-pi, pi])
yticks([pi/4, pi/2, 3*pi/4]);
yticklabels({"$\frac{\pi}{4}$", "$\frac{\pi}{2}$", "$\frac{3\pi}{4}$"})

axis padded
xlim([0 1])

%%
diagrams.save(gcf, 'opt_before_after_q_CRX')