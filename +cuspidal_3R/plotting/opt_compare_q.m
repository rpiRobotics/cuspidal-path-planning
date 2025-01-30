% Plot q(k) vs k before and after optimization



kin = cuspidal_3R.get_kin();
q = zeros([3 1]);
cuspidal_3R.initial_poses;
cuspidal_3R.optimized_poses;

%% Helix
[~, p_path_orig] = example_toolpath.helix();
pose_0 = helix_pose_0_A;
optimized = helix_optimized_A;

[~, p_path] = graph_path_planning.quat_offset_path(pose_0(1:3), pose_0(4:6), [], p_path_orig);
Q_path = cuspidal_3R.generate_Q_path(kin, p_path);
G = graph_path_planning.generate_path_graph(Q_path);
Q_optimal_initial = graph_path_planning.highlight_optimal_path(G, Q_path);

[~, p_path] = graph_path_planning.quat_offset_path(optimized(1:3), optimized(4:6), [], p_path_orig);
Q_path = cuspidal_3R.generate_Q_path(kin, p_path);
G = graph_path_planning.generate_path_graph(Q_path);
Q_optimal_optimized = graph_path_planning.highlight_optimal_path(G, Q_path);


%%
lambda = linspace(0, 1, width(Q_optimal_optimized))

h_fig = figure(10); delete(findall(gcf,'type','annotation'));
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

colororder([diagrams.colors.black; diagrams.colors.blue; diagrams.colors.green]);

plot(lambda, Q_optimal_initial'); hold on
set(gca,'ColorOrderIndex',1); % reset color order
Q_optimal_optimized_wrap = unwrap(Q_optimal_optimized')';
plot(lambda, Q_optimal_optimized_wrap', LineWidth=4); hold off

xlabel("$\lambda/L$", Interpreter='latex');
ylabel("$q_i$", Interpreter='latex');

fontsize(2*8, 'points')
xaxisproperties= get(gca, 'XAxis');
xaxisproperties.TickLabelInterpreter = 'latex';
yaxisproperties= get(gca, 'YAxis');
yaxisproperties.TickLabelInterpreter = 'latex';

% ylim([-pi, pi])
% ylim([-3*pi/2, pi/2])
yticks([-3*pi/2 -pi, -pi/2, 0, pi/2, pi]);
yticklabels({"$-\frac{3\pi}{2}$","$-\pi$", "$-\frac{\pi}{2}$", "$0$", "$\frac{\pi}{2}$", "$\pi$"})

colors = colororder();
text(0.23, 0,    "$q_1$", Interpreter="latex", FontSize=2*8, color = colors(1,:))
text(0.23, -2.7, "$q_2$", Interpreter="latex", FontSize=2*8, color = colors(2,:))
text(0.23, -1.6, "$q_3$", Interpreter="latex", FontSize=2*8, color = colors(3,:))


annotation('arrow', [0.4 0.4], [0.65 0.79], color = colors(1,:));

annotation('arrow', [0.4 0.4], [0.51 0.55], color = colors(3,:));

annotation('arrow', [0.4 0.4], [0.38 0.3], color = colors(2,:));

% legend(string(1:3))
%%
diagrams.save(gcf, 'opt_before_after_q_3R')