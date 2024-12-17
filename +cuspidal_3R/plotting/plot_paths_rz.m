% Run plot_singularities.m first

cuspidal_3R.initial_poses()
cuspidal_3R.optimized_poses()
%% Helix
[~, p_path_orig] = example_toolpath.helix(H=1.2, R = 0.4);
pose_0_A = helix_pose_0_A;
optimized_A = helix_optimized_A;
pose_0_B = helix_pose_0_B;
optimized_B = helix_optimized_B;

%% moveL
[~, p_path_orig] = example_toolpath.moveL([], [], [0;0;0], [0;0;0.5], [], [], 100);
pose_0 = moveL_pose_0;
optimized = moveL_optimized;
%%

h_fig = figure(10);  delete(findall(gcf,'type','annotation'))
tiledlayout(1,1,'TileSpacing','none','Padding','compact');
nexttile
figure_size = 2*[1.5 2.45];
set(h_fig, "Units", "inches")
pos_old = h_fig.OuterPosition;
if ~all(pos_old(3:4) == figure_size)
set(h_fig, "OuterPosition", [pos_old(1:2)-figure_size+pos_old(3:4) figure_size])
end
set(h_fig, "Units", "pixels")
findfigs

plot_path(pose_0_A(1:3), pose_0_A(4:6), p_path_orig, 'color', diagrams.colors.red); hold on
plot_path(optimized_A(1:3), optimized_A(4:6), p_path_orig, 'color', diagrams.colors.green)
annotation('arrow', [0.48 0.5], [0.68 0.62])

plot_path(pose_0_B(1:3), pose_0_B(4:6), p_path_orig, 'color', diagrams.colors.red);
plot_path(optimized_B(1:3), optimized_B(4:6), p_path_orig, 'color', diagrams.colors.green)
annotation('arrow', [0.71 0.76], [0.62 0.57])

fplot(r_sing_A, z_sing_A, 'k');
fplot(r_sing_B, z_sing_B, 'k'); 

xlabel("$\rho = \sqrt{x^2+y^2}$", Interpreter="latex");
ylabel("$z$", Interpreter="latex");

yticks(-3:3)

fontsize(2*8, 'points')
xaxisproperties= get(gca, 'XAxis');
xaxisproperties.TickLabelInterpreter = 'latex';
yaxisproperties= get(gca, 'YAxis');
yaxisproperties.TickLabelInterpreter = 'latex';

axis equal
hold off

%%
diagrams.save(gcf, '3R_opt_rz')



%%
function plot_path(p, quat_XY, p_path_orig, varargin)
    [~, p_path] = graph_path_planning.quat_offset_path(p, quat_XY, [], p_path_orig);

    r = vecnorm(p_path(1:2,:));
    z = p_path(3,:);
    plot(r,z, varargin{:});
end