kin = define_CRX;
q = zeros([6 1]);
CRX.initial_poses;
CRX.optimized_poses;

%% moveL

pose_0 = moveL_pose_0;
optimized = moveL_optimized;
[R_path_orig, p_path_orig] = example_toolpath.moveL(eye(3), eye(3), [0;0;0], [0;0;0.5], [], [], 100);

%% Helix

pose_0_A = helix_pose_0_A;
optimized_A = helix_optimized_A;
pose_0_B = helix_pose_0_B;
optimized_B = helix_optimized_B;
[R_path_orig, p_path_orig] = example_toolpath.helix(N=200);


%% 3D plot


diagrams.setup(); hold on

diagrams.robot_plot(kin, q, ...
    'unit_size', 0.1, ...
    'cyl_half_length', 0.1, ...
    'cyl_radius', 0.05);

p_A = plot_path_3D(pose_0(1:3), pose_0(4:6), p_path_orig, 'color', diagrams.colors.red);
diagrams.text(p_A, "Starting");

p_A = plot_path_3D(optimized(1:3),optimized(4:6), p_path_orig, 'color', diagrams.colors.green);
diagrams.text(p_A, "Optimized");
diagrams.redraw(); hold off
grid on
axis on



%% 2D plot

h_fig = figure(10); delete(findall(gcf,'type','annotation'))
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
annotation('arrow', [0.61 0.57], [0.7 0.75])

plot_path(pose_0_B(1:3), pose_0_B(4:6), p_path_orig, 'color', diagrams.colors.red); hold on
plot_path(optimized_B(1:3), optimized_B(4:6), p_path_orig, 'color', diagrams.colors.green);
annotation('arrow', [0.45 0.5], [0.55 0.45])

zv = [0;0;0];
ez = [0;0;1];
diagrams.circle(zv, ez, 0.71+0.54);

xlabel("$\rho = \sqrt{x^2+y^2}$", Interpreter="latex");
ylabel("$z$", Interpreter="latex");

yticks(-3:3)

fontsize(2*8, 'points')
xaxisproperties= get(gca, 'XAxis');
xaxisproperties.TickLabelInterpreter = 'latex';
yaxisproperties= get(gca, 'YAxis');
yaxisproperties.TickLabelInterpreter = 'latex';


axis equal
xlim([0 1.75])
hold off

%%
diagrams.save(gcf, 'crx_opt_rz')


%%
function p_A = plot_path_3D(p, quat_XY, p_path_orig, varargin)
    [~, p_path] = graph_path_planning.quat_offset_path(p, quat_XY, [], p_path_orig);
    diagrams.utils.plot3_mat(p_path, varargin{:});
    p_A = p_path(:,1);
end

function plot_path(p, quat_XY, p_path_orig, varargin)
    [~, p_path] = graph_path_planning.quat_offset_path(p, quat_XY, [], p_path_orig);

    r = vecnorm(p_path(1:2,:));
    z = p_path(3,:);
    plot(r,z, varargin{:});
end