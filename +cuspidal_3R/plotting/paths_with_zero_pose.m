kin = cuspidal_3R.get_kin();
q = zeros([3 1]);
cuspidal_3R.initial_poses;
cuspidal_3R.optimized_poses;

%% Helix
[~, p_path_orig] = example_toolpath.helix();
pose_0 = helix_pose_0;
optimized = helix_optimized;

%% moveL
[~, p_path_orig] = example_toolpath.moveL([], [], [0;0;0], [0;0;0.5], [], [], 100);
pose_0 = moveL_pose_0;
optimized = moveL_optimized;

%%

diagrams.setup(); hold on
diagrams.robot_plot(kin, q);

plot_helix_path(pose_0(1:3), pose_0(4:6), p_path_orig, 'color', diagrams.colors.red)
plot_helix_path(optimized(1:3), optimized(4:6), p_path_orig, 'color', diagrams.colors.green)

grid on
axis on

diagrams.redraw(); hold off

function plot_helix_path(p, quat_XY, p_path_orig, varargin)
    [~, p_path] = graph_path_planning.quat_offset_path(p, quat_XY, [], p_path_orig);
    diagrams.utils.plot3_mat(p_path, varargin{:});
end