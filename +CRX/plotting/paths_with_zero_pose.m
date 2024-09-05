kin = define_CRX;
q = zeros([6 1]);
CRX.initial_poses;
CRX.optimized_poses;

%% moveL

pose_0 = moveL_pose_0;
optimized = moveL_optimized;
[R_path_orig, p_path_orig] = example_toolpath.moveL(eye(3), eye(3), [0;0;0], [0;0;0.5], [], [], 100);

%% Helix

pose_0 = helix_pose_0;
optimized = helix_optimized;
[R_path_orig, p_path_orig] = example_toolpath.helix(200);


%%


diagrams.setup(); hold on

diagrams.robot_plot(kin, q, ...
    'unit_size', 0.1, ...
    'cyl_half_length', 0.1, ...
    'cyl_radius', 0.05);

p_A = plot_path(pose_0(1:3), pose_0(4:6), p_path_orig, 'color', diagrams.colors.red);
diagrams.text(p_A, "Starting");

p_A = plot_path(optimized(1:3),optimized(4:6), p_path_orig, 'color', diagrams.colors.green);
diagrams.text(p_A, "Optimized");
diagrams.redraw(); hold off
grid on
axis on

function p_A = plot_path(p, quat_XY, p_path_orig, varargin)
    [~, p_path] = graph_path_planning.quat_offset_path(p, quat_XY, [], p_path_orig);
    diagrams.utils.plot3_mat(p_path, varargin{:});
    p_A = p_path(:,1);
end