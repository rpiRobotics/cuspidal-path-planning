% Run plot_singularities.m first

%% Helix
[~, p_path_orig] = example_toolpath.helix();
pose_0 = helix_pose_0;
optimized = helix_optimized;

%% moveL
[~, p_path_orig] = example_toolpath.moveL([], [], [0;0;0], [0;0;0.5], [], [], 100);
pose_0 = moveL_pose_0;
optimized = moveL_optimized;
%%

plot_path(pose_0(1:3), pose_0(4:6), p_path_orig, 'color', diagrams.colors.red); hold on
plot_path(optimized(1:3), optimized(4:6), p_path_orig, 'color', diagrams.colors.green)

fp1 = fplot(r12(1), z12(1), 'k'); hold on
fp2 = fplot(r12(2), z12(2), 'k'); hold off

fp1.MeshDensity = 100;
fp2.MeshDensity = 100;

axis equal
hold off
xlabel("r")
ylabel("z")




function plot_path(p, quat_XY, p_path_orig, varargin)
    [~, p_path] = graph_path_planning.quat_offset_path(p, quat_XY, [], p_path_orig);

    r = vecnorm(p_path(1:2,:));
    z = p_path(3,:);
    plot(r,z, varargin{:});
end