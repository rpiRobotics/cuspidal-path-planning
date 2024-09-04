pose_0 = [0.7013    1.8369   -0.6936  0.3884   -0.6435   -0.2061]';
optimized =[-0.19328340028517898563364951769472, 0.15472249611146848824461130789132, -4.4200684503758260746053565526381, 1716.7136288001620414434000849724, 1277.3946329562686514691449701786, -1143.6160441060003449820214882493]';

p_path_orig = example_toolpath.helix();

plot_helix_path(pose_0(1:3), pose_0(4:6), p_path_orig, 'color', diagrams.colors.red); hold on
plot_helix_path(optimized(1:3), optimized(4:6), p_path_orig, 'color', diagrams.colors.green)


fp1 = fplot(r12(1), z12(1), 'k'); hold on
fp2 = fplot(r12(2), z12(2), 'k'); hold off

fp1.MeshDensity = 100;
fp2.MeshDensity = 100;

axis equal
hold off
xlabel("r")
ylabel("z")




function plot_helix_path(p, quat_XY, p_path_orig, varargin)
    R = quat2rotm([quat_XY' 0]);
    p_path = R*(p + p_path_orig); % Careful - position then rotation

    r = vecnorm(p_path(1:2,:));
    z = p_path(3,:);
    plot(r,z, varargin{:});
end