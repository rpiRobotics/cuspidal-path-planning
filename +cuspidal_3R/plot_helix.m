pose_0 = [0.7013    1.8369   -0.6936  0.3884   -0.6435   -0.2061]';
optimized =[-0.19328340028517898563364951769472, 0.15472249611146848824461130789132, -4.4200684503758260746053565526381, 1716.7136288001620414434000849724, 1277.3946329562686514691449701786, -1143.6160441060003449820214882493]';

kin = cuspidal_3R.get_kin();
q = zeros([3 1]);
p_path_orig = example_toolpath.helix();

diagrams.setup(); hold on
diagrams.robot_plot(kin, q);

plot_helix_path(orig(1:3), orig(4:6), p_path_orig, 'color', diagrams.colors.red)
plot_helix_path(optimized(1:3), optimized(4:6), p_path_orig, 'color', diagrams.colors.green)

grid on
axis on

diagrams.redraw(); hold off

function plot_helix_path(p, quat_XY, p_path_orig, varargin)
    p_path = cuspidal_3R.quat_offset_path(p, quat_XY, p_path_orig);
    diagrams.utils.plot3_mat(p_path, varargin{:});
end