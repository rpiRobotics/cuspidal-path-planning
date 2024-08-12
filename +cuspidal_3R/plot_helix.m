orig = [0.7013    1.8369   -0.6936   -1.6368   -0.5241    1.5935]';
optimized = [0.8462    2.2120    0.0234    0.1465   -2.5520   -2.4581]';

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

function plot_helix_path(p, axang, p_path_orig, varargin)
    R = rot(axang/norm(axang), norm(axang));
    p_path = p + R*p_path_orig;
    diagrams.utils.plot3_mat(p_path, varargin{:});
end