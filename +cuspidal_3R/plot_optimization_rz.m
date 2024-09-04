orig = [0.7013    1.8369   -0.6936   -1.6368   -0.5241    1.5935]';
optimized = [0.8462    2.2120    0.0234    0.1465   -2.5520   -2.4581]';

p_path_orig = example_toolpath.helix();

plot_helix_path(orig(1:3), orig(4:6), p_path_orig, 'color', diagrams.colors.red); hold on
plot_helix_path(optimized(1:3), optimized(4:6), p_path_orig, 'color', diagrams.colors.green)


fp1 = fplot(r12(1), z12(1), 'k'); hold on
fp2 = fplot(r12(2), z12(2), 'k'); hold off

fp1.MeshDensity = 100;
fp2.MeshDensity = 100;

axis equal
hold off
xlabel("r")
ylabel("z")




function plot_helix_path(p, axang, p_path_orig, varargin)
    R = rot(axang/norm(axang), norm(axang));
    p_path = p + R*p_path_orig;

    r = vecnorm(p_path(1:2,:));
    z = p_path(3,:);
    plot(r,z, varargin{:});
end