pose_0 = [0.2;0.2;0.2; 1;0;0];
optimized = [1.7651    1.9297   -0.2462   -6.2101   -0.0094    0.0115]';

plot_moveL_path(pose_0(1:3), pose_0(4:6), 'color', diagrams.colors.red); hold on
plot_moveL_path(optimized(1:3), optimized(4:6), 'color', diagrams.colors.green)


fp1 = fplot(r12(1), z12(1), 'k'); hold on
fp2 = fplot(r12(2), z12(2), 'k'); hold off

fp1.MeshDensity = 100;
fp2.MeshDensity = 100;

axis equal
hold off
xlabel("r")
ylabel("z")


function plot_moveL_path(p, quat_XY, varargin)
    R = quat2rotm([quat_XY' 0]); % Auto-normalize

    p_A = R*p;
    p_B = R*(p + [0;0;0.5]);

    N = 100;
    p_path = cuspidal_3R.generate_moveL(p_A, p_B, N);

    r = vecnorm(p_path(1:2,:));
    z = p_path(3,:);
    plot(r,z, varargin{:});
end