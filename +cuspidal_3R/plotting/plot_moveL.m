pose_0 = [0.2;0.2;0.2; 1;0;0];
optimized = [1.7651    1.9297   -0.2462   -6.2101   -0.0094    0.0115]';

kin = cuspidal_3R.get_kin();
q = zeros([3 1]);

diagrams.setup(); hold on

diagrams.robot_plot(kin, q);

plot_moveL_path(pose_0(1:3), pose_0(4:6), 'color', diagrams.colors.red); hold on
plot_moveL_path(optimized(1:3), optimized(4:6), 'color', diagrams.colors.green)

diagrams.redraw(); hold off


function plot_moveL_path(p, quat_XY, varargin)
    R = quat2rotm([quat_XY' 0]); % Auto-normalize

    p_A = R*p;
    p_B = R*(p + [0;0;0.5]);

    % N = 100;
    % p_path = cuspidal_3R.generate_moveL(p_A, p_B, N);
    p_path = [p_A p_B];

    diagrams.utils.plot3_mat(p_path, varargin{:});
end