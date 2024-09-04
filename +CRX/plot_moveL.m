kin = define_CRX;
q = zeros([6 1]);

diagrams.setup(); hold on

diagrams.robot_plot(kin, q, ...
    'unit_size', 0.1, ...
    'cyl_half_length', 0.1, ...
    'cyl_radius', 0.05);

initial = [0.2;0.2;0.2; 0;-1;0];
final = [1.1275   -0.0573   -0.2173   -0.0022   -0.0080    0.0004]';

p_A = plot_moveL_path(initial(1:3), initial(4:6), 'color', diagrams.colors.red);
diagrams.text(p_A, "Starting");

p_A = plot_moveL_path(final(1:3),final(4:6), 'color', diagrams.colors.green);
diagrams.text(p_A, "Optimized");
diagrams.redraw(); hold off
grid on
axis on

function p_A=plot_moveL_path(p, quat_XY, varargin)
    R = quat2rotm([quat_XY' 0]); % Auto-normalize

    p_A = R*p;
    p_B = R*(p + [0;0;0.5]);

    % N = 100;
    % p_path = cuspidal_3R.generate_moveL(p_A, p_B, N);
    p_path = [p_A p_B];

    diagrams.utils.plot3_mat(p_path, varargin{:});
end