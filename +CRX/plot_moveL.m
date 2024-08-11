kin = define_CRX;
q = zeros([6 1]);

diagrams.setup(); hold on

diagrams.robot_plot(kin, q, ...
    'unit_size', 0.1, ...
    'cyl_half_length', 0.1, ...
    'cyl_radius', 0.05);

initial = [0.2;0.2;0.2; 0;-1;0];
final = [1.1275   -0.0573   -0.2173   -0.0022   -0.0080    0.0004]';

plot_moveL_line(initial, {'color', diagrams.colors.red});
diagrams.text(initial(1:3), "Starting");

plot_moveL_line(final, {'color', diagrams.colors.green});
diagrams.text(final(1:3), "Optimized");
diagrams.redraw(); hold off
grid on
axis on

function plot_moveL_line(pose, opts)
    p_A = pose(1:3);
    axang = pose(4:6);
    p_B = p_A + 0.5* axang/norm(axang);
    diagrams.line(p_A, p_B, opts{:});
end