kin = define_cuspidal_3R();
q = zeros([3 1]);

diagrams.setup(); hold on

% Plot the 3R robot
diagrams.robot_plot(kin, q);

% Plot the initial moveL trajectory
diagrams.line([0.2;0.2;0.2], calc_p_B([0.2;0.2;0.2], [0;-1;0]), 'color', diagrams.colors.red);

% Plot optimized
%  4.5018   -0.0931    0.0014   -0.0187   15.9194    0.0427
diagrams.line([4.5018   -0.0931    0.0014], calc_p_B([4.5018   -0.0931    0.0014], [-0.0187   15.9194    0.0427]), 'color', diagrams.colors.green);
diagrams.redraw(); hold off