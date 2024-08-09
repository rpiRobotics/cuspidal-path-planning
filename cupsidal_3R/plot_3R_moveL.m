
kin = define_cuspidal_3R();
q = zeros([3 1]);

diagrams.setup(); hold on


% Plot the 3R robot
diagrams.robot_plot(kin, q);

% Plot the initial moveL trajectory
diagrams.line([0.2;0.2;0.2], calc_p_B([0.2;0.2;0.2], [0;-1;0]), 'color', diagrams.colors.red);

% Plot optimized
% 1.2849   -2.2780    0.2576    3.1405    0.0023
%diagrams.line([1.2849;-2.2780;0.2576], calc_p_B([1.2849;-2.2780;0.2576], 3.1405, 0.0023), 'color', diagrams.colors.green);

%  4.5018   -0.0931    0.0014   -0.0187   15.9194    0.0427
diagrams.line([4.5018   -0.0931    0.0014], calc_p_B([4.5018   -0.0931    0.0014], [-0.0187   15.9194    0.0427]), 'color', diagrams.colors.green);
diagrams.redraw(); hold off


%% Make sure intial pose matches before and after reparameterization
% [0.2;0.2;0.2; pi/2; 0]
calc_p_B_old([0.2;0.2;0.2], pi/2, 0)
calc_p_B    ([0.2;0.2;0.2], [0;-1;0])


function p_B = calc_p_B(p_A, axang)
    p_B = p_A + 0.5* axang/norm(axang);
end

function p_B = calc_p_B_old(p_A, theta_x, theta_y)
    ex = [1;0;0];
    ey = [0;1;0];
    ez = [0;0;1];
    p_B = p_A + rot(ex, theta_x) * rot(ey, theta_y) * 0.5*ez;
end