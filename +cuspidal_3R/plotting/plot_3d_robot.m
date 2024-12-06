kin = cuspidal_3R.get_kin;

ex = [1;0;0];
ey = [0;1;0];
ez = [0;0;1];
zv = [0;0;0];

p = [2.5;0;0.5];
Q = cuspidal_3R.IK(p, kin);
q = Q(:,4);
q(1) = q(1) + deg2rad(20); % Add extra rotation

diagrams.setup([2 2]); hold on;

diagrams.robot_plot(kin, q, ...
    auto_scale=true, ...
    show_joint_labels=false, ...
    show_base_label = false, ...
    show_task_label = false, ...
    show_arrows = false, ...
    show_base_frame = false, ...
    show_task_frame = false, ...
    show_arrow_labels = false)

diagrams.arrow(zv, 4*ex);
diagrams.arrow(-3.5*ez, 3.5*ez);
diagrams.text(4*ex, "$\rho$", verticalAlignment='bottom', margin=0);
diagrams.text(3.5*ez, "$z$", verticalAlignment='middle', margin=5)

diagrams.dot(p);
diagrams.angle_arc(p, ez, ez*p(3), deg2rad(20));
diagrams.text(rot(ez, deg2rad(12))*p, "$\phi$", ...
    horizontalAlignment='center', verticalAlignment="bottom");

fplot3(r_sing_A, sym(0), z_sing_A, 'k'); hold on
fplot3(r_sing_B, sym(0), z_sing_B, 'k'); 

camva(9)
camtarget([2.25 0 -0.3])
diagrams.redraw; hold off;
%%
diagrams.save(gcf, "cupsidal_3R_3d")