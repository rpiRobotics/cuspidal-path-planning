%% Make sure kinematics match with RobotStudio

q = zeros(6,1);

diagrams.setup(); hold on

diagrams.robot_plot(kin, q, auto_scale = true);

diagrams.redraw(); hold off

%%
[R, p] = fwdkin(kin, q)
%%
[R, p] = fwdkin(kin, deg2rad(15)*ones(6,1))