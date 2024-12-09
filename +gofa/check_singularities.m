q = rand_angle([7 1]);

q(4) = 0;

J = robotjacobian(kin, q);

svd(J)


%%
diagrams.setup; hold on

diagrams.robot_plot(kin, q, auto_scale=true)

diagrams.redraw(); hold off