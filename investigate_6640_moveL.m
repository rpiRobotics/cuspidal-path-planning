kin = hardcoded_IK_setups.IRB_6640.get_kin();
kin.P(:,end)=0;

% q_A = rand_angle([6 1]);
% q_B = rand_angle([6 1]);
q_A = [ 1.7798   -2.4257    3.0069    2.1903   -2.8234   -0.2124]';
q_B = [-1.0955    0.8181   -1.6946    0.5019    0.6482    0.6276]';

[R_A, T_A] = fwdkin(kin, q_A);
[R_B, T_B] = fwdkin(kin, q_B);

[R_path, T_path] = CRX.generate_moveL(R_A, R_B, T_A, T_B, 1000);

[Q_path, Q_path_LS] = generate_Q_path(kin, R_path, T_path);

q_disp_ind = 3;
plot(squeeze(Q_path   (q_disp_ind,:,:))', 'k.'); hold on
plot(squeeze(Q_path_LS(q_disp_ind,:,:))', 'r.'); hold off
ylabel("q_"+q_disp_ind);
xlabel("\lambda")

%%
z_path = T_path(3,:);
r_path = vecnorm(T_path(1:2,:));

center_z = kin.P(3,2);
center_r = kin.P(1,2);
plot(r_path, z_path, 'k'); hold on
plot(r_path(1), z_path(1), 'xk');

diagrams.circle([ center_r; center_z;0], [0;0;1], norm(kin.P(:,3)) + norm(kin.P(:,4)));
diagrams.circle([ center_r; center_z;0], [0;0;1], norm(kin.P(:,3)) - norm(kin.P(:,4)));

diagrams.circle([-center_r; center_z;0], [0;0;1], norm(kin.P(:,3)) + norm(kin.P(:,4)), 'color', 'blue');
diagrams.circle([-center_r; center_z;0], [0;0;1], norm(kin.P(:,3)) - norm(kin.P(:,4)), 'color', 'blue');

% diagrams.circle([ center_r; center_z;0], [0;0;1], norm(kin.P(:,3)), 'lineStyle', '--');

% plot circle for LHS workspace
axis equal
hold off
xlabel("r");
ylabel("z");
xline(0);
%%
diagrams.setup(); hold on;
diagrams.robot_plot(kin, zeros([6 1]));
diagrams.redraw(); hold off;

%%
function [Q_path, Q_path_LS] = generate_Q_path(kin, R_path, T_path)
N = width(T_path);

Q_path = NaN([6, 8, N]);
Q_path_LS = NaN([6, 8, N]);

for i = 1:N
    [Q_all, is_LS_vec] = IK.IK_spherical_2_parallel(R_path(:,:,i), T_path(:,i), kin);

    Q_i = Q_all;
    Q_LS = Q_all;
    Q_i(:,is_LS_vec==1) = NaN;
    Q_LS(:,is_LS_vec==0) = NaN;


    Q_path(:, 1:width(Q_i), i) = Q_i;
    Q_path_LS(:, 1:width(Q_i), i) = Q_LS;
end
end