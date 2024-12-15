% q_A = rand_angle([7 1])
% q_B = rand_angle([7 1])

q_A = [ 2.2584    1.9194    0.4821   -1.9923   -1.6341    2.4285   -2.9614]';
q_B = [-0.0635   -2.0865    3.0076    1.3364    0.0030   -0.1817   -2.7670]';

kin = robot_kin.yumi;

[R_A, T_A, psi_A] = yumi_FK(q_A, kin);
[R_B, T_B, psi_B] = yumi_FK(q_B, kin);

% SEW = sew_conv([0;0;1]);
% Q_i = SEW_IK.IK_gen_7_dof_mex(R_1, T_1, SEW, psi_1, kin)

N = 100;
[R_path, T_path, psi_path] = example_toolpath.moveL(R_A, R_B, T_A, T_B, psi_A, psi_B, N);



Q_path = yumi.generate_Q_path(kin, R_path, T_path, psi_path);
%%
plot(squeeze(Q_path(4,:,:))', 'k.')
xlabel("path index")
ylabel("q_4")
% ylim([0 pi])
ylim([-pi pi])
% yline(Q_path(4,  8  ,1), 'g');
% yline(Q_path(4,  9  ,end), 'r');

%%
[G, start_nodes, end_nodes] = graph_path_planning.generate_path_graph(Q_path)
%%

graph_path_planning.plot_path_graph(G, Q_path, 3)
%%
SEW = sew_conv([0;0;1]);
codegen +SEW_IK/IK_gen_7_dof.m -args {R_A, T_A, SEW, psi_A, kin}
%%

function [R, T, psi] = yumi_FK(q, kin)
    SEW = sew_conv([0;0;1]);
    
    [R, T, P_SEW_t] = fwdkin_inter(kin, q, [1 4 7]);
    psi = SEW.fwd_kin(P_SEW_t(:,1),P_SEW_t(:,2),P_SEW_t(:,3));
end