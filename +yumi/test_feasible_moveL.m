q_A = rand_angle([7 1])
q_B = rand_angle([7 1])

q_A = [ 2.2584    1.9194    0.4821   -1.9923   -1.6341    2.4285   -2.9614]';
q_B = [-0.0635   -2.0865    3.0076    1.3364    0.0030   -0.1817   -2.7670]';

kin = robot_kin.yumi;

[R_A, T_A, psi_A] = yumi_FK(q_A, kin);
[R_B, T_B, psi_B] = yumi_FK(q_B, kin);

SEW = sew_conv([0;0;1]);
% Q_i = SEW_IK.IK_gen_7_dof_mex(R_1, T_1, SEW, psi_1, kin)

N = 100;
[R_path, T_path, psi_path] = yumi.generate_7DOF_moveL(R_A, R_B, T_A, T_B, psi_A, psi_B, N);



Q_path = yumi.generate_Q_path(kin, R_path, T_path, psi_path);

plot(squeeze(Q_path(4,:,:))', 'k.')
xlabel("path index")
ylabel("q_4")
% ylim([0 pi])
ylim([-pi pi])
% yline(Q_path(4,  8  ,1), 'g');
% yline(Q_path(4,  9  ,end), 'r');

%%
[G, start_nodes, end_nodes] = yumi.generate_path_graph(Q_path)

%%
for i = 1:length(start_nodes)
    for j = 1:length(end_nodes)
        P = shortestpath(G, start_nodes(i), end_nodes(j));
        if ~isempty(P)
            disp(string(i)+ ", " + string(j))
        end
    end
end

%%
XData = [];
YData = [];
N = length(Q_path);
for i = 1:N
    for j = 1:64
        XData = [XData i];
        YData = [YData Q_path(4, j, i)];
    end
end
XData = [zeros([1 64]) XData];
YData = [zeros([1 64]) YData];
XData(isnan(YData)) = 0;
YData(isnan(YData)) = 0;


plot(G, XData=XData(1:6404), YData=YData(1:6404))
%%
codegen +SEW_IK/IK_gen_7_dof.m -args {R_A, T_A, SEW, psi_A, kin}
%%

function [R, T, psi] = yumi_FK(q, kin)
    SEW = sew_conv([0;0;1]);
    
    [R, T, P_SEW_t] = fwdkin_inter(kin, q, [1 4 7]);
    psi = SEW.fwd_kin(P_SEW_t(:,1),P_SEW_t(:,2),P_SEW_t(:,3));
end