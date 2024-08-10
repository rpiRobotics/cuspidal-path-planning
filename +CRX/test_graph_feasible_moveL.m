kin = define_CRX;

q_B = [-2.6111   -0.6297   -1.5088    1.8854   -0.4309    2.5802]'; % 4 solns

% 0 feasible paths:
% q_B = [-1.9990   -1.4841   -2.2271   -2.2866    2.3203    0.5008]'; % 8 soln

% 4 feasible paths:
q_A = [2.0323    1.2241   -1.1492    2.8288   -2.9252   -0.3849   -0.7442]'; % 12 soln 

% q_B = rand_angle([7 1])

[R_06_A, p_0T_A] = fwdkin(kin, q_A);
[R_06_B, p_0T_B] = fwdkin(kin, q_B);

N = 500;

[R_path, T_path] = CRX.generate_6DOF_moveL(R_06_A, R_06_B, p_0T_A, p_0T_B, N);
%%
Q_path = CRX.generate_Q_path(kin, R_path, T_path);

%%
G = generate_path_graph(Q_path);

%%

plot_path_graph(G, Q_path, 4)
%%
plot(squeeze(Q_path(4,:,:))', 'k.')
xlabel("path index")
ylabel("q_4")
% ylim([0 pi])
ylim([-pi pi])
% yline(Q_path(4,  8  ,1), 'g');
% yline(Q_path(4,  9  ,end), 'r');



%%
P = shortestpath(G, 1, 2)