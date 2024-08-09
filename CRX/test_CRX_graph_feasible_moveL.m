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

[R_path, T_path] = generate_6DOF_moveL(R_06_A, R_06_B, p_0T_A, p_0T_B, N);
%%


Q_path = CRX_generate_Q_path(kin, R_path, T_path);
%%
plot(squeeze(Q_path(4,:,:))', 'k.')
xlabel("path index")
ylabel("q_4")
% ylim([0 pi])
ylim([-pi pi])
% yline(Q_path(4,  8  ,1), 'g');
% yline(Q_path(4,  9  ,end), 'r');

%% 
[G, start_nodes, end_nodes] = CRX_generate_path_graph(Q_path);

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
    for j = 1:12
        XData = [XData i];
        YData = [YData Q_path(4, j, i)];
    end
end
XData = [zeros([1 12]) XData];
YData = [zeros([1 12]) YData];
YData(isnan(YData)) = 0;
XData(isnan(XData)) = 0;

plot(G, XData=XData(1:6008), YData=YData(1:6008))

%%
P = shortestpath(G, 19, 6008)

%%
TR = shortestpathtree(G, 19)