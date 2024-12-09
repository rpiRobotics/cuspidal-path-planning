kin = define_CRX;


% q_A = round(rand_angle([7,1])*10)/10;
% q_B = round(rand_angle([7,1])*10)/10;

%q_A = [ 1.0000    2.8000    1.4000   -0.6000    2.1000   -2.3000   -2.8000]';
%q_B = [-2.6000   -2.1000   -1.1000   -1.2000   -3.1000    0.3000   -2.5000]';
q_A = [   -2.8304    2.6087   -3.0211   -2.8021   -1.3803   -1.1154]';
q_B = [    1.0490    2.1330   -2.6428   -2.0174    2.4881    2.5346]';

[R_06_A, p_0T_A] = fwdkin(kin, q_A);
[R_06_B, p_0T_B] = fwdkin(kin, q_B);

N = 10;
N_HD = 500;

[R_path, T_path] = example_toolpath.moveL(R_06_A, R_06_B, p_0T_A, p_0T_B, [], [], N);
Q_path = CRX.generate_Q_path(kin, R_path, T_path);

[R_path_HD, T_path_HD] = example_toolpath.moveL(R_06_A, R_06_B, p_0T_A, p_0T_B, [], [], N_HD);
Q_path_HD = CRX.generate_Q_path(kin, R_path_HD, T_path_HD);



plot(linspace(1,N, N_HD), squeeze(Q_path_HD(4,:,:))', 'g.'); hold on
plot(1:N,                 squeeze(Q_path(4,:,:))', 'k.'); hold off
xlabel("path index")
ylabel("q_4")
ylim([-pi pi])

%%
plot(squeeze(Q_path(6,:,:))', 'k.')
xlabel("path index")
ylabel("q_6")
ylim([-pi pi])

%%
qi_path = squeeze(Q_path(6,:,:));

det_path = NaN(size(qi_path));

for i_lambda = 1:N
    for i_soln = 1:height(det_signs)
        q_i = Q_path(:,i_soln, i_lambda);
        J = robotjacobian(kin, q_i);
        det_path(i_soln, i_lambda) = det(J);
    end
end

qi_path_pos = qi_path;
qi_path_pos(det_path<0)= NaN;
qi_path_neg = qi_path;
qi_path_neg(det_path>=0) = NaN;

plot(qi_path_pos', '.r'); hold on
plot(qi_path_neg', '.k'); hold off
ylim([-pi pi])
%%
plot(det_path', '.k'); yline(0);


%%
G = graph_path_planning.generate_path_graph(Q_path); 
graph_path_planning.plot_path_graph(G, Q_path, 4)

%%
G = graph_path_planning.generate_path_graph(Q_path); 
[P, q_dot_norm] = shortestpath(G, 1, 2);

h_fig = figure(10);
tiledlayout(1,1,'TileSpacing','none','Padding','compact');
nexttile
figure_size = 2*[3.5 2.5];
set(h_fig, "Units", "inches")
pos_old = h_fig.OuterPosition;
if ~all(pos_old(3:4) == figure_size)
set(h_fig, "OuterPosition", [pos_old(1:2)-figure_size+pos_old(3:4) figure_size])
end
set(h_fig, "Units", "pixels")
findfigs


plot(linspace(1,N, N_HD), squeeze(Q_path_HD(4,:,:))', '.', color=0.8*diagrams.colors.white); hold on
graph_path_planning.diagrams_plot_path_graph(G, P, Q_path, 4)
ylabel("$q_4$", Interpreter="latex")
xlabel("$k$", Interpreter="latex")

% TODO show sign of det(J)

fontsize(2*8, 'points')
xaxisproperties= get(gca, 'XAxis');
xaxisproperties.TickLabelInterpreter = 'latex';
yaxisproperties= get(gca, 'YAxis');
yaxisproperties.TickLabelInterpreter = 'latex';

ylim([-pi, pi])
yticks([-pi, -pi/2, 0, pi/2, pi]);
yticklabels({"$-\pi$", "$-\frac{\pi}{2}$", "$0$", "$\frac{\pi}{2}$", "$\pi$"})

xticks([1:3:N])
xlim([-1, N+2])

text(0  ,0, "$S$", Interpreter="latex", FontSize = 2*10, HorizontalAlignment="right", VerticalAlignment="bottom");
text(N+1,0, "$F$", Interpreter="latex", FontSize = 2*10, HorizontalAlignment="left", VerticalAlignment="bottom");

%%
diagrams.save(gcf, "example_graph")