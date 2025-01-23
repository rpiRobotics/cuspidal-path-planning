% Run derive_kin for gofa 5 (robotstudio)
% TODO: convert MAT file with pose to matlab code

q_A = q_min + rand([6 1]).* (q_max-q_min)
q_B = q_min + rand([6 1]).* (q_max-q_min)

q_A(1) = 0;
q_B(1) = 0;
q_B(2) = q_A(2);
%%
[R_A, T_A] = fwdkin(kin, q_A);
[R_B, T_B] = fwdkin(kin, q_B);
%%
for i = 1:1e3

% R_A = R_6T;
% R_B = R_6T;
% R_A = eye(3);
% R_B = eye(3);


R_A = rot(rand_normal_vec, rand_angle);
R_B = R_A;


T_A = rand_vec; T_A(3) = abs(T_A(3));
T_B = rand_vec;
T_B(3) = abs(T_B(3));
% T_B(3) = abs(T_B(3));
% T_B(3) = T_A(3);


[R_path, T_path] = example_toolpath.moveL(R_A, R_B, T_A, T_B, [], [], 100);
if all(vecnorm(T_path([1 2], :)) > 0.2)
    break
end
end


% T_A = T_path(:,70)
% T_B = T_path(:,90)
%%
N = 200;
% lambda = linspace(0,1,N);

% extras = []/100

[R_path, T_path] = example_toolpath.moveL(R_A, R_B, T_A, T_B, [], [], N);



Q_path = gofa.generate_Q_path(kin, R_path, T_path);

%%
plot(1:N, rad2deg(squeeze(Q_path(4,:,:)))', 'k.'); hold on
plot(1:N, rad2deg(squeeze(Q_path(4,:,:)))'-360, 'r.'); hold off
yline(180);
yline(0);
yline(-180);
ylim([-180 180]);
%%
plot(1:N, rad2deg(squeeze(Q_path(3,:,:)))', 'k.'); hold on
plot(1:N, rad2deg(squeeze(Q_path(3,:,:)))'-360, 'r.'); hold off
yline(-225);
yline(85);

%%

% plot(1:N, squeeze(Q_path(5,:,:)+0.5*Q_path(1,:,:))', 'k.')
plot(1:N, squeeze(Q_path(6,:,:))', 'k.')
% xlim([1, N])
%%


%%
Q = IK.IK_2_parallel_2_intersecting(R_B, T_B, kin);

%%

diagrams.setup(); hold on

diagrams.robot_plot(kin, q_A, auto_scale = true);
diagrams.robot_plot(kin, q_B, auto_scale = true);


diagrams.redraw(); hold off

%%
kin = rmfield(kin, 'RT')
%%
codegen +IK/IK_2_parallel_2_intersecting.m -args {R_A, T_A, kin}

%% 1: Example 10

%%

lambda = linspace(0, 1,N);


h_fig = figure(10);
set(h_fig, 'renderer', 'painters')
tiledlayout(1,1,'TileSpacing','none','Padding','compact');
nexttile
% figure_size = 2*[3.5 3];
figure_size = 2*[2 2.5];
set(h_fig, "Units", "inches")
pos_old = h_fig.OuterPosition;
if ~all(pos_old(3:4) == figure_size)
set(h_fig, "OuterPosition", [pos_old(1:2)-figure_size+pos_old(3:4) figure_size])
end
set(h_fig, "Units", "pixels")
findfigs

% plot(lambda, squeeze(Q_path(4,:,:))', 'k.');
G = graph_path_planning.generate_path_graph(Q_path, thresh = (2.5e-1)^2);

G = addedge(G, 1862, 1861, 0); % vertical top
G = addedge(G, 1797, 1798, 0); % vertical bottom

G = addedge(G, 2438, 2437, 0);
G = addedge(G, 2898, 2900, 0);

G = addedge(G, 3033, 3032, 0);

graph_path_planning.diagrams_plot_path_graph(G, [], Q_path, 4, lambda=lambda, display_dots=false, display_SF=false);

ylim([-pi pi]);
ylabel("$q_4$", Interpreter="latex");
xlabel("$\lambda/L$", Interpreter="latex");
yticks([-pi, -pi/2, 0, pi/2, pi])
yticklabels({'$-\pi$', '$-\frac{\pi}{2}$', '$0$', '$\frac{\pi}{2}$', '$\pi$'})
fontsize(2*8, 'points')
xaxisproperties= get(gca, 'XAxis');
xaxisproperties.TickLabelInterpreter = 'latex';
yaxisproperties= get(gca, 'YAxis');
yaxisproperties.TickLabelInterpreter = 'latex';

yline(0);

%%
diagrams.save(gcf, "gofa_ex10")

%%
graph_path_planning.plot_path_graph(G, Q_path, 4, lambda)
    