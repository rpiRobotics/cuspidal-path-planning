kin = define_CRX;
%%

for i = 1:1e3
    
R_A = rot(rand_normal_vec, rand_angle);
% R_A = eye(3);
R_B = R_A;

T_A = rand_vec;
T_A(3) = abs(T_A(3)) + 0.1;
T_B = rand_vec;
T_B(3) = abs(T_B(3)) + 0.1;

[R_path, T_path] = example_toolpath.moveL(R_A, R_B, T_A, T_B, [], [], 100);
if all(vecnorm(T_path([1 2], :)) > 0.2)
    break
end
end

% T_A = T_path(:,105)
% T_B = T_path(:,425)
%%
% N = 5000;
N = 200;
lambda = linspace(0,1,N);

extras = [446 1167 1880 2276]/5000;
lambda = sort([lambda extras]);
N = N + numel(extras);

[R_path, T_path] = example_toolpath.moveL(R_A, R_B, T_A, T_B, [], [], lambda);



Q_path = CRX.generate_Q_path(kin, R_path, T_path);
% plot(1:N, squeeze(Q_path(2,:,:))', 'k.')
    

plot(lambda, squeeze(Q_path(6,:,:))', 'k.'); hold on
plot(lambda, squeeze(Q_path(6,:,:))'+2*pi, 'r.');
plot(lambda, squeeze(Q_path(6,:,:))'-2*pi, 'r.'); hold off
ylim([-2*pi 2*pi])
yline(0);
%%

% Generate det(J) vs lambda
det_J_path = graph_path_planning.Q_to_det_J(Q_path, kin);

plot(lambda, det_J_path', 'k.'); yline(0);

%%
lambda = linspace(0, 1,N);
plot(lambda, squeeze(Q_path(6,:,:))', 'k.', MarkerSize=2); 
% plot(lambda, squeeze(Q_path(6,:,:))'+2*pi, 'k.', MarkerSize=2); hold off
ylim([-pi pi])
xlabel("\lambda")
ylabel("q_6")

%%
lambda = linspace(0, 1,N);
plot(lambda, squeeze(Q_path(4,:,:))', 'k.', MarkerSize=2); 
% plot(lambda, squeeze(Q_path(6,:,:))'+2*pi, 'k.', MarkerSize=2); hold off
ylim([-pi pi])
xlabel("\lambda")
ylabel("q_4")

%% Display q_roboguide
% J1, J2, J5 +/- 360°| J4, J6 +/- 380°| J3 +/-540°
W_A = Q_path(:,:,1);
W_A(3,:) = wrapToPi(W_A(3,:) - W_A(2,:));
rad2deg(W_A(:,3))
T_A

W_B = Q_path(:,:,end);
W_B(3,:) = wrapToPi(W_B(3,:) - W_B(2,:));
rad2deg(W_B(:,2))
T_B

%%
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
G = graph_path_planning.generate_path_graph(Q_path, thresh = 1, det_J_path=det_J_path);
graph_path_planning.diagrams_plot_path_graph(G, [], Q_path, 4, lambda=lambda, ...
    display_dots=false, skip_wraparound=true, lambda_add_vertical=extras, ...
    display_SF=false, LineWidth=1);

ylim([-pi pi]);
ylabel("$q_4$", Interpreter="latex");
xlabel("$\lambda/L$", Interpreter="latex");
yticks([-pi, -pi/2, 0, pi/2, pi])
xticks(0:0.25:1)
yticklabels({'$-\pi$', '$-\frac{\pi}{2}$', '$0$', '$\frac{\pi}{2}$', '$\pi$'})
fontsize(2*8, 'points')
xaxisproperties= get(gca, 'XAxis');
xaxisproperties.TickLabelInterpreter = 'latex';
yaxisproperties= get(gca, 'YAxis');
yaxisproperties.TickLabelInterpreter = 'latex';

yline(0);

%%
diagrams.save(gcf, "crx_ex4")