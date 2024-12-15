kin = cuspidal_3R.get_kin;

p_1 = [1; 0; 0]; % 0 feasible
p_2 = [4; 0; 0];

% p_1 = [2; 0; 0]; % 2 feasible
% p_2 = [1; 0; 0];

N = 100;
lambda = linspace(0, 1, N);

extras = [0.220822 0.638064];
lambda = sort([lambda extras]);
N = N + numel(extras);

p_path = lambda .* p_2 + (1 - lambda) .* p_1;


q1_path = NaN(4, N);
q2_path = NaN(4, N);
q3_path = NaN(4, N);
Q_path = NaN(3, 4, N);
for i=1:N
    Q_i = cuspidal_3R.IK(p_path(:, i), kin);
    q1_path(1:width(Q_i), i) = Q_i(1,:);
    q2_path(1:width(Q_i), i) = Q_i(2,:);
    q3_path(1:width(Q_i), i) = Q_i(3,:);
    Q_i = uniquetol(Q_i', 1e-2, ByRows=true)'; % Remove close doubles
    Q_path(:,1:width(Q_i),i) = Q_i;
end

q1_path(q1_path>2) = q1_path(q1_path>2)-2*pi; % Shift for better graph
Q_path(Q_path>2) = Q_path(Q_path>2)-2*pi; 
%% Generate det(J) vs lambda
det_J_path = graph_path_planning.Q_to_det_J(Q_path, kin);

plot(lambda, det_J_path', 'k.'); yline(0);
%%
h_fig = figure(10);
tiledlayout(1,1,'TileSpacing','none','Padding','compact');
nexttile

figure_size = 2*[1.5 2];
set(h_fig, "Units", "inches")
pos_old = h_fig.OuterPosition;
if ~all(pos_old(3:4) == figure_size)
    set(h_fig, "OuterPosition", [pos_old(1:2)-figure_size+pos_old(3:4) figure_size])
end
set(h_fig, "Units", "pixels")
findfigs

% plot(lambda, q1_path' , '.k');
G = graph_path_planning.generate_path_graph(Q_path, thresh = 2.5e-1);
graph_path_planning.diagrams_plot_path_graph(G, [], Q_path, 1, lambda=lambda, display_dots=false, display_SF=false, bold_det_J=det_J_path);
ylim([-5*pi/4, pi/4]);
ylabel("$q_1$", Interpreter="latex");
xlabel("$\lambda/L$", Interpreter="latex");


% yticks([-5*pi/4, -pi, -3*pi/4, -pi/2, -pi/4, 0, pi/4])
% yticklabels({'$-5\pi/4$', '$-\pi$', '$-3\pi/4$', '$-\pi/2$', '$-\pi/4$', '$0$', '$\pi/4$'})

yticks([-pi,  -pi/2, 0]);
yticklabels({ '$-\pi$', '$-\frac{\pi}{2}$',  '$0$'});

fontsize(2*8, 'points')

xaxisproperties= get(gca, 'XAxis');
xaxisproperties.TickLabelInterpreter = 'latex';
yaxisproperties= get(gca, 'YAxis');
yaxisproperties.TickLabelInterpreter = 'latex';

%%
diagrams.save(gcf, "3R_infeasible")


%% Manual plot with sign(det(J))

q1_disp = squeeze(Q_path(1,:,:));

q1_disp_pos = q1_disp;
q1_disp_pos(sign(det_J_path)<0) = NaN;

plot(lambda, q1_disp', 'k.'); hold on
plot(lambda, q1_disp_pos', 'kx'); hold off