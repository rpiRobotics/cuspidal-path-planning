pose_A = load("+cuspidal_3R\plotting\opt_history_3R_A.mat");
pose_B = load("+cuspidal_3R\plotting\opt_history_3R_B.mat");

h_fig = figure(10);
tiledlayout(1,1,'TileSpacing','none','Padding','compact');
nexttile
figure_size = 2*[2.5 2.45];
set(h_fig, "Units", "inches")
pos_old = h_fig.OuterPosition;
if ~all(pos_old(3:4) == figure_size)
set(h_fig, "OuterPosition", [pos_old(1:2)-figure_size+pos_old(3:4) figure_size])
end
set(h_fig, "Units", "pixels")
findfigs


% Important!! Make sure x values are >0 for log plot
semilogx(pose_A.history(1,:)+1, calc_RMS(pose_A.history(2,:)), '.-k'); hold on
semilogx(pose_B.history(1,:)+1, calc_RMS(pose_B.history(2,:)), '.-', Color=diagrams.colors.dark_red);
hold off



xlabel("Iteration (Log scale)", Interpreter='latex');
ylabel("RMS Error (rad/m)", Interpreter='latex');

fontsize(2*8, 'points')
xaxisproperties= get(gca, 'XAxis');
xaxisproperties.TickLabelInterpreter = 'latex';
yaxisproperties= get(gca, 'YAxis');
yaxisproperties.TickLabelInterpreter = 'latex';

%%
diagrams.save(gcf, 'opt_history_3R')

%%
function RMS = calc_RMS(W)

N = 500;

[~, p_path_orig] = example_toolpath.helix(H=1.2, R = 0.4, N=N);

DeltaT = diff(p_path_orig')';
L = sum(vecnorm(DeltaT));
DeltaLambda = L/(N-1);

C = W / DeltaLambda;

RMS = sqrt(C/L);
end