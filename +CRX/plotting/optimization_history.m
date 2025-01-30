% RMS error vs optimization iteration

load("+CRX\plotting\opt_history_CRX_A.mat")
load("+CRX\plotting\opt_history_CRX_B.mat")

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
semilogx(history_A(1,:)+1, calc_RMS(history_A(2,:)), '.-', color = diagrams.colors.blue); hold on
semilogx(history_B(1,:)+1, calc_RMS(history_B(2,:)), '.-', Color=diagrams.colors.dark_green); hold off


xlabel("Iteration (Log scale)", Interpreter='latex');
ylabel("RMS Error (rad/m)", Interpreter='latex');

fontsize(2*8, 'points')
xaxisproperties= get(gca, 'XAxis');
xaxisproperties.TickLabelInterpreter = 'latex';
yaxisproperties= get(gca, 'YAxis');
yaxisproperties.TickLabelInterpreter = 'latex';

%%
diagrams.save(gcf, 'opt_history_CRX')

%%
function RMS = calc_RMS(W)

N = 500;

[~, p_path_orig] = example_toolpath.helix(N=N);

DeltaT = diff(p_path_orig')';
L = sum(vecnorm(DeltaT));
DeltaLambda = L/(N-1);

C = W / DeltaLambda;

RMS = sqrt(C/L);
end