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

 % TODO convert to RMS?
plot(pose_A.history(1,:), pose_A.history(2,:), '.-k'); hold on
plot(pose_B.history(1,:), pose_B.history(2,:), '.-', Color=diagrams.colors.dark_red); hold off



xlabel("Iteration", Interpreter='latex');
ylabel("Error", Interpreter='latex'); % TODO label units

fontsize(2*8, 'points')
xaxisproperties= get(gca, 'XAxis');
xaxisproperties.TickLabelInterpreter = 'latex';
yaxisproperties= get(gca, 'YAxis');
yaxisproperties.TickLabelInterpreter = 'latex';

%%
diagrams.save(gcf, 'opt_history_3R')