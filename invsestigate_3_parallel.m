

P = IK_setups.IK_3_parallel.setup;
kin = P.kin;
R = P.R;
T = P.T;

S = IK_setups.IK_3_parallel.run(P);
Q = S.Q;


%%
% Random pose

found = false;
attempts = 1;

while ~found
% P = IK_setups.IK_3_parallel.setup;
% P = IK_setups.IK_3_parallel_2_intersecting.setup;
% P = orthogonal_random_3_parallel();
S.Q = round(rand_angle([6 1])*10)/10;
[P.R, P.T] = fwdkin(P.kin, S.Q);

kin = P.kin;
R = P.R;
T = P.T;

S = IK_setups.IK_3_parallel.run(P);
% S = IK_setups.IK_3_parallel_2_intersecting.run(P);
Q = S.Q;


% sgn(det(J)) for each soln
signs = NaN([1 width(Q)]);
for i = 1:numel(signs)
    J = robotjacobian(kin, Q(:,i));
    signs(i) = sign(det(J));
end

idx_pos = find(signs>0);
idx_neg = find(signs<0);
% signs


% Paths for all positive and all negative solutions
N = 100;
lambda = linspace(0, 1,  N);

if numel(idx_pos)>1  & numel(idx_neg)>1
q_A_list = [Q(:,idx_pos(1))  Q(:,idx_neg(1))];
q_B_list = [Q(:,idx_pos(2))  Q(:,idx_neg(2))];
else
    continue
end



det_path_mat = NaN(width(q_A_list),N);
for i_pair = 1:width(q_A_list)
    q_A = q_A_list(:,i_pair);
    q_B = q_B_list(:,i_pair);
    q_path = lambda.*q_B + (1-lambda).*q_A;
    for i = 1:N
        J = robotjacobian(kin, q_path(:,i));
        det_path_mat(i_pair, i) = det(J);
    end
end



% Check if there exists a path with all the same sign
det_signs = sign(det_path_mat);
for i = 1:height(det_signs)
    if all(det_signs(i,1) == det_signs(i,:)) && all(abs(det_path_mat(i,:))>1e-3)
        % disp("got it!")
        % disp("attempts:" + attempts)
        disp(attempts)
        plot(lambda, det_path_mat'); hold on
        plot(lambda, det_path_mat(i,:), 'k', LineWidth=6); hold off
        xlabel("\lambda")
        ylabel("det(J)")
        yline(0);
        found = true;
        q_A = q_A_list(:,i)
        q_B = q_B_list(:,i) 
        break
    end
end

attempts = attempts + 1;
end


%%
ex = [1;0;0];
ey = [0;1;0];
ez = [0;0;1];
zv = [0;0;0];
P.kin.joint_type = zeros(1,6);
P.kin.P = ...
[        0    0.1000         0         0         0    0.3000         0
         0    0.7000         0         0         0         0         0.5
         0         0    0.7000    0.7000    0.7000    0.9000         0];
P.kin.H = [ez ey ey ey ex ey];

%%
q_A = [-2.4000   -0.9000    1.1000   -0.8000    2.3000   -1.3000]';
q_B = [ 0.9940   -1.4391    0.9530    1.2368    1.0004    1.5942]';

%%

diagrams.setup(); hold on

diagrams.robot_plot(P.kin, q_A, auto_scale=true, show_arrows = false, ...
    show_arrow_labels=false, show_joint_labels=false, show_dots=false, ...
    show_base_frame=true, show_task_frame=true, show_base_label=false, ...
    show_task_label=false); 
diagrams.robot_plot(P.kin, q_B, auto_scale=true, show_arrows = false, ...
    show_arrow_labels=false, show_joint_labels=false, show_dots=false, ...
    show_base_frame=false, show_task_frame=false, show_base_label=false, ...
    show_task_label=false, link_color=diagrams.colors.dark_green); 

view(43, 33);

diagrams.redraw(); hold off

%%
diagrams.save(gcf, '3_parallel_bots')

%%
q_A'
q_B'

%%
det_path = NaN(1,N);
q_path = lambda.*q_B + (1-lambda).*q_A;
for i = 1:N
    J = robotjacobian(kin, q_path(:,i));
    det_path(i) = det(J);
end



h_fig = figure(10);

figure_size = 2*[3.5 1.5];
set(h_fig, "Units", "inches")
pos_old = h_fig.OuterPosition;
if ~all(pos_old(3:4) == figure_size)
set(h_fig, "OuterPosition", [pos_old(1:2)-figure_size+pos_old(3:4) figure_size])
end
set(h_fig, "Units", "pixels")
findfigs

tiledlayout(1,1,'TileSpacing','none','Padding','compact');
nexttile

plot(lambda, det_path, 'k')
xlabel("$\lambda/L$", Interpreter="latex")
ylabel("$\det(J)$", Interpreter="latex")
ylim([-0.2, 0])

fontsize(2*8, 'points')
xaxisproperties= get(gca, 'XAxis');
xaxisproperties.TickLabelInterpreter = 'latex';
yaxisproperties= get(gca, 'YAxis');
yaxisproperties.TickLabelInterpreter = 'latex';
%%
diagrams.save(gcf, '3_parallel_det_J')
%%



function P = orthogonal_random_3_parallel()

ex = [1;0;0];
ey = [0;1;0];
ez = [0;0;1];
zv = [0;0;0];

S.Q = round(rand_angle([6 1])*10)/10;
P.kin.joint_type = zeros(1,6);

P.kin.H = [ez ey ey ey ex ey];

P.kin.P = [zv 0.1*ex+rand_tenths*ey rand_tenths*ez rand_tenths*ez rand_tenths*ez rand_tenths*ez+rand_tenths*ex zv];

[P.R, P.T] = fwdkin(P.kin, S.Q);

end

function R = rand_tenths()
    R = round(rand*10)/10;
end