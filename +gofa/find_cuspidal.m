% Random pose

found = false;
attempts = 1;

while ~found
q = round(rand_angle([6 1])*1e2)/1e2;
% q = 1:6
[R, p] = fwdkin(kin, q);

% All IK solns
Q = IK.IK_2_parallel_2_intersecting(R, p, kin);

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
% 
% if numel(idx_pos) == 4
% q_A_list = [Q(:,idx_pos(1)) Q(:,idx_pos(1)) Q(:,idx_pos(1)) Q(:,idx_pos(2)) Q(:,idx_pos(2)) Q(:,idx_pos(3)) Q(:,idx_neg(1)) Q(:,idx_neg(1)) Q(:,idx_neg(1)) Q(:,idx_neg(2)) Q(:,idx_neg(2)) Q(:,idx_neg(3))];
% q_B_list = [Q(:,idx_pos(2)) Q(:,idx_pos(3)) Q(:,idx_pos(4)) Q(:,idx_pos(3)) Q(:,idx_pos(4)) Q(:,idx_pos(4)) Q(:,idx_neg(2)) Q(:,idx_neg(3)) Q(:,idx_neg(4)) Q(:,idx_neg(3)) Q(:,idx_neg(4)) Q(:,idx_neg(4))];
% elseif numel(idx_pos) == 3
% q_A_list = [Q(:,idx_pos(1)) Q(:,idx_pos(1)) Q(:,idx_pos(2)) Q(:,idx_neg(1)) Q(:,idx_neg(1)) Q(:,idx_neg(2))];
% q_B_list = [Q(:,idx_pos(2)) Q(:,idx_pos(3)) Q(:,idx_pos(3)) Q(:,idx_neg(2)) Q(:,idx_neg(3)) Q(:,idx_neg(3))];
% elseif numel(idx_pos) == 2
% q_A_list = [Q(:,idx_pos(1))  Q(:,idx_neg(1))];
% q_B_list = [Q(:,idx_pos(2))  Q(:,idx_neg(2))];
% else
%     continue
% end


if numel(idx_pos) == 4
q_A_list = [Q(:,idx_pos(1)) Q(:,idx_pos(1)) Q(:,idx_pos(1)) Q(:,idx_pos(2)) Q(:,idx_pos(2)) Q(:,idx_pos(3))];
q_B_list = [Q(:,idx_pos(2)) Q(:,idx_pos(3)) Q(:,idx_pos(4)) Q(:,idx_pos(3)) Q(:,idx_pos(4)) Q(:,idx_pos(4))];
elseif numel(idx_pos) == 3
q_A_list = [Q(:,idx_pos(1)) Q(:,idx_pos(1)) Q(:,idx_pos(2))];
q_B_list = [Q(:,idx_pos(2)) Q(:,idx_pos(3)) Q(:,idx_pos(3))];
elseif numel(idx_pos) == 2
q_A_list = [Q(:,idx_pos(1))];
q_B_list = [Q(:,idx_pos(2))];
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

%% Double check

q_A = [-0.8000    0.5900    2.3400    2.7200    1.0600   -1.8400]'
q_B = [2.2599    2.1999    2.6677    2.5298   -2.5286    0.4831]'
%%
% 
% [R_1, p_1] = fwdkin(kin, q_A)
% [R_2, p_2] = fwdkin(kin, q_B)
N = 100;
lambda = linspace(0, 1,  N);
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
ylim([0, 0.045])

fontsize(2*8, 'points')
xaxisproperties= get(gca, 'XAxis');
xaxisproperties.TickLabelInterpreter = 'latex';
yaxisproperties= get(gca, 'YAxis');
yaxisproperties.TickLabelInterpreter = 'latex';
%%
diagrams.save(gcf, "gofa_det_J")

%%
diagrams.setup(); hold on

diagrams.robot_plot(kin, q_A, auto_scale=true, show_arrows = false, ...
    show_arrow_labels=false, show_joint_labels=false, show_dots=false, ...
    show_base_frame=true, show_task_frame=true, show_base_label=false, ...
    show_task_label=false); 
diagrams.robot_plot(kin, q_B, auto_scale=true, show_arrows = false, ...
    show_arrow_labels=false, show_joint_labels=false, show_dots=false, ...
    show_base_frame=false, show_task_frame=false, show_base_label=false, ...
    show_task_label=false, link_color=diagrams.colors.dark_green); 

view(118, 42);

diagrams.redraw(); hold off
%%
diagrams.save(gcf, 'gofa_bots')