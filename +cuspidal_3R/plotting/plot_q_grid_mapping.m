syms q2_sym q3_sym real

kin = cuspidal_3R.get_kin;

q1 = 0;

[~, p] = fwdkin(kin, [q1; q2_sym; q3_sym]);
r = sqrt(p(1)^2+p(2)^2);
z = p(3);

r = simplify(r);
z = simplify(z);
    
%%
J = cuspidal_3R.robotjacobian_sym(kin, [q1; q2_sym; q3_sym]);
Jp = J(4:6,:);
dJ = det(Jp);
dJ = simplify(dJ);

% fimplicit(dJ)

%%
q2 = pi/6;
dJ_i = subs(dJ, q2_sym, q2)
q3_bounds = solve(dJ, q3_sym, Real=true)
q2_bounds = solve(dJ, q2_sym, Real=true)

%% Infeasible path
infeasible.p_A = [1; 0; 0];
infeasible.p_B = [4; 0; 0];

N = 1e2;
lambda = linspace(0, 1, N);
p_path = lambda .* infeasible.p_B + (1 - lambda) .* infeasible.p_A;


q1_path = NaN(4, N);
q2_path = NaN(4, N);
q3_path = NaN(4, N);
for i=1:N
    Q_i = cuspidal_3R.IK(p_path(:, i), kin);
    q1_path(1:width(Q_i), i) = Q_i(1,:);
    q2_path(1:width(Q_i), i) = Q_i(2,:);
    q3_path(1:width(Q_i), i) = Q_i(3,:);
end

%% Nonsingular change of solution

nonsingular.p = [2.5;0;0.5];
nonsingular.Q = cuspidal_3R.IK(nonsingular.p, kin);
nonsingular.Q_A = nonsingular.Q(:,2);
nonsingular.Q_B = nonsingular.Q(:,3);
N = 1e2;
lambda = linspace(0, 1, N);
Q_path = lambda .* nonsingular.Q_B + (1 - lambda) .* nonsingular.Q_A;

for i = 1:N
    [~, nonsingular.p_path(:,i)] = fwdkin(kin, Q_path(:,i));
end

nonsingular.z_path = nonsingular.p_path(3,:);
nonsingular.r_path = vecnorm(nonsingular.p_path([1 2],:));
%% joint space
% Fix q2, vary q3

h_fig = figure(10);
tiledlayout(1,1,'TileSpacing','none','Padding','compact');
nexttile
figure_size = 2*[2.5 3];
set(h_fig, "Units", "inches")
pos_old = h_fig.OuterPosition;
if ~all(pos_old(3:4) == figure_size)
set(h_fig, "OuterPosition", [pos_old(1:2)-figure_size+pos_old(3:4) figure_size])
end
set(h_fig, "Units", "pixels")
findfigs

N = 20;
q2_list = linspace(-pi, pi, N);
q3_max = NaN;
q3_min = NaN;

for i = 1:N
    q2 = q2_list(i);
    r_i = subs(r, q2_sym, q2);
    z_i = subs(z, q2_sym, q2);
    q3_bounds_i = double(subs(q3_bounds, q2_sym, q2));
    if q3_bounds_i(2) < q3_bounds_i(1)
        q3_bounds_i(2) = q3_bounds_i(2) + 2*pi;
    end
    % fplot(r_i, z_i, q3_bounds_i', color=diagrams.colors.red); 
    plot([q2 q2], q3_bounds_i, color=diagrams.colors.red);
    if i==1
        hold on
    end

    q3_max = max(q3_max, q3_bounds_i(2));
    q3_min = min(q3_min, q3_bounds_i(1));
end
    
% Fix q3, vary q2

N = 20;
q3_list = linspace(q3_min, q3_max, N);

for i = 1:N
    q3 = q3_list(i);
    r_i = subs(r, q3_sym, q3);
    z_i = subs(z, q3_sym, q3);
    q2_bounds_i = double(subs(q2_bounds, q3_sym, q3));
    if q3<0.32 % splits in 2
        q2_bounds_i_A = [-pi min(q2_bounds_i)];
        q2_bounds_i_B = [max(q2_bounds_i) pi];
        plot(q2_bounds_i_A, [q3 q3], color=diagrams.colors.green);
        plot(q2_bounds_i_B, [q3 q3], color=diagrams.colors.green);
        % q2_bounds_i(1) = q2_bounds_i(1)+2*pi;
    elseif q3 < 0.77 % span from -pi to pi
        q2_bounds_i = [-pi pi];
        plot(q2_bounds_i, [q3 q3], color=diagrams.colors.green);
    else
        plot(q2_bounds_i, [q3 q3], color=diagrams.colors.green);
    end

    % fplot(r_i, z_i, [min(q2_bounds_i) max(q2_bounds_i)], color=diagrams.colors.green); 
end

fplot(q3_bounds(1), [-pi, pi],'k');
fplot(q3_bounds(1)+pi, [-pi, pi],'k');

plot([nonsingular.Q_A(2) nonsingular.Q_B(2)], [nonsingular.Q_A(3) nonsingular.Q_B(3)], 'k', linewidth=1.5)


xlabel("$q_2$", Interpreter='latex');
ylabel("$q_3$", Interpreter='latex');
yticks([-pi -pi/2 0 pi/2 pi]);
yticklabels({'$-\pi$', '$-\pi/2$', '$0$', '$\pi/2$','$\pi$'})
xticks([-pi -pi/2 0 pi/2 pi]);
xticklabels({'$-\pi$', '$-\pi/2$', '$0$', '$\pi/2$','$\pi$'})

fontsize(2*8, 'points')
xaxisproperties= get(gca, 'XAxis');
xaxisproperties.TickLabelInterpreter = 'latex';
yaxisproperties= get(gca, 'YAxis');
yaxisproperties.TickLabelInterpreter = 'latex';


axis image
hold off
%%
diagrams.save(gcf, "joint_space_grid")

%% Task space
% Fix q2, vary q3

h_fig = figure(10);
tiledlayout(1,1,'TileSpacing','none','Padding','compact');
nexttile
figure_size = 2*[1.5 3];
set(h_fig, "Units", "inches")
pos_old = h_fig.OuterPosition;
if ~all(pos_old(3:4) == figure_size)
set(h_fig, "OuterPosition", [pos_old(1:2)-figure_size+pos_old(3:4) figure_size])
end
set(h_fig, "Units", "pixels")
findfigs

N = 20;
q2_list = linspace(-pi, pi, N);
q3_max = NaN;
q3_min = NaN;

for i = 1:N-1 % Don't double-draw +-pi line
    q2 = q2_list(i);
    r_i = subs(r, q2_sym, q2);
    z_i = subs(z, q2_sym, q2);
    q3_bounds_i = double(subs(q3_bounds, q2_sym, q2));
    if q3_bounds_i(2) < q3_bounds_i(1)
        q3_bounds_i(2) = q3_bounds_i(2) + 2*pi;
    end
    fplot(r_i, z_i, q3_bounds_i', color=diagrams.colors.red); 
    % plot([q2 q2], q3_bounds_i, color=diagrams.colors.red);
    if i==1
        hold on
    end

    q3_max = max(q3_max, q3_bounds_i(2));
    q3_min = min(q3_min, q3_bounds_i(1));
end
    
% Fix q3, vary q2

N = 20;
q3_list = linspace(q3_min, q3_max, N);

for i = 1:N
    q3 = q3_list(i);
    r_i = subs(r, q3_sym, q3);
    z_i = subs(z, q3_sym, q3);
    q2_bounds_i = double(subs(q2_bounds, q3_sym, q3));
    if q3<0.32 % splits in 2
        % q2_bounds_i_A = [-pi min(q2_bounds_i)];
        % q2_bounds_i_B = [max(q2_bounds_i) pi];
        % plot(q2_bounds_i_A, [q3 q3], color=diagrams.colors.green);
        % plot(q2_bounds_i_B, [q3 q3], color=diagrams.colors.green);
        q2_bounds_i(1) = q2_bounds_i(1)+2*pi;
    elseif q3 < 0.77 % span from -pi to pi
        q2_bounds_i = [-pi pi];
        % plot(q2_bounds_i, [q3 q3], color=diagrams.colors.green);
    else
        % plot(q2_bounds_i, [q3 q3], color=diagrams.colors.green);
    end

    fplot(r_i, z_i, [min(q2_bounds_i) max(q2_bounds_i)], color=diagrams.colors.green); 
end

r_sing_A = subs(r, q3_sym, q3_bounds(1));
z_sing_A = subs(z, q3_sym, q3_bounds(1));

r_sing_B = subs(r, q3_sym, q3_bounds(1)+pi);
z_sing_B = subs(z, q3_sym, q3_bounds(1)+pi);

fplot(r_sing_A, z_sing_A, 'k');
fplot(r_sing_B, z_sing_B, 'k'); 

diagrams.line(infeasible.p_A, infeasible.p_B)

plot(nonsingular.r_path, nonsingular.z_path, 'k', LineWidth=1.5);

xlabel("$\rho = \sqrt{x^2+y^2}$", Interpreter="latex");
ylabel("$z$", Interpreter="latex");

fontsize(2*8, 'points')
xaxisproperties= get(gca, 'XAxis');
xaxisproperties.TickLabelInterpreter = 'latex';
yaxisproperties= get(gca, 'YAxis');
yaxisproperties.TickLabelInterpreter = 'latex';

axis equal
hold off

%%
diagrams.save(gcf, "task_space_grid")