kin = define_CRX;

q_A = [-2.6111   -0.6297   -1.5088    1.8854   -0.4309    2.5802]'; % 4 solns

% 0 feasible paths:
% q_B = [-1.9990   -1.4841   -2.2271   -2.2866    2.3203    0.5008]'; % 8 soln

% 4 feasible paths:
q_B = [2.0323    1.2241   -1.1492    2.8288   -2.9252   -0.3849   -0.7442]'; % 12 soln 

% q_B = rand_angle([7 1])

[R_06_A, p_0T_A] = fwdkin(kin, q_A);
[R_06_B, p_0T_B] = fwdkin(kin, q_B);

N = 500;

[R_path, T_path] = generate_6DOF_moveL(R_06_A, R_06_B, p_0T_A, p_0T_B, N);

%%

[is_feasible, stop_at] = CRX_feasible_moveL(kin, R_path, T_path, 280)

%%

q4_path = NaN(16, N);
q5_path = NaN(16, N);
for i = 1:N
    [Q_LS, is_LS_vec] = IK_3_pairs_intersecting_LS_mex(R_path(:,:,i), T_path(:,i), kin, false);
    Q_i = Q_LS(:, ~any(is_LS_vec(1:2, :)));
    
    q4_path(1:width(Q_i),i) = Q_i(4,:);
    q5_path(1:width(Q_i),i) = Q_i(5,:);
end

plot(q4_path', 'k.')
xlabel("path index")
ylabel("q_4")
ylim([-pi pi])
%%
for i = 1:16
    scatter(1:N, q4_path(i,:), [], q5_path(i,:), '.'); hold on
end
hold off
colormap hsv
xlabel("path index")
ylabel("q_4")
ylim([-pi pi])

%%
i = 560
[Q, is_LS_vec] = IK_3_pairs_intersecting_LS_mex(R_path(:,:,i), T_path(:,i), kin, true)
% Q(:, ~any(is_LS_vec(1:2, :)))
Q(:, abs(is_LS_vec(end, :))<1e-4)