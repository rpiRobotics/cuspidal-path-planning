% Random pose

kin = define_CRX;

found = false;
attempts = 1;

while ~found
q = rand_angle([6 1]);
% q = 1:6
[R, p] = fwdkin(kin, q);

% All IK solns
Q = IK_3_pairs_intersecting_mex(R, p, kin, false);

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

if numel(idx_pos) == 4
q_A_list = [Q(:,idx_pos(1)) Q(:,idx_pos(1)) Q(:,idx_pos(1)) Q(:,idx_pos(2)) Q(:,idx_pos(2)) Q(:,idx_pos(3)) Q(:,idx_neg(1)) Q(:,idx_neg(1)) Q(:,idx_neg(1)) Q(:,idx_neg(2)) Q(:,idx_neg(2)) Q(:,idx_neg(3))];
q_B_list = [Q(:,idx_pos(2)) Q(:,idx_pos(3)) Q(:,idx_pos(4)) Q(:,idx_pos(3)) Q(:,idx_pos(4)) Q(:,idx_pos(4)) Q(:,idx_neg(2)) Q(:,idx_neg(3)) Q(:,idx_neg(4)) Q(:,idx_neg(3)) Q(:,idx_neg(4)) Q(:,idx_neg(4))];
elseif numel(idx_pos) == 3
q_A_list = [Q(:,idx_pos(1)) Q(:,idx_pos(1)) Q(:,idx_pos(2)) Q(:,idx_neg(1)) Q(:,idx_neg(1)) Q(:,idx_neg(2))];
q_B_list = [Q(:,idx_pos(2)) Q(:,idx_pos(3)) Q(:,idx_pos(3)) Q(:,idx_neg(2)) Q(:,idx_neg(3)) Q(:,idx_neg(3))];
elseif numel(idx_pos) == 2
q_A_list = [Q(:,idx_pos(1))  Q(:,idx_neg(1))];
q_B_list = [Q(:,idx_pos(2))  Q(:,idx_neg(2))];
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

[R_1, p_1] = fwdkin(kin, q_A)
[R_2, p_2] = fwdkin(kin, q_B)

det_path = NaN(1,N);
q_path = lambda.*q_B + (1-lambda).*q_A;
for i = 1:N
    J = robotjacobian(kin, q_path(:,i));
    det_path(i) = det(J);
end
plot(det_path)

%% Try moveL
% TODO q_A and q_B need to be different poses

[R_path, T_path] = example_toolpath.moveL(R_06_A, R_06_B, p_0T_A, p_0T_B, [], [], N);