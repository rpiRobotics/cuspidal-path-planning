% Yumi with fixed q3
% from subproblem-polynomial repo

kin_7 = define_yumi;

kin = fwdkin_partial(kin_7, pi/2, 3);

[R, p] = fwdkin(kin, zeros([6 1]));

Q = IK.IK_4_6_intersecting(R, p, kin)

[R_t, p_t] = fwdkin(kin, Q(:,1))

%%

% Random pose

% q = rand_angle([6 1]); % [ 1.7871   -1.3096    0.6505    2.9181   -0.4242    1.2237]
q = [ 1.7871   -1.3096    0.6505    2.9181   -0.4242    1.2237]'
[R, p] = fwdkin(kin, q);

% All IK solns
Q = IK.IK_4_6_intersecting(R, p, kin)
%

% sgn(det(J)) for each soln
signs = NaN([1 width(Q)]);
for i = 1:numel(signs)
    J = robotjacobian(kin, Q(:,i));
    signs(i) = sign(det(J));
end

idx_pos = find(signs>0);
idx_neg = find(signs<0);
signs

%% Path between 2 solns
q_A = Q(:,idx_pos(1)); % [   -1.6884    1.2167   -2.9637   -0.1269   -0.9153    0.9605]
q_B = Q(:,idx_pos(2)); % [1.7871   -1.3096    0.6505    2.9181   -0.4242    1.2237]


N = 100;
lambda = linspace(0, 1,  N);
q_path = lambda.*q_B + (1-lambda).*q_A;
det_path = NaN(1,N);
for i = 1:N
    J = robotjacobian(kin, q_path(:,i));
    det_path(i) = det(J);

    % J = robotjacobian(kin_7, [q_path(1:2,i); pi/2 ;q_path(3:end,i)]);
    % det_path(i) = min((svd(J)));
end

plot(lambda, det_path)
xlabel("\lambda")
ylabel("det(J)")
yline(0)

%% Paths for all positive and all negative solutions
N = 100;
lambda = linspace(0, 1,  N);

if numel(idx_pos) == 4
q_A_list = [Q(:,idx_pos(1)) Q(:,idx_pos(1)) Q(:,idx_pos(1)) Q(:,idx_neg(1)) Q(:,idx_neg(1)) Q(:,idx_neg(1))];
q_B_list = [Q(:,idx_pos(2)) Q(:,idx_pos(3)) Q(:,idx_pos(4)) Q(:,idx_neg(2)) Q(:,idx_neg(3)) Q(:,idx_neg(4))];
elseif numel(idx_pos) == 3
q_A_list = [Q(:,idx_pos(1)) Q(:,idx_pos(1)) Q(:,idx_neg(1)) Q(:,idx_neg(1))];
q_B_list = [Q(:,idx_pos(2)) Q(:,idx_pos(3)) Q(:,idx_neg(2)) Q(:,idx_neg(3))];
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


plot(lambda, det_path_mat')
xlabel("\lambda")
ylabel("det(J)")
yline(0);

%% Iterate through psi = q_3

N_1 =50;
N_2 = 170;
q3_path_1 = linspace(pi/2, pi, 50+1);
q3_path_1 = q3_path_1(1:end-1);
q3_path_2 = linspace(pi/2, -pi,N_2+1);
q3_path_2 = q3_path_2(1:end-1);
Q_path_1 = NaN([6, N_1]);
Q_path_1(:,1) = q_B;
Q_path_2 = NaN([6, N_2]);
Q_path_2(:,1) = q_B;
det_path_1 = NaN(1,N_1);
det_path_2 = NaN(1,N_2);

for i = 2:N_1
    disp(i);
    kin_i = fwdkin_partial(kin_7, q3_path_1(i), 3);
    Q_i =  IK.IK_4_6_intersecting(R, p, kin_i);
    Q_path_1(:,i) = closest_q(Q_i, Q_path_1(:,i-1));

    % J = robotjacobian(kin_i, Q_path(:,i));
    % det_path(i) = det(J);
    J = robotjacobian(kin_7, [Q_path_1(1:2,i); q3_path_1(i) ;Q_path_1(3:end,i)]);
    det_path_1(i) = min((svd(J)));
end

for i = 2:N_2
    disp(i);
    kin_i = fwdkin_partial(kin_7, q3_path_2(i), 3);
    Q_i =  IK.IK_4_6_intersecting(R, p, kin_i);
    Q_path_2(:,i) = closest_q(Q_i, Q_path_2(:,i-1));

    % J = robotjacobian(kin_i, Q_path(:,i));
    % det_path(i) = det(J);
    J = robotjacobian(kin_7, [Q_path_2(1:2,i); q3_path_2(i) ;Q_path_2(3:end,i)]);
    det_path_2(i) = min((svd(J)));
end
%%
q3_path = [q3_path_1 q3_path_2];
Q_path = [Q_path_1, Q_path_2];
plot(q3_path, Q_path', 'x')

%%
plot(q3_path, [det_path_1, det_path_2])
xlabel("\lambda")
ylabel("det(J)")

%% what is sign(det(J)) when we fix each joint?

J = robotjacobian(kin_7, [q_A(1:2); pi/2 ; q_A(3:end)]);
sign_vec = NaN([7 1]);
for i = 1:7
    % sign_vec(i) = sign(det(J(:, ~(1:7 ==i))));
    sign_vec(i) = (det(J(:, ~(1:7 ==i))));
end
sign_vec

%%
kin_i = fwdkin_partial(kin_7, -pi+0.01, 3);
    Q_i =  IK.IK_4_6_intersecting(R, p, kin_i);

%% Fully plot of all IK solutions showing all self-motion manifolds
% Iterate through psi = q_3

N = 1000;
q3_path = linspace(-pi, pi, N);
Q_SM = [];

for i = 1:N
    disp(i);
    kin_i = fwdkin_partial(kin_7, q3_path(i), 3);
    Q_i =  IK.IK_4_6_intersecting(R, p, kin_i);
    if ~isempty(Q_i)
        Q_i_7 = [Q_i(1:2,:); q3_path(i)*ones([1 width(Q_i)]);Q_i(3:6,:)];
        Q_SM = [Q_SM Q_i_7];
    end
end
%%
scatter( (Q_SM(1, :)), (Q_SM(2, :)), [], Q_SM(3,:), '.'); hold on
scatter(Q(1,idx_pos), Q(2,idx_pos), 200, 'rx'); 
scatter(Q(1,idx_neg), Q(2,idx_neg), 200, 'kx');
plot(q_path(1,:), q_path(2,:), 'k');
hold off
colormap hsv
xlabel("q_1")
ylabel("q_2")
axis tight