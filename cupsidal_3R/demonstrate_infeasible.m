kin = define_cuspidal_3R;

% p_1 = [1; 0; 0]; % 0 feasible
% p_2 = [4; 0; 0];

p_1 = [2; 0; 0]; % 2 feasible
p_2 = [1; 0; 0];

N = 1e3;
lambda = linspace(0, 1, N);
p_path = lambda .* p_2 + (1 - lambda) .* p_1;


q1_path = NaN(4, N);
q2_path = NaN(4, N);
q3_path = NaN(4, N);
for i=1:N
    Q_i = IK_3R(p_path(:, i), kin);
    q1_path(1:width(Q_i), i) = Q_i(1,:);
    q2_path(1:width(Q_i), i) = Q_i(2,:);
    q3_path(1:width(Q_i), i) = Q_i(3,:);
end

% plot(lambda, q1_path' , '.k');
plot(lambda, q1_path');
ylim([-pi, pi]);
ylabel("q_1");
xlabel("\lambda");